#include <EEPROM.h>
#include <math.h>

// DRV8825 / PWM piny 
const int stepPin    = 12; // STEP
const int dirPin     = 13; // DIR
const int rst_slpPin = 11; // RESET/SLEEP (aktywny HIGH)

const int pwmPin     = 6;  // wyjście PWM do regulacji prądu
int pwmValue         = 0;  // setpoint: 0..32 (mA)
const int PWM_LIMIT  = 32; // maks. poziom prądu w mA

// Nextion: identyfikatory stron 
const byte MENU_PAGE_ID   = 0; // menu
const byte HEIGHT_PAGE_ID = 1; // wysokość
const byte PWM_PAGE_ID    = 2; // prąd (PWM)

// Silnik krokowy: dwa biegi 
const int SPEED_SLOW_US = 1328;
const int SPEED_FAST_US = 878;

const unsigned long DOUBLE_TAP_MS   = 350;
const unsigned long HOLD_TO_FAST_MS = 2000;

int  motorDelayUs       = SPEED_SLOW_US;
bool motorRunning       = false;
bool fastMode           = false;
unsigned long lastTapTime   = 0;
unsigned long pressStartMs  = 0;

// Auto-sleep sterownika 
unsigned long lastMotorActivity = 0;
const unsigned long SLEEP_DELAY_MS = 5000;
bool driverSleeping = false;

// EEPROM 
const int  EE_MAGIC_ADDR = 0;
const int  EE_PWM_ADDR   = 1;
const byte EE_MAGIC      = 0xA5;
uint8_t    savedPWMValue = 0;

// Maszyna stanów 
enum PageState { ST_MENU, ST_PWM, ST_HEIGHT };
PageState currentState = ST_MENU;

// Polling strony Nextion 
const unsigned long PAGE_POLL_MS = 150;
unsigned long lastPagePoll = 0;

// AMPEROMIERZ: pomiar na A3 
// RCURRENT (pad nie-GND) -> 1kΩ -> A3; przy A3 kondensator 10µF do GND.
const byte  I_SENSE_PIN        = A3;      // wejście ADC
const float R_SHUNT_OHM        = 56.0f;   // RCURRENT
const float ADC_VREF_V         = 5.0f;    // DEFAULT (AVcc ≈ 5 V)
const unsigned long SENSE_PERIOD_MS = 100;
unsigned long lastSenseMs      = 0;
float ema_A                    = 0.0f;    // filtr EMA
const float EMA_ALPHA          = 0.25f;   // 0..1 (większe = szybsza reakcja)

// Kalibracja wskazania A3 (z Twoich danych: multimetr vs A3) 
const float I_SCALE_A  = 0.996068f;   // skalowanie
const float I_OFFSET_A = 0.0000969f;  // [A] = 0.0969 mA

// Kalibracja sterowania (setpoint -> PWM) 
// Dopasowanie I = K*w + C  =>  w(s) = (s - C) / K   (Twoje dane 1..32 mA, OLS)
const float CAL_K_mA_per_pwm = 0.348357f;   // [mA / PWM]
const float CAL_C_mA         = 0.157477f;   // [mA]

// Narzędzia Nextion
void nextionEnd(){ Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF); } // terminator
void nextionCmd(const char* cmd){ Serial.print(cmd); nextionEnd(); }

// setpoint (mA) – pole 'currentText'
void updateCurrentText(int pwmVal){
  Serial.print("currentText.txt=\"");
  Serial.print(pwmVal);
  Serial.print(" mA\"");
  nextionEnd();
}

// format mA z przecinkiem (PL), jedna cyfra po przecinku 
String fmt_mA(float mA){
  if (mA < 0) mA = 0;            // kosmetyka: wartości ujemne wyświetlaj jako 0
  String s = String(mA, 1);      // np. 19,8 mA
  s.replace('.', ',');
  s += " mA";
  return s;
}

// Zerowanie wyświetlania (niezależne od nastawy) 
const float ZERO_CLAMP_MA     = 0.15f;     // próg wejścia w zero (mA)
const float ZERO_RELEASE_MA   = 0.25f;     // próg wyjścia z zera (mA) – histereza
const unsigned long ZERO_HOLD_MS = 600;    // czas stabilizacji, by zakotwiczyć (ms)
bool zeroLatched              = false;
unsigned long zeroStartMs     = 0;

// Zastępuje dawną wersję uzależnioną od pwmValue
float applyDisplayZeroClamp(float mA){
  unsigned long nowMs = millis();

  if (!zeroLatched) {
    if (fabsf(mA) < ZERO_CLAMP_MA) {
      if (zeroStartMs == 0) zeroStartMs = nowMs;
      if (nowMs - zeroStartMs >= ZERO_HOLD_MS) {
        zeroLatched = true;
        return 0.0f;
      }
    } else {
      zeroStartMs = 0;
    }
    return mA; // jeszcze nie zakotwiczone
  } else {
    // jesteśmy zakotwiczeni w 0, trzymaj 0 do przekroczenia progu zwolnienia
    if (fabsf(mA) > ZERO_RELEASE_MA) {
      zeroLatched = false;
      zeroStartMs = 0;
      return mA;
    }
    return 0.0f;
  }
}

// publikacja prądu rzeczywistego w mA – pole 'realText'
void showRealCurrent_mA(float I_A){
  float mA = I_A * 1000.0f;      // A -> mA
  mA = applyDisplayZeroClamp(mA); // sprytowe zero niezależnie od nastawy
  Serial.print("realText.txt=\"");
  Serial.print(fmt_mA(mA));
  Serial.print("\"");
  nextionEnd();
}

// odczyt surowy (A) – średnia z 16 próbek ADC
float readCurrent_A_raw(){
  uint32_t acc = 0;
  for (int i=0;i<16;i++) acc += analogRead(I_SENSE_PIN); // 0..1023 (10 bit)
  float adc = acc / 16.0f;
  float V   = adc * (ADC_VREF_V / 1023.0f);              // V na A3
  return (V / R_SHUNT_OHM);                              // I = V/R [A]
}

// Mapowanie setpoint -> analogWrite() (skalibrowane) 
int mapPWMToAnalog(int s_mA){
  float w = ((float)s_mA - CAL_C_mA) / CAL_K_mA_per_pwm; // ≈ 2.870615*s - 0.452056
  w = constrain(w, 0.0f, 255.0f);
  return (int)lroundf(w); // 8-bit PWM (0..255)
}

void applyPWMOutput(){
  if(currentState != ST_PWM){
    analogWrite(pwmPin, 0);
  }else{
    analogWrite(pwmPin, mapPWMToAnalog(pwmValue));
  }
}

// EEPROM 
void eepromInitAndLoad(){
  byte magic = EEPROM.read(EE_MAGIC_ADDR);
  if(magic != EE_MAGIC){
    EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC);
    EEPROM.update(EE_PWM_ADDR, 0);
    savedPWMValue = 0;
  }else{
    savedPWMValue = EEPROM.read(EE_PWM_ADDR);
    if(savedPWMValue > PWM_LIMIT) savedPWMValue = 0;
  }
}

void eepromSavePWM(uint8_t value){
  value = constrain(value, 0, PWM_LIMIT);
  EEPROM.update(EE_PWM_ADDR, value);
  savedPWMValue = value;
}

// Wejście/wyjście stanu 
void onEnterState(PageState st){
  switch(st){
    case ST_MENU:   break;
    case ST_PWM:
      pwmValue = savedPWMValue;
      updateCurrentText(pwmValue);   // setpoint (mA)
      break;
    case ST_HEIGHT: break;
  }
  applyPWMOutput();
}

void onExitState(PageState st){
  switch(st){
    case ST_MENU:   break;
    case ST_PWM:
      eepromSavePWM((uint8_t)pwmValue);
      pwmValue = 0;
      analogWrite(pwmPin, 0);
      break;
    case ST_HEIGHT: break;
  }
}

void switchState(PageState newState){
  if(newState == currentState) return;
  onExitState(currentState);
  currentState = newState;
  onEnterState(currentState);
}

PageState pageIdToState(byte pageID){
  if(pageID == PWM_PAGE_ID)    return ST_PWM;
  if(pageID == HEIGHT_PAGE_ID) return ST_HEIGHT;
  return ST_MENU;
}

// setup 
void setup(){
  Serial.begin(9600);           // Nextion na UART0
  // analogReference(DEFAULT);  // domyślnie ≈5 V (UNO)

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(rst_slpPin, OUTPUT);
  digitalWrite(rst_slpPin, HIGH);

  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 0);

  eepromInitAndLoad();

  // start na stronie PWM
  Serial.print("page 2"); nextionEnd();
  currentState = ST_PWM;
  onEnterState(currentState);
  nextionCmd("sendme");

  lastMotorActivity = millis();
  lastPagePoll      = millis();
  lastSenseMs       = millis();
}

// Obsługa Nextion 
void handleTouchEvent(byte pageID, byte compID, byte event){
  nextionCmd("sendme");

  if(currentState == ST_PWM && event == 1){
    // + (ID=2), − (ID=3)
    if(compID == 2 && pwmValue < PWM_LIMIT) pwmValue += 1;
    else if(compID == 3 && pwmValue > 0) pwmValue -= 1;

    pwmValue = constrain(pwmValue, 0, PWM_LIMIT);
    updateCurrentText(pwmValue); // setpoint
  }

  if(currentState == ST_HEIGHT){
    // GÓRA (ID=1) / DÓŁ (ID=2)
    if(compID == 1 || compID == 2){
      unsigned long nowMs = millis();
      if(event == 1){
        digitalWrite(dirPin, (compID == 1) ? HIGH : LOW);
        fastMode = (nowMs - lastTapTime <= DOUBLE_TAP_MS);
        lastTapTime   = nowMs;
        motorDelayUs  = fastMode ? SPEED_FAST_US : SPEED_SLOW_US;
        pressStartMs  = nowMs;
        motorRunning  = true;
      }else if(event == 0){
        motorRunning  = false;
      }
    }
  }
}

void handlePageEvent(byte pageID){ switchState(pageIdToState(pageID)); }

bool readNextionFrame(){
  static byte buf[16];
  static byte len = 0;

  while(Serial.available()){
    byte b = Serial.read();
    if(len == 0 && b != 0x65 && b != 0x66) continue; // touch/page
    buf[len++] = b;

    if(len >= 4 && buf[len-1]==0xFF && buf[len-2]==0xFF && buf[len-3]==0xFF){
      if(buf[0]==0x66 && len>=5){
        byte pageID = buf[1];
        handlePageEvent(pageID);
      }else if(buf[0]==0x65 && len>=7){
        byte pageID = buf[1];
        byte compID = buf[2];
        byte event  = buf[3]; // 1=press, 0=release
        handleTouchEvent(pageID, compID, event);
      }
      len = 0;
      return true;
    }
    if(len >= sizeof(buf)) len = 0;
  }
  return false;
}

void loop(){
  readNextionFrame();

  unsigned long now = millis();
  if(now - lastPagePoll >= PAGE_POLL_MS){
    nextionCmd("sendme");
    lastPagePoll = now;
  }

  // hold-to-fast
  if(motorRunning && !fastMode && (now - pressStartMs >= HOLD_TO_FAST_MS)){
    fastMode     = true;
    motorDelayUs = SPEED_FAST_US;
  }

  // generowanie kroków
  if(motorRunning){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorDelayUs);
    lastMotorActivity = now;
  }

  // sleep DRV8825
  bool shouldSleep = (currentState != ST_HEIGHT) ||
                     (!motorRunning && (now - lastMotorActivity > SLEEP_DELAY_MS));
  if(shouldSleep && !driverSleeping){
    digitalWrite(rst_slpPin, LOW);
    driverSleeping = true;
  }else if(!shouldSleep && driverSleeping){
    digitalWrite(rst_slpPin, HIGH);
    driverSleeping = false;
  }

  // wyjście PWM
  applyPWMOutput();

  // Amperomierz: publikacja w mA na stronie PWM 
  if(currentState == ST_PWM && (now - lastSenseMs >= SENSE_PERIOD_MS)){
    float A_now = readCurrent_A_raw();
    A_now = A_now * I_SCALE_A + I_OFFSET_A;                 // korekcja pomiaru
    ema_A = EMA_ALPHA * A_now + (1.0f - EMA_ALPHA) * ema_A; // wygładzenie
    showRealCurrent_mA(ema_A);                              // realText (mA)
    lastSenseMs = now;
  }
}
