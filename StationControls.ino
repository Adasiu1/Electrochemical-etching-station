#include <EEPROM.h>

// DRV8825 / PWM piny
const int stepPin = 12; // pin STEP do sterownika
const int dirPin = 13; // pin DIR - kierunek obrotów
const int rst_slpPin = 11; // RESET/SLEEP pin (aktywny HIGH, budzimy sterownik)

const int pwmPin = 6; // wyjście PWM do regulacji prądu silnika
int pwmValue = 0; // aktualna wartość prądu 0..32 (mA)
const int PWM_LIMIT = 32; // maksymalny poziom prądu w mA
#define PWM_MAX_OUTPUT 90 // mapowanie na analogWrite (0-255)

// Nextion - identyfikatory stron
const byte MENU_PAGE_ID = 0; // strona główna menu
const byte HEIGHT_PAGE_ID = 1; // strona kontroli wysokości
const byte PWM_PAGE_ID = 2; // strona ustawień prądu

// Silnik krokowy - różne prędkości
const int SPEED_SLOW_US = 1666; // wolny bieg - mikrosekundy między zboczami
const int SPEED_FAST_US = 878; // szybki bieg - mikrosekundy między zboczami

const unsigned long DOUBLE_TAP_MS = 350; // okno czasowe na drugi klik (double-tap)
const unsigned long HOLD_TO_FAST_MS = 2000; // po tylu ms trzymania przełącz na szybki tryb

int motorDelayUs = SPEED_SLOW_US; // aktualny odstęp między krokami
bool motorRunning = false; // czy silnik się obraca
bool fastMode = false; // czy jedziemy w szybkim trybie
unsigned long lastTapTime = 0; // timestamp ostatniego wciśnięcia
unsigned long pressStartMs = 0; // kiedy zaczęto trzymać przycisk

// Automatyczne usypianie sterownika po bezczynności
unsigned long lastMotorActivity = 0; // ostatnia aktywność silnika
const unsigned long SLEEP_DELAY_MS = 5000; // po tylu ms bez ruchu usypiamy sterownik
bool driverSleeping = false; // czy sterownik śpi

// EEPROM - zapisywanie ustawień
const int EE_MAGIC_ADDR = 0; // adres magic byte w EEPROM
const int EE_PWM_ADDR = 1; // adres wartości PWM w EEPROM
const byte EE_MAGIC = 0xA5; // magic byte - sprawdza czy EEPROM jest zainicjowany
uint8_t savedPWMValue = 0; // ostatnio zapisana wartość prądu z EEPROM

// Maszyna stanów - śledzenie aktualnej strony
enum PageState { ST_MENU, ST_PWM, ST_HEIGHT };
PageState currentState = ST_MENU;

// Periodyczne sprawdzanie strony Nextion
const unsigned long PAGE_POLL_MS = 150; // co ile ms pytamy o aktualną stronę
unsigned long lastPagePoll = 0; // timestamp ostatniego pytania

// Pomocnicze funkcje do komunikacji z Nextion
void nextionEnd() { 
  // każda komenda Nextion kończy się trzema 0xFF
  Serial.write(0xFF); 
  Serial.write(0xFF); 
  Serial.write(0xFF); 
}

void nextionCmd(const char* cmd) { 
  // wyślij komendę do Nextion
  Serial.print(cmd); 
  nextionEnd(); 
}

void updateCurrentText(int pwmVal) {
  // aktualizuje tekst na wyświetlaczu z aktualną wartością prądu
  Serial.print("currentText.txt=\"");
  Serial.print(pwmVal);
  Serial.print(" mA\"");
  nextionEnd();
}

// Mapuje wartość mA (0-32) na analogWrite (0-PWM_MAX_OUTPUT)
int mapPWMToAnalog(int val) {
  val = constrain(val, 0, PWM_LIMIT);  // ogranicz do dozwolonego zakresu
  return (PWM_LIMIT > 0) ? map(val, 0, PWM_LIMIT, 0, PWM_MAX_OUTPUT) : 0;
}

// Reguła PWM: tylko na stronie PWM jest prąd, poza tym 0
void applyPWMOutput() {
  if(currentState != ST_PWM) {
    analogWrite(pwmPin, 0);  // poza stroną PWM zawsze 0 mA
  } else {
    // na stronie PWM aplikuj rzeczywistą wartość
    analogWrite(pwmPin, mapPWMToAnalog(pwmValue));
  }
}

// Inicjalizacja EEPROM i wczytanie zapisanych ustawień
void eepromInitAndLoad() {
  byte magic = EEPROM.read(EE_MAGIC_ADDR);
  if(magic != EE_MAGIC) {
    // pierwszego razu - inicjalizuj EEPROM
    EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC);
    EEPROM.update(EE_PWM_ADDR, 0);  // domyślnie 0 mA
    savedPWMValue = 0;
  } else {
    // wczytaj zapisaną wartość
    savedPWMValue = EEPROM.read(EE_PWM_ADDR);
    if(savedPWMValue > PWM_LIMIT) savedPWMValue = 0;  // sprawdź poprawność
  }
}

// Zapisz wartość PWM do EEPROM
void eepromSavePWM(uint8_t value) {
  value = constrain(value, 0, PWM_LIMIT); // zabezpiecz przed błędnymi wartościami
  EEPROM.update(EE_PWM_ADDR, value); // update zapisuje tylko jak się zmieniło
  savedPWMValue = value; // zapamiętaj aktualną zapisaną wartość
}

// Co robimy gdy wchodzimy na nową stronę
void onEnterState(PageState st) {
  switch(st) {
    case ST_MENU:
      // na menu nic specjalnego nie robimy
      break;
      
    case ST_PWM:
      // wchodzimy na stronę PWM - przywróć wartość z EEPROM i pokaż na wyświetlaczu
      pwmValue = savedPWMValue;
      updateCurrentText(pwmValue);
      break;
      
    case ST_HEIGHT:
      // strona kontroli wysokości - też nic specjalnego
      break;
  }
  applyPWMOutput(); // zastosuj nowe reguły wyjścia PWM
}

// Co robimy gdy opuszczamy stronę
void onExitState(PageState st) {
  switch(st) {
    case ST_MENU:
      // z menu nic nie robimy
      break;
      
    case ST_PWM:
      // wychodzimy z PWM - zapisz ustawienia i natychmiast odetnij prąd
      eepromSavePWM((uint8_t)pwmValue);
      pwmValue = 0; // logiczne wyzerowanie
      analogWrite(pwmPin, 0); // fizyczne odcięcie prądu
      break;
      
    case ST_HEIGHT:
      // z height też nic
      break;
  }
}

// Przełącz stan na nowy (z obsługą enter/exit)
void switchState(PageState newState) {
  if(newState == currentState) return; // już jesteśmy w tym stanie
  onExitState(currentState); // posprzątaj po starym stanie
  currentState = newState; // przełącz
  onEnterState(currentState); // przygotuj nowy stan
}

// Mapuje ID strony Nextion na nasz stan
PageState pageIdToState(byte pageID) {
  if(pageID == PWM_PAGE_ID) return ST_PWM; // strona 2 = PWM
  if(pageID == HEIGHT_PAGE_ID) return ST_HEIGHT; // strona 1 = HEIGHT  
  return ST_MENU;  // wszystko inne = MENU (strona 0)
}

void setup() {
  Serial.begin(9600); // komunikacja z Nextion
  
  // konfiguruj piny sterownika silnika
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(rst_slpPin, OUTPUT);
  digitalWrite(rst_slpPin, HIGH); // sterownik od razu wybudzony
  
  // pin PWM
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 0); // na starcie brak prądu
  
  // wczytaj ustawienia z EEPROM
  eepromInitAndLoad();
  
  // startujemy od razu na stronie PWM
  Serial.print("page 2"); 
  nextionEnd();
  currentState = ST_PWM;
  onEnterState(currentState); // aktywuj stan PWM
  nextionCmd("sendme"); // poproś Nextion o potwierdzenie ID strony
  
  // inicjalizuj timery
  lastMotorActivity = millis();
  lastPagePoll = millis();
}

// Obsługa zdarzeń dotykowych z Nextion (ramka 0x65)
void handleTouchEvent(byte pageID, byte compID, byte event) {
  nextionCmd("sendme");  // po każdym dotyku sprawdź aktualną stronę
  
  if(currentState == ST_PWM && event == 1) { // tylko wciśnięcia na stronie PWM
    // przycisk + ma ID=2, przycisk - ma ID=3
    if(compID == 2 && pwmValue < PWM_LIMIT) {
      pwmValue += 1;  // zwiększ prąd
    } else if(compID == 3 && pwmValue > 0) {
      pwmValue -= 1;  // zmniejsz prąd
    }
    
    pwmValue = constrain(pwmValue, 0, PWM_LIMIT);  // zabezpieczenie
    updateCurrentText(pwmValue);  // pokaż nową wartość na wyświetlaczu
    // fizyczne wyjście PWM załatwi applyPWMOutput() w głównej pętli
  }
  
  if(currentState == ST_HEIGHT) {
    // przyciski GÓRA (ID=1) i DÓŁ (ID=2)
    if(compID == 1 || compID == 2) {
      unsigned long nowMs = millis();
      
      if(event == 1) {  // wciśnięcie przycisku
        // ustaw kierunek obrotu (GÓRA=HIGH, DÓŁ=LOW)
        digitalWrite(dirPin, (compID == 1) ? HIGH : LOW);
        
        // sprawdź czy to double-tap (drugi klik w krótkim czasie)
        if(nowMs - lastTapTime <= DOUBLE_TAP_MS) {
          fastMode = true;  // od razu szybko
        } else {
          fastMode = false;  // normalny start (wolno)
        }
        lastTapTime = nowMs;  // zapamiętaj czas tego kliku
        
        // ustaw odpowiednią prędkość i uruchom silnik
        motorDelayUs = fastMode ? SPEED_FAST_US : SPEED_SLOW_US;
        pressStartMs = nowMs; // zapamiętaj kiedy zaczęto trzymać
        motorRunning = true;
        
      } else if(event == 0) { // puszczenie przycisku
        motorRunning = false; // zatrzymaj silnik
      }
    }
  }
}

// Obsługa zmiany strony (ramka 0x66)
void handlePageEvent(byte pageID) {
  switchState(pageIdToState(pageID)); // przełącz stan na odpowiedni dla tej strony
}

// Czyta i parsuje ramki z Nextion
bool readNextionFrame() {
  static byte buf[16]; // bufor na ramkę
  static byte len = 0; // aktualna długość
  
  while(Serial.available()) {
    byte b = Serial.read();
    
    // akceptuj tylko ramki zaczynające się od 0x65 (touch) lub 0x66 (page)
    if(len == 0 && b != 0x65 && b != 0x66) {
      continue; // ignoruj śmieci
    }
    buf[len++] = b;
    
    // ramka kończy się sekwencją 0xFF 0xFF 0xFF (ostatnie 3 bajty)
    if(len >= 4 && buf[len-1] == 0xFF && buf[len-2] == 0xFF && buf[len-3] == 0xFF) {
      
      if(buf[0] == 0x66 && len >= 5) {
        // ramka zmiany strony: 0x66 [pageID] [0xFF 0xFF 0xFF]
        byte pageID = buf[1];
        handlePageEvent(pageID);
      } else if(buf[0] == 0x65 && len >= 7) {
        // ramka dotyku: 0x65 [pageID] [compID] [event] [0xFF 0xFF 0xFF]
        byte pageID = buf[1];
        byte compID = buf[2];
        byte event = buf[3]; // 1=press, 0=release
        handleTouchEvent(pageID, compID, event);
      }
      
      len = 0; // resetuj bufor
      return true;
    }
    
    if(len >= sizeof(buf)) len = 0; // overflow protection
  }
  return false;
}

void loop() {
  // ciągle odbieraj i parsuj ramki z Nextion
  readNextionFrame();
  
  // co PAGE_POLL_MS sprawdź aktualną stronę (dla pewności)
  unsigned long now = millis();
  if(now - lastPagePoll >= PAGE_POLL_MS) {
    nextionCmd("sendme"); // poproś o ID strony
    lastPagePoll = now;
  }
  
  // automatyczne przejście na szybki tryb po długim trzymaniu
  if(motorRunning && !fastMode && (now - pressStartMs >= HOLD_TO_FAST_MS)) {
    fastMode = true;
    motorDelayUs = SPEED_FAST_US; // przełącz na szybką prędkość
  }
  
  // generuj kroki silnika jeśli jest aktywny
  if(motorRunning) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelayUs); // połowa okresu
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorDelayUs); // druga połowa okresu
    lastMotorActivity = now; // aktualizuj timestamp aktywności
  }
  
  // zarządzanie snem sterownika DRV8825
  bool shouldSleep = (currentState != ST_HEIGHT) ||                                   // poza stroną HEIGHT zawsze śpi
                     (!motorRunning && (now - lastMotorActivity > SLEEP_DELAY_MS));  // lub po bezczynności
  
  if(shouldSleep && !driverSleeping) {
    digitalWrite(rst_slpPin, LOW); // uśpij sterownik
    driverSleeping = true;
  } else if(!shouldSleep && driverSleeping) {
    digitalWrite(rst_slpPin, HIGH); // wybudź sterownik
    driverSleeping = false;
  }
  
  // ciągle kontroluj wyjście PWM według aktualnej strony
  applyPWMOutput();
}