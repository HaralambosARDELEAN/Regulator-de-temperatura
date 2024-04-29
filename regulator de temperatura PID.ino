//DECLARAREA BIBLIOTECILOR
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//DEFINIREA PINILOR SI A VARIABILELOR
#define ONE_WIRE_BUS 23
#define pinTransistor 2

const int BUTTON_OK = 18;
const int BUTTON_CANCEL = 19;
const int BUTTON_MINUS = 20;
const int BUTTON_PLUS = 21;


unsigned long previousMillis = 0;
unsigned long prevProcessingMillis = 0;
unsigned long timp_incalzire_ms = 10000;  // timp de incalzire in milisecunde (40 sec)
unsigned long timp_mentinere_ms = 45000;  // timp de mentinere in milisecunde (25 sec)
unsigned long timp_racire_ms = 65000;     // timp de racire in milisecunde (15 sec)

bool incalzire = false;
bool mentinere = false;
bool racire = false;



//INITIALIZAREA LCD-ului SI SENZORULUI DE TEMPERATURA
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

float tempC;
float targetTemp;
 float tempIncreasePerMs;
 double error;
 double dErr;
  double timeChange;
unsigned long currentMillis = 0;

//DEFINIREA MENIURILOR SI A STARII CURENTE
enum Buttons{
  OK_BUTTON,
  CANCEL_BUTTON,
  PLUS_BUTTON,
  MINUS_BUTTON,
  EV_NONE,
  EV_MAX_NUM
};

enum Menus{
  MAIN_MENU,
  SET_TEMPERATURA,
  SET_TIMP_INCALZIRE,
  SET_TIMP_MENTINERE,
  SET_TIMP_RACIRE,
  SET_KP,
  SET_KI,
  SET_KD,
  MENU_MAX_NUM
};

Menus scroll_menu = MAIN_MENU;
Menus current_menu = MAIN_MENU;

void state_machine(enum Menus menu, enum Buttons button);
Buttons GetButtons(void);
void afisareDisplay(enum Menus menu, float temp_cur);

typedef void (state_machine_handler_t)(void);

unsigned long lastTime;
double Output, T_set=25;
double errSum, lastErr;
double kp=1000000.0, ki=-1.0, kd=1000000.0;


//FUNCTIA DE SALVARE A VARIABILELOR IN MEMORIA FLASH(EEPROM)
void saveToEEPROM() {
  EEPROM.put(0, T_set);
  EEPROM.put(sizeof(double), kp);
  EEPROM.put(sizeof(double) * 2, ki);
  EEPROM.put(sizeof(double) * 3, kd);
  EEPROM.put(sizeof(unsigned long) * 4, timp_incalzire_ms);
  EEPROM.put(sizeof(unsigned long) * 5, timp_racire_ms);
  EEPROM.put(sizeof(unsigned long) * 6, timp_mentinere_ms);
}


//FUNCTIA DE INCARCARE A VARIABILELOR IN MEMORIA FLASH(EEPROM)
void loadFromEEPROM() {
  EEPROM.get(0, T_set);
  EEPROM.get(sizeof(double), kp);
  EEPROM.get(sizeof(double) * 2, ki);
  EEPROM.get(sizeof(double) * 3, kd);
  EEPROM.get(sizeof(unsigned long) * 4, timp_incalzire_ms);
  EEPROM.get(sizeof(unsigned long) * 5, timp_racire_ms);
  EEPROM.get(sizeof(unsigned long) * 6, timp_mentinere_ms);
  // Add other variables as needed
}


void setup() {
  //saveToEEPROM();
  loadFromEEPROM();

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  pinMode(pinTransistor, OUTPUT);

// Setează pinii butoanelor ca intrări cu rezistențe de PULL-UP activate
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  sensors.begin();
}




void loop() {
  volatile Buttons event = GetButtons();
  currentMillis = millis();

  if (event != EV_NONE) {
    state_machine(current_menu, event);
  }

  afisareDisplay(scroll_menu, tempC);

  if ((prevProcessingMillis + 1000) < currentMillis) {
    sensors.requestTemperatures();

    if (sensors.getAddress(tempDeviceAddress, 0)) {
      tempC = sensors.getTempC(tempDeviceAddress);

      if (tempC < T_set || (currentMillis - previousMillis <= timp_incalzire_ms)) {
        // Procesul de încălzire
        targetTemp = tempC + tempIncreasePerMs * (currentMillis - previousMillis);

        if (targetTemp >= T_set) {
          targetTemp = T_set;
        }

        CalculPID(targetTemp);

        digitalWrite(pinTransistor, Output);
        incalzire = true;
        mentinere = false;
        racire = false;

        if (currentMillis - previousMillis >= timp_incalzire_ms) {
          previousMillis = currentMillis;
        }
      } else if (tempC >= T_set && currentMillis - previousMillis > timp_incalzire_ms &&
                 currentMillis - previousMillis <= timp_incalzire_ms + timp_mentinere_ms) {
        // Procesul de menținere
        digitalWrite(pinTransistor, LOW);
        incalzire = false;
        mentinere = true;
        racire = false;

        if (currentMillis - previousMillis >= timp_incalzire_ms + timp_mentinere_ms) {
          previousMillis = currentMillis;
        }
      } else if (currentMillis - previousMillis > timp_incalzire_ms + timp_mentinere_ms) {
        // Procesul de răcire
        digitalWrite(pinTransistor, LOW);
        incalzire = false;
        mentinere = false;
        racire = true;

        if (currentMillis - previousMillis >= timp_incalzire_ms + timp_mentinere_ms + timp_racire_ms) {
          previousMillis = currentMillis;
        }
      }
    }

    prevProcessingMillis = currentMillis;
  }

  delay(350);
}



void CalculPID(double Temp_curenta)
{
   //unsigned long now = millis();//moment curent de timp

   timeChange = (double)(currentMillis - lastTime); //diferenta de timp dintre 2 citiri ale temperaturii 
  
  //Calculul erorilor 
    error = T_set - Temp_curenta; //eroare
   errSum += (error * timeChange); //suma erorilor istorice 
    dErr = (error - lastErr) / timeChange; //rate de schimbare a erorii 
  
   Output = kp * error + ki * errSum + kd * dErr;//iesirea controlerului PID
  
   // limite superioare și inferioare pentru valoarea de ieșire
   if(Output>254) 
      Output=254;
   else if(Output<0)
      Output=0;
  
  //memorare variabile pentru urmatoarea iteratie 
   lastErr = error;
   lastTime = currentMillis;

  Serial.print("Temperatura setata: "); Serial.println(T_set);
  Serial.print(" - Temperatura curenta: "); Serial.println(Temp_curenta);
  Serial.print(" -Output : "); Serial.println(Output);
}

Buttons GetButtons(void)
{
  //delay(50);  // Așteaptă puțin pentru debouncing
  enum Buttons ret_val = EV_NONE;
  
  // Folosește funcția de debouncing pentru fiecare buton
  if (!debouncedRead(BUTTON_OK))
  {
    ret_val = OK_BUTTON;
  }
  else if (!debouncedRead(BUTTON_CANCEL))
  {
    ret_val = CANCEL_BUTTON;
  }
  else if (!debouncedRead(BUTTON_PLUS))
  {
    ret_val = PLUS_BUTTON;
  }
  else if (!debouncedRead(BUTTON_MINUS))
  {
    ret_val = MINUS_BUTTON;
  }

  //Serial.print(ret_val);
  return ret_val;
}

int debouncedRead(int buttonPin) {
  int citire = digitalRead(buttonPin);
  //delay(10);  // Așteaptă 10 milisecunde pentru debouncing
  return digitalRead(buttonPin) && citire;//combina citirea curenta cu cea initiala si daca ambele sunt true atunci returneaza true 
                                            //aici ne asiguram ca semnalul butonului este stabilit si nu este rezultatul unor schimbari temporare
}


void afisareDisplay(enum Menus menu, float temp_cur) {
  lcd.clear();
  switch (menu) {
    case SET_TEMPERATURA:
      lcd.print("Temp_set:");
      lcd.print(T_set);
      break;
    case SET_TIMP_INCALZIRE:
      lcd.print("Timp_inc: ");
      lcd.print(timp_incalzire_ms);
      break;
    case SET_TIMP_MENTINERE:
      lcd.print("Timp_ment");
      lcd.print(timp_mentinere_ms);
      break;
    case SET_TIMP_RACIRE:
      lcd.print("Timp_racire");
      lcd.print(timp_racire_ms);
      break;
    case SET_KP:
      lcd.print("Kp=");
      lcd.print(kp);
      break;
    case SET_KI:
      lcd.print("Ki= ");
      lcd.print(ki);
      break;
    case SET_KD:
      lcd.print("Kd= ");
      lcd.print(kd);
      break;
      case MAIN_MENU:
      default:
      lcd.print("Temp: ");
      lcd.print(T_set);
      lcd.setCursor(0, 1);
      lcd.print("Curr: ");
      lcd.print(temp_cur);
      break;
  }
  if(current_menu != MAIN_MENU)
  {
  	lcd.setCursor(0,1);
  	lcd.print("modifica");
  }
}

void enter_menu(void)
{
  current_menu = scroll_menu;
}

void go_home(void)
{
  scroll_menu = MAIN_MENU;
  current_menu = scroll_menu;
}

void go_next(void)
{
  scroll_menu = (Menus) ((int)scroll_menu + 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);//
}

void go_prev(void)
{
  scroll_menu = (Menus) ((int)scroll_menu - 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);//Operatorul "%" il folosim pentru a ne asigura ca valoarea meniului ramane in intervalul
                                                          // valid al numerelor de miniuri disponibile 
                                                          //De exemplu, dacă MENU_MAX_NUM este 5, și se ajunge la meniul 6, 
                                                          //% MENU_MAX_NUM va aduce rezultatul 1, astfel încât să se revină la începutul secvenței.
}

void inc_kp(void)
{
  kp++;
  saveToEEPROM();
}

void dec_kp(void)
{
  kp--;
  saveToEEPROM();
}

void inc_ki(void)
{
  ki++;
  saveToEEPROM();
}

void dec_ki(void)
{
  ki--;
  saveToEEPROM();
}

void inc_kd(void)
{
  kd++;
  saveToEEPROM();
}

void dec_kd(void)
{
  kd--;
  saveToEEPROM();
}

void inc_temp(void)
{
  T_set++;
  saveToEEPROM();
}

void dec_temp(void)
{
  T_set--;
  saveToEEPROM();
}

void inc_timp_incalzire(void)
{
  timp_incalzire_ms+=1000;
  saveToEEPROM();
}

void dec_timp_incalzire(void)
{
  timp_incalzire_ms-=1000;
  saveToEEPROM();
}


void inc_timp_mentinere(void)
{
  timp_mentinere_ms+=1000;
  saveToEEPROM();
}

void dec_timp_mentinere(void)
{
  timp_mentinere_ms-=1000;
  saveToEEPROM();
}


void inc_timp_racire(void)
{
  timp_racire_ms+=1000;
  saveToEEPROM();
}

void dec_timp_racire(void)
{
  timp_racire_ms-=1000;
  saveToEEPROM();
}


//folosim o matrice bidimensionala care stocheaza pointeri catre functii pentru a gestiona evenimentele 
//fiecare linie corespunde unui meniu specific, iar fiecare coloana corespunde unui eveniment specific 
state_machine_handler_t* sm[MENU_MAX_NUM][EV_MAX_NUM] = 
{ //events: OK , CANCEL , NEXT, PREV
  {enter_menu, go_home, go_next, go_prev},                          // MAIN_MENU
  {go_home, go_home, inc_temp, dec_temp},                           // MENU_TEMP
  {go_home, go_home, inc_kp, dec_kp},                               // MENU_Kp
  {go_home, go_home, inc_ki, dec_ki},                               // MENU_Ki
  {go_home, go_home, inc_kd, dec_kd},                               // MENU_Kd
  {go_home, go_home, inc_timp_incalzire, dec_timp_incalzire},       // MENU_timp_incalzire
  {go_home, go_home, inc_timp_mentinere, dec_timp_mentinere},       // MENU_timp_mentinere
  {go_home, go_home, inc_timp_racire, dec_timp_racire},             // MENU_timp_racire
  
};

void state_machine(enum Menus menu, enum Buttons button)
{
  sm[menu][button]();
}

