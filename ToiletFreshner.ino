 #include <LiquidCrystal.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NewPing.h>

//Distance sensor
#define TRIGGER_PIN  9 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A0 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters).

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Data wire is plugged into port A3 on the Arduino
#define ONE_WIRE_BUS A3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// constants won't change.
const int LDR = A1;             // the pin for light sensor
const int motionSensor = A2;    // the number of the motion sensor pin
const int buttonsPin = A4;      // the number of the pushbutton pin
const int greenLedPin = A5;     // the number of the green LED pin -- spraying
const int freshner = 6;         // the number of the freshner pin
const int redLedPin = 13;       // the number of the red LED pin -- system on
const int magneticSensor = 8;   // the number of the magnetic sensor pin
int state_magnetic;             // 0 close - 1 open wwitch

//EEPROM cannot hold a value bigger then 255 in one byte so we will need two bytes in order to perform this;
const int addressFirst2 = 0;       // memory address for the number of sprays
const int addressLast2 = 1;        // memory address for the number of sprays

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int valueFirst2;
int valueLast2;
int multiplicator = 100;  //helper in calculating the amount of sprays left;
int distance = 0;         //distance sensor variable
int light = 0;            //light sensor variable
int motion;               //motion sensor variable
int motionState = LOW;    // by default, no motion detected

//menu variables
int menuState = 0;        //inactive menu
int selectedOption = -1;  // no selected option
int currentOption = 0;    //variable to cycle through options
String options[] = { 
  "Spray Delay",
  "Replace can"
};
int optionValues[] = {15,1};
int optionLength = 1;


//nonblocking delay variables
unsigned long currentMillis;
unsigned long previousMillisButtonSpray = 0;
unsigned long previousMillisSensors = 0;
unsigned long previousMillisMenu = 0;
int refreshInterval = 2000;
int buttonInterval = 16000;
int menuInterval = 5000;

void initializeEEPROM(){
  EEPROM.update(addressFirst2, 24);
  EEPROM.update(addressLast2, 0);
}

int initializeNumberOfSprays(){
  valueFirst2 = EEPROM.read(addressFirst2);
  valueLast2 = EEPROM.read(addressLast2);
  if (valueFirst2 > 0){
    return (valueFirst2*multiplicator + valueLast2);
  } else {
    return (valueLast2);
  }
}

int numberOfAvailableSprays = initializeNumberOfSprays();

void decreseNumberOfSpraysAndPrint(){
  if( valueLast2 == 0 && valueFirst2 == 0){
    return;
  }
  if( valueLast2 == 0){
    valueFirst2--;
    valueLast2 = 99;
  } else {
    valueLast2--;
  }
  if (valueFirst2 == 0){
    multiplicator = 0;
  } 
  numberOfAvailableSprays = valueFirst2*multiplicator + valueLast2;
  EEPROM.update(addressFirst2, valueFirst2);
  EEPROM.update(addressLast2, valueLast2);
  displayNumberSprays();
}

void initializeLCD(){
  // set up the LCD's number of columns and rows and print the available sprays:
  lcd.begin(16, 2);
}

void displayNumberSprays(){
  // set the cursor to column 0, line 0
  lcd.setCursor(0, 0);
  if ( numberOfAvailableSprays > 100 ){
    lcd.print(String(numberOfAvailableSprays) + " sprays");
  } else {
    lcd.print("LOW");
  }
}


//initialize pins for leds
void setupLEDS(){
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  digitalWrite(redLedPin, HIGH);
}

//initialize sensor
void setupTemperatureSensor(){
  // Start up the library
  sensors.begin();
}

void setupMagnetic(){
  pinMode(magneticSensor, INPUT);
}

//initialize pin for the light sensor
void setUpLightSensor(){
  pinMode(LDR, INPUT);
}

//initialize pin for freshner
void setupFreshner(){
  pinMode(freshner, OUTPUT);
}

//initialize pin for motion sensor
void setupMotionSensor(){
  pinMode(motionSensor, INPUT);
}

//initialize pin for buttons
void setupButtons(){
  pinMode(buttonsPin, INPUT);
}

void setup() {
  //this functions is run only once at the first setup of the whole system
  setupMagnetic();
  setupLEDS();
  setupTemperatureSensor();
  //initializeEEPROM();
  initializeLCD();
  setupFreshner();
  setupButtons();
}


void displayTemperature(){
  // set the cursor to column 10, line 1
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  lcd.setCursor(10, 1);

  //make only one decimal point string from temperature
  String tempCString = String(tempC);
  int length = tempCString.length();
  tempCString.remove(length-1);
  if (tempC != DEVICE_DISCONNECTED_C) {
    lcd.print(tempCString + " C");
  }
}

void getMotion(){
  motion = digitalRead(motionSensor);   // read sensor value
  if (motion == HIGH) {              // check if the sensor is HIGH
    delay(200);
    if (motionState == LOW) {
      motionState = HIGH;         // update variable state to HIGH
    }
  } 
  else {
    delay(200);
    if (motionState == HIGH){
        motionState = LOW;        // update variable state to LOW
    }
  }
}

void getDistance(){
   distance = sonar.ping_cm();
}

void getLight(){
  light = analogRead(LDR);
}

void resetMenu(){
  if(currentMillis - previousMillisMenu >= menuInterval && menuState){
    menuState = 0;
    currentOption = 0;
    selectedOption = -1;
    lcd.clear();
    displayMainScreen();
  }
}
void displayMenu(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Options"));
  lcd.setCursor(0,1);
  lcd.print("< " + options[currentOption] + " >");
}

void displayMainScreen(){
  displayNumberSprays();
  displayTemperature();
}

void displayCurrentOption(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(options[selectedOption]);
  displayOptionValue();
}

void displayOptionValue(){
  lcd.setCursor(7,1);
  if(selectedOption == 0){
    lcd.print(String(optionValues[selectedOption]) + " s");
  } else {
    lcd.print("Yes ->");
  }
}

void increaseOptionValue(){
  if(selectedOption == 0){
    if(optionValues[selectedOption] < 60){
      optionValues[selectedOption]++;
    }
  } else {
    //replaceCan
    initializeEEPROM();
    numberOfAvailableSprays = 2400;
    lcd.clear();
    displayMainScreen();
  }
}

void decreaseOptionValue(){
  if(selectedOption == 0){
    if(optionValues[selectedOption] > 15){
      optionValues[selectedOption]--;
    }
  } 
}

void scrollOptionRight(){
  if(currentOption < optionLength){
    currentOption++;
    displayMenu();
    delay(500);
  }
}

void scrollOptionLeft(){
  if(currentOption > 0){
    currentOption--;
    displayMenu();
    delay(500);
  }
}

void checkPossibleSpray(){
  if(distance < 10 && state_magnetic == HIGH){
    if(millis() - previousMillisButtonSpray >= buttonInterval){
      spray();
    }
  }
}

void spray(){
  analogWrite(freshner, 255);
  digitalWrite(greenLedPin, HIGH);
  previousMillisButtonSpray = millis();
  decreseNumberOfSpraysAndPrint();
}

void loop() { 
  // read the state of the pushbutton value:
  checkPossibleSpray();
  buttonState = analogRead(buttonsPin);
  currentMillis = millis();
  if(currentMillis - previousMillisSensors >= refreshInterval && !menuState){
    displayMainScreen();
    getLight();
    getDistance();
    getMotion();
    previousMillisSensors = currentMillis; 
  }
  
  //5 seconds wait time for the menu to dissapear
  resetMenu();
  
  //magnetic sensor state
  state_magnetic = digitalRead(magneticSensor);
  
  currentMillis = millis();
  if(buttonState < 50){
    if(currentMillis - previousMillisButtonSpray >= buttonInterval){
      analogWrite(freshner, 0);
      digitalWrite(greenLedPin, LOW);
    }
  } else if(buttonState > 900){
    if(menuState){
      if(selectedOption != -1){
        increaseOptionValue();
        displayOptionValue();
        delay(500);
      } else {
        scrollOptionRight();
      }
    }
    previousMillisMenu = currentMillis;
  } else if(buttonState > 300){
    if(menuState){
      if(selectedOption == -1){
        selectedOption = currentOption;
        displayCurrentOption();
        delay(200);
      } else {
        //go back
        selectedOption = -1;
        displayMenu();
        delay(200);
      }
    } else {
      menuState = 1;
      displayMenu();
      delay(500);
    }
    previousMillisMenu = currentMillis;
  } else if(buttonState > 50){
      if(!menuState){
        //spray
        spray();
      } else {
          if(selectedOption != -1){
            decreaseOptionValue();
            displayOptionValue();
            delay(500);
          } else {
            scrollOptionLeft();
          }
          previousMillisMenu = currentMillis;
      }
  }
}
