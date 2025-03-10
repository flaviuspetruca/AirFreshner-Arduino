 #include <LiquidCrystal.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NewPing.h>

//Distance sensor
#define TRIGGER_PIN  9 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters).

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Data wire is plugged into port A3 on the Arduino
#define ONE_WIRE_BUS A3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 6, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// PINS.
const int magneticSensor = A0;  // the number of the magnetic sensor pin
const int LDR = A1;             // the pin for light sensor
const int motionSensor = A2;    // the number of the motion sensor pin
const int buttonsPin = A4;      // the number of the pushbutton pin
const int greenLedPin = A5;     // the number of the green LED pin -- spraying
const int freshner = 3;         // the number of the freshner pin
const int redLedPin = 13;       // the number of the red LED pin -- system on

// variables :
int state_magnetic;       // 0 close - 1 open switch
int buttonState = 0;      // variable for reading the pushbutton status
int multiplicator = 100;  // helper in calculating the amount of sprays left;
int distance = 0;         // distance sensor variable
int light = 0;            // light sensor variable
int motion;               // motion sensor variable
int motionState = LOW;    // by default, no motion detected
int numberOfSprays = 0;   // number of sprays registered and that will be sprayed

//EEPROM cannot hold a value bigger then 255 in one byte so we will need two bytes in order to perform this;
const int addressFirst2 = 0;       // memory address for the number of sprays
const int addressLast2 = 1;        // memory address for the number of sprays

//EEPROM for delay
const int addressDelay = 2;        // memory address for delay variable

//variables required to register the number of sprays in EEPROM mempory
int valueFirst2;
int valueLast2;

int betweenSpraysInterval = 30000;
unsigned long lastSprayTime = 0;

//required to determine if there is motion in the bathroom
//if there is none the program will not check for a possible use
unsigned long lastMotionTime = 0;
unsigned long motionInterval = 300000; 

//debouncing
int lastButtonState;
unsigned long debounceDelay = 50;
unsigned long lastDebounceTimeButton = 0;

// in order to delay the spray have to keep track of the time the action to spray was initiated and if it has been sprayed or not(configurabale delay)
int delayedSpray = 0;
unsigned long initiatedSprayTime = 0; 

//menu variables
int menuState = 0;        // inactive menu
int selectedOption = -1;  // no selected option
int currentOption = 0;    // variable to cycle through options
volatile int greenLEDState = LOW;
String options[] = {      // available 
  "Spray Delay",
  "Replace can"
};
int optionValues[] = {EEPROM.read(addressDelay),1};
int optionLength = 1;


//nonblocking delay variables
unsigned long currentMillis;
unsigned long previousMillisSpray = 0;
unsigned long previousMillisSensors = 0;
unsigned long previousMillisMenu = 0;
int refreshInterval = 1000;
int minimumInterval = 16000;
int menuInterval = 5000;


//variables required for in use checking and calculating the spray state
const unsigned long nr2Time = 80000;
const unsigned long nr1Time = 30000;
const int distanceIntervalMin = 0;
const int distanceIntervalMax = 60; 
bool inInterval = false;

//in order to detect the use of the toilet we check if the distance for the last 10 sensor data recieved are in the interval of distances in which the toilet might be in use
int distancesArray[10];
int distArrLength = 10;
bool countingUseTime = false;
bool endSet = false;
unsigned long  inUseStart = -1;
unsigned long  inUseEnd = -1;
unsigned long previnUseStart = -1;
unsigned long previnUseEnd = -1;
int useState = 0; //can be // -- 0 for not in use // -- 1 for number 1// --2 for number 2// -- (-1) for unknown // -- 3 for cleaning 

void addDistanceToArray() {
  int i;
  inInterval = true;
  for (i = 0; i < distArrLength - 1 ; i++) {
    distancesArray[i] = distancesArray[i + 1];
    if (!(distancesArray[i] >= distanceIntervalMin) || !(distancesArray[i] <= distanceIntervalMax)) {
      inInterval = false;
    }
  }
  distancesArray[i] = distance;
  if (useState == 3 || !(distancesArray[i] >= distanceIntervalMin) || !(distancesArray[i] <= distanceIntervalMax)) {
    inInterval = false;
  }
}

void inUse() {
  if (inInterval) {
    if (!countingUseTime) {
      useState = -1;
      countingUseTime = true;
      inUseStart = millis();
      Serial.println(inUseStart);
    }
  } else {
    if (inUseStart != -1 && countingUseTime) {
      inUseEnd = millis();
      Serial.println(inUseEnd);
      countingUseTime = false;
    }
  }
}

// number of sprays initialization of EEPROM memory
void initializeEEPROM() {
  EEPROM.update(addressFirst2, 24);
  EEPROM.update(addressLast2, 0);
}

int initializeNumberOfAvailableSprays() {
  valueFirst2 = EEPROM.read(addressFirst2);
  valueLast2 = EEPROM.read(addressLast2);
  if (valueFirst2 > 0){
    return (valueFirst2*multiplicator + valueLast2);
  } else {
    return (valueLast2);
  }
}

int numberOfAvailableSprays = initializeNumberOfAvailableSprays();

void decreseNumberOfSpraysAndPrint() {
  if (valueLast2 == 0 && valueFirst2 == 0) {
    return;
  }
  if ( valueLast2 == 0) {
    valueFirst2--;
    valueLast2 = 99;
  } else {
    valueLast2--;
  }
  if (valueFirst2 == 0) {
    multiplicator = 0;
  } 
  numberOfAvailableSprays = valueFirst2*multiplicator + valueLast2;
  EEPROM.update(addressFirst2, valueFirst2);
  EEPROM.update(addressLast2, valueLast2);
  displayNumberSprays();
}

// -------------DISPLAY AND MENU---------------------

void initializeLCD() {
  // set up the LCD's number of columns and rows
  lcd.begin(16, 2);
}

void displayNumberSprays() {
  // set the cursor to column 0, line 0
  lcd.setCursor(0, 0);
  if ( numberOfAvailableSprays > 100 ){
    lcd.print(String(numberOfAvailableSprays) + " sprays");
  } else {
    lcd.print("LOW");
  }
}

void displayTemperature() {
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
  } else {
    Serial.println("not Connected temp");
  }
}

void displayMenu() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Options"));
  lcd.setCursor(0,1);
  lcd.print("< " + options[currentOption] + " >");
}

void displayMainScreen() {
  displayNumberSprays();
  displayTemperature();
}

void displayCurrentOption() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(options[selectedOption]);
  displayOptionValue();
}

void displayOptionValue() {
  lcd.setCursor(7,1);
  if (selectedOption == 0) {
    lcd.print(String(optionValues[selectedOption]) + " s");
  } else {
    lcd.print("Yes ->");
  }
}

void resetMenu(){
  if (currentMillis - previousMillisMenu >= menuInterval && menuState) {
    menuState = 0;
    currentOption = 0;
    selectedOption = -1;
    lcd.clear();
    displayMainScreen();
  }
}

void increaseOptionValue() {
  if (selectedOption == 0) {
    if (optionValues[selectedOption] < 60) {
      optionValues[selectedOption]++;
      EEPROM.update(addressDelay, optionValues[selectedOption]);
    }
  } else {
    //replaceCan
    initializeEEPROM();
    numberOfAvailableSprays = 2400;
    lcd.clear();
    displayMainScreen();
  }
}

void decreaseOptionValue() {
  if (selectedOption == 0) {
    if (optionValues[selectedOption] > 16) {
      optionValues[selectedOption]--;
      EEPROM.update(addressDelay, optionValues[selectedOption]);
    }
  } 
}

void scrollOptionRight() {
  if (currentOption < optionLength) {
    currentOption++;
    displayMenu();
    delay(500);
  }
}

void scrollOptionLeft() {
  if (currentOption > 0) {
    currentOption--;
    displayMenu();
    delay(500);
  }
}

// -------------SETUP---------------------
//initialize pins for leds
void setupLEDS() {
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  digitalWrite(redLedPin, HIGH);
}

//initialize sensor
void setupTemperatureSensor() {
  // Start up the library
  sensors.begin();
}

void setupMagnetic() {
  pinMode(magneticSensor, INPUT);
}

//initialize pin for the light sensor
void setUpLightSensor() {
  pinMode(LDR, INPUT);
}

//initialize pin for freshner
void setupFreshner() {
  pinMode(freshner, OUTPUT);
}

//initialize pin for motion sensor
void setupMotionSensor() {
  pinMode(motionSensor, INPUT);
}

//initialize pin for buttons
void setupButtons() {
  pinMode(buttonsPin, INPUT);
}

void setup() {
  attachInterrupt(1, changeGreenLedState, CHANGE);
  setupMagnetic();
  setupLEDS();
  setupTemperatureSensor();
  initializeLCD();
  setupFreshner();
  setupButtons();
  Serial.begin(9600);
}

// -------------GET VALUES SENSORS AND ACTUATORS---------------------
void getMotion(){
  if (!(millis() - lastMotionTime >= motionInterval || !lastMotionTime)) {
    return;
  }
  motion = digitalRead(motionSensor);   // read sensor value
  if (motion == HIGH) {              // check if the sensor is HIGH
    if (motionState == LOW) {
      motionState = HIGH;         // update variable state to HIGH
      lastMotionTime = millis();
      Serial.println("Last motion time: " + String(lastMotionTime));
    }
  } 
  else {
    if (motionState == HIGH) {
        motionState = LOW;        // update variable state to LOW
    }
  }
}

void getDistance() {
   distance = sonar.ping_cm();
}

void getLight() {
  light = analogRead(LDR);
}

void getFlushState() {
  //magnetic sensor state
  state_magnetic = digitalRead(magneticSensor);
}

bool buttonStateChanged() {
  if (lastButtonState > 900 && buttonState > 900) {
    return false;
  } else if ((lastButtonState > 350 && lastButtonState < 900) && (buttonState > 350 && buttonState < 900)) {
    return false;
  } else if ((lastButtonState > 50 && lastButtonState < 350) && (buttonState > 50 && buttonState < 540)) {
    return false;
  }
  return true;
}

// -------------SPRAY EVENTS---------------------
void calculateNumberOfSprays() {
  if (state_magnetic == HIGH) {
    if (inUseEnd != -1 && inUseStart != -1 && inUseEnd > inUseStart && previnUseEnd != inUseEnd && previnUseStart != inUseStart) {
      unsigned long timeOnToilet = inUseEnd - inUseStart;
      previnUseStart = inUseStart;
      previnUseEnd = inUseEnd;
      Serial.println("TimeOnToilet:" + String(timeOnToilet));
      if (timeOnToilet > nr2Time) {
        useState = 2;
        increaseNumberOfSprays(2);
      } else if (timeOnToilet > nr1Time && timeOnToilet < nr2Time) {
        useState = 1;
        increaseNumberOfSprays(1);
      } else {
        useState = 3;
      }
    } else {
      useState = 3;
    }
  } else {
    if (inInterval) {
      useState = -1;
    } else {
    useState = 0;
    }
  }
}

void checkPossibleSpray() {
  if (numberOfSprays && (millis() - lastSprayTime >= betweenSpraysInterval || lastSprayTime == 0)) {
    numberOfSprays--;
    spray();
    lastSprayTime = millis();
  }
}

void spray() {
  initiatedSprayTime = millis();
  if (optionValues[0] > 16) {
    delayedSpray = 1;
  } else if (currentMillis - previousMillisSpray >= minimumInterval) {
    analogWrite(freshner, 255);
    previousMillisSpray = millis();
    decreseNumberOfSpraysAndPrint();
  }
}


//if there has been a configurabale delay set this function gets called 
void delaySpray() {
  if (delayedSpray == 1 && optionValues[0] > 16 && (millis() - initiatedSprayTime >= optionValues[0]*1000 - minimumInterval)) {
    analogWrite(freshner, 255);
    previousMillisSpray = millis();
    decreseNumberOfSpraysAndPrint();
    delayedSpray = 0;
  }
}

void increaseNumberOfSprays(int amount) {
  if ((millis() - lastSprayTime >= betweenSpraysInterval && numberOfSprays == 0) || lastSprayTime == 0) {
    numberOfSprays += amount;
  }
}

void changeGreenLedState() {
    greenLEDState = !greenLEDState;
}

void loop() { 
  //checks if there is a spray required and performs the action
  checkPossibleSpray();
  //chaning greenLedState based on the interrupt
  digitalWrite(greenLedPin, greenLEDState);
  //if the selay has been configured to another time
  delaySpray();
  //5 seconds wait time for the menu to dissapear
  resetMenu();
  // read the state of the pushbutton value:
  buttonState = analogRead(buttonsPin);
  if (buttonStateChanged()) {
    lastDebounceTimeButton = millis();
  }
  
  currentMillis = millis();
  if (currentMillis - previousMillisSensors >= refreshInterval && !menuState) {
    displayMainScreen();
    getLight();
    getMotion();
    //checking for light and motion in order not to ask for data when not needed
    //done in windowless bathroom 
    if (light > 100 && motionState == HIGH) {
      getFlushState();
      getDistance();
      addDistanceToArray();
      calculateNumberOfSprays();
      inUse();
      Serial.println(String(useState) + '\n' + "DISTANCE: " + String(distance));
    }
    previousMillisSensors = currentMillis; 
  }

  //Menu And Button Spray
  currentMillis = millis();
  if (buttonState < 50) { //no button press
    if (currentMillis - previousMillisSpray >= optionValues[0]*1000) {
      analogWrite(freshner, 0);
    }
  }
  if ((currentMillis - lastDebounceTimeButton) > debounceDelay) {
    if (buttonState > 900) { // right most button press
      if (menuState) {
        if (selectedOption != -1) {
          increaseOptionValue();
          displayOptionValue();
          delay(500);
        } else {
          scrollOptionRight();
        }
      }
      previousMillisMenu = currentMillis;
    } else if (buttonState > 350) { // middle button press
      if (menuState) {
        if (selectedOption == -1) {
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
        delay(200);
      }
      previousMillisMenu = currentMillis;
    } else if(buttonState > 50) { // left most button press
        if (!menuState) {
          increaseNumberOfSprays(1);
        } else {
            if (selectedOption != -1) {
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
  lastButtonState = buttonState;
}
