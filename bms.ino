#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

Adafruit_ADS1115 ads;


// Cell voltage limits.
const float MAX_VOLTAGE = 3.6; 
const float MIN_VOLTAGE = 2.5;  

// Battery Current Limit
const float CURRENT_LIMIT = 2.0; // Change! This depends on the fact i am using 8 turns of the wire to represent a higher current.

// How many cells does the battery have? Max of 16.
const int NUMBER_OF_CELLS = 16;

const int RELAY_PIN = 13;

// Temperature Values
const int THERMISTOR_PIN = A3;          
const float DEFAULT_RESISTANCE = 10000.0;
const float DEFAULT_TEMP = 25.0;             
const float BETA = 3950.0; // May need to change.                  
const float SERIES_RESISTOR = 10000;

// Set Max Temperature
const float MAX_TEMPERATURE = 50.0;    

// Multiplexor truth table.
const int truthTable[16][4] = {
    {0, 0, 0, 0},
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {1, 1, 0, 0},
    {0, 0, 1, 0},
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {1, 1, 1, 0},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
    {0, 1, 0, 1},
    {1, 1, 0, 1},
    {0, 0, 1, 1},
    {1, 0, 1, 1},
    {0, 1, 1, 1},
    {1, 1, 1, 1}
};

// Digitial pins to control Mux 1
const int mux1Pins[4] = {2, 3, 4, 5};
// Digitial pins to control Mux 2
const int mux2Pins[4] = {6, 7, 8, 9};


enum BatteryState { connected, disconnected };
enum DisconnectReason { none, overvoltage, undervoltage, overcurrent, overtemperature  };

// Battery starts disconnected
BatteryState batteryConnectionState = disconnected;
DisconnectReason disconnectReason = none;

void setup() {
  Serial.begin(9600);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);

  pinMode(THERMISTOR_PIN, INPUT); 

  // Start as disconnected
  // It seems to pulse when first connecting can i stop this?
  digitalWrite(RELAY_PIN, LOW);
  
  Serial.println("Start disconnected before checking cell voltages. ");

  
  // Check that i2c connection to the ADC is correct. Otherwise don't continue. 
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADC.");
    while (1);
  }

  ads.setGain(GAIN_ONE);
}

void loop() {
  checkAllVoltages();
  checkCurrent();
  checkTemperature();
  delay(500);
}

void checkCurrent() {
    // put your main code here, to run repeatedly:
  int sensorValue = analogRead(A0);

  float voltage = sensorValue * (5.0 / 1023.0);

  // Serial.println(voltage);

  // Op Amp
  float actualVoltage = (voltage - 2.7705) / 0.5504;  

  // Needs to be negative with reverse current. Or is there another way to do it?
  float offset = 0.250;

  // - offset when + current. + offset when - current?
  // How to determine which direction before offset?

  float current = (actualVoltage - offset) / 100.0;
  current = current * 1000;
  current = current / 8.0; // Just for testing. Remove for actual use!

  if (current >= CURRENT_LIMIT) {
    Serial.println("Current limit reached.");

    if (batteryConnectionState == connected) {
      toggleBatteryConnectionState(overcurrent);
    }
  }

  Serial.print("Current (A): ");
  Serial.println(current, 5);

  delay(50); // What would be the best value for this?
}

void checkAllVoltages() {
  float currentTapVoltage;
  float previousTapVoltage;
  int currentTap;
  int previousTap;
  bool allCellsSafe = true; // Start assuming all cells are safe

  Serial.println("Checking all cell voltages.");

  for (int i = 1; i <= NUMBER_OF_CELLS; i++) {
    // // Using fixed cells for testing
    // currentTap = 2;
    // previousTap = 1;
    // currentTapVoltage = muxVoltage(currentTap, mux1Pins, 0);
    // previousTapVoltage = muxVoltage(previousTap, mux2Pins, 1);


    
    if (i == 1) {
      currentTap = i;
      currentTapVoltage = muxVoltage(currentTap, mux1Pins, 0);
      previousTapVoltage = 0;
    } else {
      currentTap = i;
      previousTap = i - 1;
      currentTapVoltage = muxVoltage(currentTap, mux1Pins, 0);
      previousTapVoltage = muxVoltage(previousTap, mux2Pins, 1);
    }
    
    float cellVoltage = currentTapVoltage - previousTapVoltage;

    // Check if this cell is outside safe limits
    bool cellSafe = true;
    if (cellVoltage >= MAX_VOLTAGE) {
      cellSafe = false;
      allCellsSafe = false;
      Serial.print("Cell ");
      Serial.print(i);
      Serial.println(": Overvoltage");
      
      if (batteryConnectionState == connected) {
        toggleBatteryConnectionState(overvoltage);
      }

    } else if (cellVoltage <= MIN_VOLTAGE) {
      cellSafe = false;
      allCellsSafe = false;
      Serial.print("Cell ");
      Serial.print(i);
      Serial.println(": Undervoltage");
      
      if (batteryConnectionState == connected) {
        toggleBatteryConnectionState(undervoltage);
      }
    }

    Serial.print("Cell ");
    Serial.print(i);
    Serial.print(": Voltage: ");
    Serial.print(cellVoltage);
    Serial.print("  Status: ");
    Serial.println(cellSafe ? "Safe" : "Unsafe");

    delay(50); // What would be the best value for this?
  }

  if (batteryConnectionState == disconnected && allCellsSafe) {
    Serial.println("Status: disconnected");
    Serial.println("All cells are safe, so can connect battery.");
    
  } else if (batteryConnectionState == disconnected && !allCellsSafe) {
    Serial.println("Status: disconnected");
    Serial.println("Some unsafe cells, so stay diconnected");

  } else if (batteryConnectionState == connected && allCellsSafe) {
    Serial.println("Status: connected");
    Serial.println("Stay connected.");

  }
}

void checkTemperature() {
  float reading = analogRead(THERMISTOR_PIN);

  // Calculate resistance
  float resistance = SERIES_RESISTOR * (1023.0 / reading - 1.0);

  // Calculate temperature using steinhart equation
  float temp;
  temp = resistance / DEFAULT_RESISTANCE;
  temp = log(temp);                     
  temp /= BETA;                         
  temp += 1.0 / (DEFAULT_TEMP + 273.15); 
  temp = 1.0 / temp;                     
  temp -= 273.15;                       


  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" *C");

  if (temp >= MAX_TEMPERATURE) {
    Serial.println("Temperature limit reached.");
    if (batteryConnectionState == connected) {
      toggleBatteryConnectionState(overtemperature);
    }
  }
}


void toggleBatteryConnectionState(DisconnectReason reason) {
  if (batteryConnectionState == connected) {
    digitalWrite(RELAY_PIN, LOW);
    batteryConnectionState = disconnected;
    disconnectReason = reason;
  } else if (batteryConnectionState == disconnected && reason == none) {
    // Reconnecting the battery after safe conditions return
    // digitalWrite(BATTERY_DISCONNECT_PIN, HIGH);
    // batteryConnectionState = connected;
  }
}

float muxVoltage(int cell, const int muxPins[4], int port) {
  // Account for using cell starting at 1. 
  cell = cell - 1;

  digitalWrite(muxPins[0], truthTable[cell][0]);
  digitalWrite(muxPins[1], truthTable[cell][1]);
  digitalWrite(muxPins[2], truthTable[cell][2]);
  digitalWrite(muxPins[3], truthTable[cell][3]);

  // int analogReading = analogRead(port);

  float adc = ads.readADC_SingleEnded(port);

  float voltage = adc * (4.096 / 32768.0);

  // Account for voltage divider.
  voltage = voltage * 13;

  return voltage;
}