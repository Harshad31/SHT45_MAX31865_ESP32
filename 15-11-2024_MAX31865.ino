#include <Adafruit_MAX31865.h>

#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

#define R0 100.0  

// Constants for the Callendar-Van Dusen equation
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

void setup() {
  Serial.begin(9600);
  thermo.begin(MAX31865_3WIRE);  
}

void loop() {
  uint16_t rtd = thermo.readRTD();
  float Rt = (rtd / 32768.0  * 430.0) ;  
  
  // Serial.print("RTD Resistance: ");
  // Serial.print(Rt, 3);
  // Serial.println(" ohms");

  float temperature = calculateTemperature(Rt);
  
  Serial.print(" Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

    unsigned long currentMillis = millis();

    unsigned long ms = currentMillis % 1000;  
    unsigned long seconds = (currentMillis / 1000) % 60;  
    unsigned long minutes = (currentMillis / 60000) % 60;  

    // Print time in MM:SS:MMM format
    Serial.print("Time: ");
    if (minutes < 10) Serial.print("0");
    Serial.print(minutes);
    Serial.print(":");

    if (seconds < 10) Serial.print("0");
    Serial.print(seconds);
    Serial.print(":");

    if (ms < 100) Serial.print("0");
    if (ms < 10) Serial.print("0");
    Serial.print(ms);


    Serial.print(", ");


  // delay(1000);
}

float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    // For temperatures above 0 °C
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    // For temperatures below 0 °C
    float tolerance = 0.001;  // Convergence tolerance
    int maxIterations = 100;  
    int iteration = 0;
    float diff;
    t = Rt; 

    do {
      float fValue = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - Rt;
      float fDerivative = R0 * (A + 2 * B * t + 3 * C * (t - 100) * t * t + C * t * t * t);
      float nextT = t - fValue / fDerivative;
      diff = abs(nextT - t);
      t = nextT;
      iteration++;
    } while (diff > tolerance && iteration < maxIterations);

    if (iteration == maxIterations) {
      Serial.println("Max iterations reached. Convergence not achieved.");
    }
  }
   // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }

  return t;
} 