 #include <Adafruit_MAX31865.h>

#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

#define R0 100.0  
#define Rref 430.0 

#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

// Timing variables
unsigned long lastTemperatureUpdate = 0;
const unsigned long temperatureInterval = 500; // 500 milli second between temperature reads

// Averaging variables
int numReadings = 10;

void setup() {
  Serial.begin(9600); 
  thermo.begin(MAX31865_3WIRE);  
}

void loop() {
  unsigned long currentMillis = millis();

  // Read and average 10 readings for temperature
  if (currentMillis - lastTemperatureUpdate >= temperatureInterval) {
    float sumRt = 0.0;
    for (int i = 0; i < numReadings; i++) {
      uint16_t adcCode = thermo.readRTD(); 
      float Rt = (adcCode * Rref) / 32768.0; 
      sumRt += Rt; 
    }

    // Calculate average RTD resistance
    float averagedRt = sumRt / numReadings;

    // Calculate temperature from averaged RTD resistance
    float temperature = calculateTemperature(averagedRt);

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    lastTemperatureUpdate = currentMillis;
  }
}

// Calculate temperature from RTD resistance using Callendar-Van Dusen equation
float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    // For temperatures above 0°C
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    // For temperatures below 0°C
    float tolerance = 0.001;  
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
    
      Serial.println("Warning: Temperature calculation failed to converge.");
    }
  }

  return t;
}
