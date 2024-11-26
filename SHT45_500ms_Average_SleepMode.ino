#include <Wire.h>
#include <ArtronShop_SHT45.h>

ArtronShop_SHT45 sht45(&Wire, 0x44);

const int totalReadings = 12;
double temperatureReadings[totalReadings];
double humidityReadings[totalReadings];

unsigned long totalReadingTime = 0;  // total time spent on readings
unsigned long totalSleepTime = 0;    //  store total sleep time

int readingCount = 0;  // count how many times readings are taken
int sleepCount = 0;    // count how many times sleeping 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!sht45.begin()) {
    Serial.println("SHT45 not found!");
  
  }
  Serial.println("SHT45 Sensor initialized");
}

void loop() {
  unsigned long startTime = millis();
  
  for (int i = 0; i < totalReadings; ++i) {
    if (sht45.measure()) {
      temperatureReadings[i] = sht45.temperature();
      humidityReadings[i] = sht45.humidity();
    } else {
      Serial.println("SHT45 read error");
      temperatureReadings[i] = 0.0;
      humidityReadings[i] = 0.0;
    }
  }

  unsigned long endTime = millis();
  unsigned long timeTakenForReadings = endTime - startTime;
  totalReadingTime += timeTakenForReadings;
  readingCount++;

  unsigned long remainingSleepTime = 500 - timeTakenForReadings;
  if (remainingSleepTime > 0) {
    delay(remainingSleepTime);
    totalSleepTime += remainingSleepTime;
    sleepCount++;
  }

  // Calculate the average of the last 10 readings for temperature & Humidity
  double tempToAverage[10];
  double humidityToAverage[10];

  for (int i = 2; i < totalReadings; ++i) {
    tempToAverage[i - 2] = temperatureReadings[i];
    humidityToAverage[i - 2] = humidityReadings[i];
  }

  double tempSum = 0.0;
  double humiditySum = 0.0;

  for (int i = 0; i < 10; ++i) {
    tempSum += tempToAverage[i];
    humiditySum += humidityToAverage[i];
  }

  double averageTemp = tempSum / 10;
  double averageHumidity = humiditySum / 10;

  // Calculate average reading time and sleep time
  double avgReadingTime = readingCount > 0 ? (double)totalReadingTime / readingCount : 0;
  double avgSleepTime = sleepCount > 0 ? (double)totalSleepTime / sleepCount : 0;

  // Get the current time using millis()
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

  // Print the sensor data
  Serial.print(", Temperature: ");
  Serial.print(averageTemp, 2);
  Serial.print(" °C, ");
  
  Serial.print("Humidity: ");
  Serial.print(averageHumidity, 2);
  Serial.println(" % ");

  // Print average reading time and sleep time
  // Serial.print("Avg Reading Time: ");
  // Serial.print(avgReadingTime, 2);
  // Serial.print(" ms, ");
  
  // Serial.print("Avg Sleep Time: ");
  // Serial.print(avgSleepTime, 2);
  // Serial.println(" ms");
}
