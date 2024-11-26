#include <Wire.h>
#include <ArtronShop_SHT45.h>
#include <Adafruit_MAX31865.h>
#include <WiFi.h>

#define MAX31865_MISO 13
#define MAX31865_MOSI 11
#define MAX31865_CLK 12
#define MAX31865_CS 10

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

ArtronShop_SHT45 sht45(&Wire, 0x44);

const char* ssid     = "VIRAT";
const char* password = "12345678";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int   daylightOffset_sec = 0;

#define R0 100.0
#define Rref 430.0
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

// Define number of readings to take
const int totalReadingsMAX31865 = 2;
const int totalReadingsSHT45 = 12;

double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0}; 
double temperatureReadingsSHT45[totalReadingsSHT45] = {0}; 
double humidityReadings[totalReadingsSHT45] = {0}; 

unsigned long totalReadingTime = 0;
unsigned long totalSleepTime = 0;
int readingCount = 0;
int sleepCount = 0;

unsigned long lastReadingTime = 0;  // Store the last time the reading was taken
const long readingInterval = 500;   // Interval between readings (500ms)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected.");

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // Check if time is synchronized
  struct tm timeinfo;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("Time synchronized");
      break;
    } else {
      Serial.println("Failed to get time, retrying...");
      delay(1000); // Retry after 1 second
    }
  }

  thermo.begin(MAX31865_3WIRE);
  Serial.println("MAX31865 Temperature Sensor Initialized");

  while (!sht45.begin()) {
    Serial.println("SHT45 not found!");
  }
  Serial.println("SHT45 Sensor Initialized");
}

void loop() {
  unsigned long startTime = millis();  // Start time for the cycle
  
  // Get current time
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    // Calculate the milliseconds
    unsigned long ms = millis() % 1000; 

    // Format time as HH:MM:SS:MS
    char formattedTime[20];
    snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

    // Print the formatted time and sensor data in one line
    Serial.print(formattedTime);
    Serial.print(" , ");

    // MAX31865 Sensor Reading
    for (int i = 0; i < totalReadingsMAX31865; i++) {
      uint16_t adcCode = thermo.readRTD();
      float Rt = (adcCode * Rref) / 32768.0;
      temperatureReadingsMAX31865[i] = Rt;
    }

    // SHT45 Sensor Reading
    for (int i = 0; i < totalReadingsSHT45; ++i) {
      if (sht45.measure()) {
        temperatureReadingsSHT45[i] = sht45.temperature();
        humidityReadings[i] = sht45.humidity();
      } else {
        Serial.println("SHT45 read error");
        temperatureReadingsSHT45[i] = 0.0;
        humidityReadings[i] = 0.0;
      }
    }

    // Calculate average temperature from MAX31865
    double tempSumMax31865 = 0;
    for (int i = 0; i < totalReadingsMAX31865; i++) {
      tempSumMax31865 += temperatureReadingsMAX31865[i];
    }
    float averagedRtMax31865 = tempSumMax31865 / totalReadingsMAX31865;
    float temperatureMax31865 = calculateTemperature(averagedRtMax31865);

    // Calculate average temperature and humidity from SHT45
    double tempSumSHT45 = 0.0;
    double humiditySum = 0.0;
    for (int i = 0; i < totalReadingsSHT45; ++i) {
      tempSumSHT45 += temperatureReadingsSHT45[i];
      humiditySum += humidityReadings[i];
    }
    double averageTempSHT45 = tempSumSHT45 / totalReadingsSHT45;
    double averageHumidity = humiditySum / totalReadingsSHT45;

    // Print the sensor data in one line
    Serial.print("MAX31865 Temp, ");
    Serial.print(temperatureMax31865, 2);
    Serial.print("°C, ");

    Serial.print("SHT45 Temp, ");
    Serial.print(averageTempSHT45, 2);
    Serial.print("°C, ");

    Serial.print("Humidity, ");
    Serial.print(averageHumidity, 2);
    Serial.print("%");

    // End of sensor reading
    unsigned long endTime = millis();
    unsigned long timeTakenForReadings = endTime - startTime;
    totalReadingTime += timeTakenForReadings;
    readingCount++;

    // Calculate remaining sleep time to keep 500ms interval
    unsigned long remainingSleepTime = readingInterval - timeTakenForReadings;
    if (remainingSleepTime > 0) {
      delay(remainingSleepTime);
      totalSleepTime += remainingSleepTime;
      sleepCount++;
    }

    // Print a newline after each loop cycle
    Serial.println();
  } else {
    Serial.println("Failed to obtain time");
  }
}

// Function to calculate temperature from MAX31865 RTD value
float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
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
