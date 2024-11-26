#include <Wire.h>
#include <ArtronShop_SHT45.h>

ArtronShop_SHT45 sht45(&Wire, 0x44); // SHT45-AD1B => 0x44

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!sht45.begin()) {
    Serial.println("SHT45 not found !");
    // delay(1000);
  }
}

void loop() {
  if (sht45.measure()) {

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

    // Add a comma after milliseconds
    Serial.print(", ");

    Serial.print("Temperature: ");
    Serial.print(sht45.temperature(), 2);
    Serial.print(" *C,\tHumidity: ");
    Serial.print(sht45.humidity(), 2);
    Serial.print(" %RH");
    Serial.println();
  } else {
    Serial.println("SHT45 read error");
  }
  // No delay needed, loop runs continuously
}

