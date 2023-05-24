#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Wire.h>

#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS = 500;
LiquidCrystal_I2C lcd(0x3F, 16, 2);
bool isUpperPrinted = 0;
bool isLowerPrinted = false;
float prevTemp = 0.0;
float prevHum = 0.0;

void setup()
{
  Serial.begin(9600);
  // Initialize device.
  dht.begin();
  lcd.init();
  lcd.backlight();
  Wire.begin();
  lcd.clear();
}

void loop()
{
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature) && !isUpperPrinted)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("                "));
    lcd.setCursor(0, 0);
    lcd.print(F("Error temp"));
    lcd.display();
    isUpperPrinted = 1;
  }
  else if (prevTemp != event.temperature)
  {
    prevTemp = event.temperature;
    lcd.setCursor(0, 0);
    lcd.print(F("                "));
    lcd.setCursor(0, 0);
    lcd.print(F("Temp: "));
    lcd.print(prevTemp);
    lcd.print(F("\xDF"));
    lcd.print(F("C"));
    lcd.display();
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity) && !isLowerPrinted)
  {
    lcd.setCursor(0, 1);
    lcd.print(F("                "));
    lcd.setCursor(0, 1);
    lcd.print(F("Error humidity"));
    lcd.display();
    isLowerPrinted = 1;
  }
  else if (prevHum != event.relative_humidity)
  {
    prevHum = event.relative_humidity;
    lcd.setCursor(0, 1);
    lcd.print(F("                "));
    lcd.setCursor(0, 1);
    lcd.print(F("Humidity: "));
    lcd.print(prevHum);
    lcd.print(F("%"));
    lcd.display();
  }
}
