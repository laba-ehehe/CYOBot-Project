#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Pin definitions
#define SDA_PIN 21
#define SCL_PIN 22

int i = 0;

MAX30105 particleSensor;

// Heart rate set up
const byte RATE_SIZE = 5;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// SpO2 set up
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;
byte pulseLED = 11;
byte readLED = 13;

// Temperature set up
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// LCD set up
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.begin();
  
  Serial.println("Initializing...");
  
  // LCD initialization
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  Serial.println("LCD initialized");
  delay(100);
  
  // Initialize MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    lcd.setCursor(0, 1);
    lcd.print("MAX30105 not found");
    while (1);
  }
  Serial.println("MAX30105 initialized");
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  delay(100);
  
  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    lcd.setCursor(0, 2);
    lcd.print("MLX sensor error");
    while (1);
  }
  Serial.println("MLX90614 initialized");
  delay(100);
  
  Serial.println("All sensors initialized successfully");
  lcd.clear();
  lcd.print("Ready!");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Heart rate and SpO2
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    Serial.println("No finger detected or weak signal");
  } else {
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60.0 / (delta / 1000.0);

      if (beatsPerMinute < 255.0 && beatsPerMinute > 20.0) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

  bufferLength = 100;

  for (byte j = 0 ; j < bufferLength ; j++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[j] = particleSensor.getRed();
    irBuffer[j] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }

  // Temperature
  float objectTemp = mlx.readObjectTempC();
  float ambientTemp = mlx.readAmbientTempC();
  float tempC_offset = objectTemp + 5;
  
  if (isnan(objectTemp) || isnan(ambientTemp)) {
    Serial.println("Failed to read from MLX sensor");
  } else {
    Serial.print("Object Temperature: ");
    Serial.print(objectTemp);
    Serial.print("°C, Ambient Temperature: ");
    Serial.print(ambientTemp);
    Serial.println("°C");
  }
  
  // Print values to LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Average BPM: ");
  lcd.print(beatAvg);

  lcd.setCursor(0, 1);
  lcd.print("SPO2: ");
  if (validSPO2) {
    lcd.print(spo2);
  } else {
    lcd.print("--");
  }

  lcd.setCursor(0, 2);
  lcd.print("Temperature: ");
  if (!isnan(tempC_offset)) {
    lcd.print(tempC_offset, 1);
    lcd.print("C");
  } else {
    lcd.print("Error");
  }

  lcd.setCursor(0, 3);
  String status;
  // if ((beatAvg > 50 && beatAvg < 95) && (tempC_offset > 35 && tempC_offset < 38) && (spo2 > 95 && spo2 < 101)) {
  if ((tempC_offset > 35 && tempC_offset < 38) && (spo2 > 95 && spo2 < 101)) {
    status = "NORMAL :DDD";
  } else {
    status = "ALERT!!!!!";
  }
  lcd.print(status);

  // Print values to Serial
  Serial.print("Average BPM: ");
  Serial.println(beatAvg);

  Serial.print("SPO2: ");
  if (validSPO2) {
    Serial.println(spo2);
  } else {
    Serial.println("Invalid");
  }

  Serial.print("Temperature: ");
  if (!isnan(objectTemp)) {
    Serial.print(objectTemp);
    Serial.println("°C");
  } else {
    Serial.println("Error");
  }

  Serial.println(i);
  i++;

  // delay(1000);
}
