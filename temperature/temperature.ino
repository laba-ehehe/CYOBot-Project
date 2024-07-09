// Define A1 pin
#define SENSOR_PIN A1

// Constants in math formula NTC.
#define A 0.001009249522
#define B 0.0002378405444
#define C 2.019202697e-7

// Stores the Analog value read from the sensor.
float value;

// Voltage divider resistor (1k Ohm) with NTC.
#define R 1000.0

// Store the current resistance value of the NTC.
float rNTC;

// Store the Natural Logarithm of the NTC resistor.
float log_rNTC;

// Store the temperature value (°K) and (°C) of NTC.
float tempK;
float tempC;

void setup()
{
  // Start the Serial UART connection at 9600 to transfer data to the computer.
  Serial.begin(115200);
}

void loop()
{
  // Read Analog value.
  value = analogRead(SENSOR_PIN);

  /**
   * Calculate temperature:
   * 1. Calculate the current resistance value of the NTC based on the Analog value
   * 2. Calculate the logarithm of the NTC resistance value
   * 3. Apply the NTC formula to calculate the temperature in °K
   * 4. Convert °K to °C
   */

  rNTC = R / ((677.0 / value) - 1.0);
  log_rNTC = log(rNTC);
  tempK = (1.0 / (A + B * log_rNTC + C * log_rNTC * log_rNTC * log_rNTC));
  tempC = tempK - 273.15;

  // Transmit the measured value of the sensor to the computer.
  Serial.print("Temperature in °C: ");
  Serial.println(tempC, 2);

  // Wait 0,5s to measure again.
  delay(500);
}
