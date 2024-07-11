// Chọn chân Analog đọc cảm biến.
// Select the Analog pin to read the sensor.
#define SENSOR_PIN A1

// Các hằng số trong công thức NTC.
// Constants in math formula NTC.
#define A 0.001009249522
#define B 0.0002378405444
#define C 2.019202697e-7

// Lưu giá trị Analog đọc từ cảm biến.
// Stores the Analog value read from the sensor.
float value;

// Điện trở phân áp (1k Ohm) với NTC.
// Voltage divider resistor (1k Ohm) with NTC.
#define R 1000.0

// Lưu giá trị điện trở hiện tại của NTC.
// Store the current resistance value of the NTC.
float rNTC;

// Lưu giá trị Logarit tự nhiên của điện trở NTC.
// Store the Natural Logarithm of the NTC resistor.
float log_rNTC;

// Lưu giá trị nhiệt độ (°K) và (°C) của NTC.
// Store the temperature value (°K) and (°C) of NTC.
float tempK;
float tempC;

void setup()
{
  // Khởi động kết nối Serial UART ở tốc độ 9600 để truyền dữ liệu lên máy tính.
  // Start the Serial UART connection at 9600 to transfer data to the computer.
  Serial.begin(115200);
}

void loop()
{
  // Đọc giá trị Analog.
  // Read Analog value.
  value = analogRead(SENSOR_PIN);

  /**
   * Các bước tính nhiệt độ của NTC
   * 1. Tính giá trị Điện trở hiện tại của NTC dựa trên giá trị Analog
   * 2. Tính Logarit của giá trị Điện trở NTC
   * 3. Áp dụng công thức của NTC, tính ra nhiệt độ °K
   * 4. Chuyển đổi °K sang °C
   */
  rNTC = R / ((1023.0 / value) - 1.0);
  log_rNTC = log(rNTC);
  tempK = (1.0 / (A + B * log_rNTC + C * log_rNTC * log_rNTC * log_rNTC));
  tempC = tempK - 273.15 - 100;

  // Truyền giá trị đo được của cảm biến lên máy tính.
  // Transmit the measured value of the sensor to the computer.
  
  Serial.print("tempK: ");
  Serial.println(tempK, 2);
  
  Serial.print("tempC: ");
  Serial.println(tempC, 2);

  // Chờ 0,5s mới đo lại.
  // Wait 0,5s to measure again.
  delay(500);
}
