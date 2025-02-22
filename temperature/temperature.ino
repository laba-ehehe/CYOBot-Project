#include <Adafruit_MLX90614.h>
#include <math.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
	Serial.begin(115200);
	while (!Serial);

	if (!mlx.begin()) {
		Serial.println("Error connecting to MLX sensor. Check wiring.");
		while (1);
	};
}

void loop() {
	// Serial.print("Ambient = "); 
  // Serial.print(mlx.readAmbientTempC());
	// Serial.print("*C\tObject = "); 
  Serial.print(mlx.readObjectTempC()); 
  int tempC = (int)mlx.readObjectTempC();
  Serial.print(tempC);
  // Serial.println("*C");
	// Serial.print("Ambient = "); 
  // Serial.print(mlx.readAmbientTempF());
	// Serial.print("*F\tObject = "); 
  // Serial.print(mlx.readObjectTempF()); Serial.println("*F");

	Serial.println();
	delay(500);
}