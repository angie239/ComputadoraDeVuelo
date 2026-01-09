#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h> // Necesaria para activar el Bypass

// --- CONFIGURACIÓN ---
#define SDA_PIN 2
#define SCL_PIN 1

// Declinación Magnética de Tenango, Morelos
#define ANGULO_DECLINACION_DEG 3.833 


Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10); // Esperar consola

  Serial.println("--- Prueba Magnetómetro ---");

  //Iniciar I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Iniciar MPU6050 y activar BYPASS
  Serial.print("Iniciando MPU6050 para Bypass... ");
  if (!mpu.begin()) {
    Serial.println("FALLO MPU6050 (No se pudo activar bypass)");
    while (1) { delay(10); }
  }
  Serial.println("OK");
  
  // Activar Bypass... permite que ESP32 hable con HMC5883L
  mpu.setI2CBypass(true);
  Serial.println("Bypass I2C activado.");

  //Iniciar Magnetómetro
  Serial.print("Iniciando MagnetómetroL... ");
  if (!mag.begin()) {
    Serial.println("FALLO Magnetómetro");
    while (1) { delay(10); }
  }
  Serial.println("OK");
  
}

void loop(void) {
  sensors_event_t event; 
  mag.getEvent(&event);

  // Valores [uTesla]
  Serial.print("Mag (uT): \tX: "); Serial.print(event.magnetic.x);
  Serial.print("\tY: "); Serial.print(event.magnetic.y);
  Serial.print("\tZ: "); Serial.print(event.magnetic.z);

  // Heading (Rumbo) 
  float headingRad = atan2(event.magnetic.y, event.magnetic.x);
  
  // Declinación (Tenango)
  float declinationRad = ANGULO_DECLINACION_DEG * (PI / 180.0); 
  headingRad += declinationRad;
  
  //Normalizando
  if(headingRad < 0) headingRad += 2*PI;
  if(headingRad > 2*PI) headingRad -= 2*PI;
  
  //En grados
  float headingDeg = headingRad * 180/M_PI; 
  
  //Dirección Cardinal
  const char* cardinal;
  if (headingDeg > 348.75 || headingDeg <= 11.25) cardinal = "N";
  else if (headingDeg <= 33.75)  cardinal = "NNE";
  else if (headingDeg <= 56.25)  cardinal = "NE";
  else if (headingDeg <= 78.75)  cardinal = "ENE";
  else if (headingDeg <= 101.25) cardinal = "E";
  else if (headingDeg <= 123.75) cardinal = "ESE";
  else if (headingDeg <= 146.25) cardinal = "SE";
  else if (headingDeg <= 168.75) cardinal = "SSE";
  else if (headingDeg <= 191.25) cardinal = "S";
  else if (headingDeg <= 213.75) cardinal = "SSW";
  else if (headingDeg <= 236.25) cardinal = "SW";
  else if (headingDeg <= 258.75) cardinal = "WSW";
  else if (headingDeg <= 281.25) cardinal = "W";
  else if (headingDeg <= 303.75) cardinal = "WNW";
  else if (headingDeg <= 326.25) cardinal = "NW";
  else                           cardinal = "NNW";

  Serial.print("\t| Heading: "); Serial.print(headingDeg);
  Serial.print(" deg \t| Dir: "); Serial.println(cardinal);
  
  delay(250); // Pausa para leer cómodamente
}