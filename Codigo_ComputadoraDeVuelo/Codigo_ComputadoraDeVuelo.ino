#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <SD.h>

//------------------PINES--------------------
#define BME_SCK  //  BME280
#define BME_MISO //faltan pines
#define BME_MOSI 
#define BME_CS 

#define PresionMarHPa (1013.25)



bool mpuOK = false; //para su inicialización
bool bmeOK = false;  //funciona o no
bool hmcOK = false;


//------------------ SENSORES
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Adafruit_BMP085 bmp; // creo que solo vamos a usar el BME280 y pues este quedaría sin función??
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//-------------------------VARIABLES
float altitudMax = 0;
float altitudActual = 0;
float altitudInicial = 0;
float presionInicial = 0;

// ------------------ INICIALIZACION SENSORES
  bool inicializarMPU() { //--------MPU6050(imu)
    Serial.println("Inicializando MPU...");
    int reintentos = 0;
    while (reintentos < 3) {
      if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        Serial.println("MPU inicializado");
        return true;
      }
      reintentos++;
      Serial.print("Reintento MPU: "); Serial.println(reintentos);
      delay(50);
    }
    Serial.println("MPU no responde");
    return false;
  }

  bool inicializarBME() { //--------BME(bár.)
    Serial.println("Inicializando BME...");
    int reintentos = 0;
    while (reintentos < 3) {
      if (bme.begin(0x76)) {
        Serial.println("BME inicializado");
        return true;
      }
      reintentos++;
      Serial.print("Reintento BME: "); Serial.println(reintentos);
      delay(50);
    }
    Serial.println("BME no responde");
    return false;
  }

  bool inicializarHMC() { //--------HMC(magnetómetro)
    Serial.println("Inicializando HMC...");
    int reintentos = 0;
    while (reintentos < 3) {
      if (bme.begin()) {
        Serial.println("HMC inicializado");
        return true;
      }
      reintentos++;
      Serial.print("Reintento HMC: "); Serial.println(reintentos);
      delay(50);
    }
    Serial.println("HMC no responde");
    return false;
  }

  // ---------------------- CALIBRACIÓN ----------------------
  void PresionInicial() { //Calculo de Presión Inicial
    if (!bmeOK) return;

    Serial.println("Calibrando presión inicial...");
    float suma = 0;
    const int muestras = 50; //muestras tomadas
    for (int i = 0; i < muestras; i++) {
      sumaPresiones += bme.readPressure(); //Suma Total de Presiones
      delay(10); //0.5 [s] para calculo de presión inicial 
    }
    presionInicial = sumaPresiones / muestras; //Obteniendo Presion inicial promedio

    Serial.print("Presión inicial: "); Serial.print(presionInicial); Serial.println(" [Pa]");
  }

  

void setup() {

  Serial.begin(115200);
  Wire.begin();


  // Inicialización de sensores
  mpuOK = inicializarMPU();
  hmcOK = inicializarHMC();
  bmeOK = inicializarBME();

  PresionInicial(); //Calibrando Presión Inicial
}

void loop() {
  // put your main code here, to run repeatedly:

}
