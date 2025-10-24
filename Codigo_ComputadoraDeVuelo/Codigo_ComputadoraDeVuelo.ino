#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>

//------------------PINES--------------------
#define BME_SCK  //  BME280
#define BME_MISO //faltan pines
#define BME_MOSI 
#define BME_CS 

#define SD_CS // SD por SPI

#define SIGN_LED // LED para las señales

//-----------------MISCELANEO --------
#define PresionMarHPa (1013.25)
#define anguloDeclinacion 3.883
File registro;
char filename[20];
int numvuel = 0;

bool mpuOK = false; //para su inicialización
bool bmeOK = false;  //funciona o no
bool magOK = false;
bool cardOK = false;
bool fileOK = false;

bool apogeo = false;
bool recuperacion = false;

int numestado = 0;

unsigned long tiempoMuestreoAnterior = 0;
unsigned long intervaloDeMuestreo = 100;

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
      if (mag.begin()) {
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

    bool inicializarSD() { //--------SDCard(memoria )
    Serial.println("Inicializando Tarjeta SD...");
    int reintentos = 0;
    while (reintentos < 3) {
      if (SD.begin(SD_CS)) {
        Serial.println("SD inicializada");
        return true;
      }
      reintentos++;
      Serial.print("Reintento SD: "); Serial.println(reintentos);
      delay(50);
    }
    Serial.println("La tarjeta SD no responde");
    return false;
  }

  //-------------------------CREACIÓN DE ARCHIVO DE VUELO-----------
  bool nuevoArchivo(){
    if (cardOK==false) return false;

    do {
      snprintf(filename, sizeof(filename),"VUELO_%03i.csv", numvuel++);
    } while (SD.exists(filename));

    registro = SD.open(filename, FILE_WRITE);
    if (registro) {
      Serial.print("Creando archivo de registro... ");
      Serial.println(filename);
      registro.println("Tiempo [ms],Altitud [km],Heading,Roll,Pitch,AccX,AccY,AccZ");
      registro.close();
      return true;
    } else {
      Serial.println("Error al crear archivo");

      return false;      
    }
  }

  // ---------------------- CALIBRACIÓN ----------------------
  void PresionInicial() { //Calculo de Presión Inicial
    if (!bmeOK) return;

    Serial.println("Calibrando presión inicial...");
    float suma = 0;
    const int muestras = 50; //muestras a tomar
    for (int i = 0; i < muestras; i++) {
      sumaPresiones += bme.readPressure(); //Suma Total de Presiones
      delay(10); //0.5 [s] para calculo de presión inicial 
    }
    presionInicial = sumaPresiones / muestras; //Obtención de Presion inicial promedio

    Serial.print("Presión inicial: "); Serial.print(presionInicial); Serial.println(" [Pa]");
  }

///////////////////

  void calculaOrientacion(float &headingDegrees, const char* &cardinal) { //Calibración magnetómetro
    if (!magOK) return;

    Serial.println("Calibrando magnetómetro...");
  
    sensors_event_t event;
    mag.getEvent(&event);
     
   float headingRad = atan2(event.magnetic.y, event.magnetic.x);
    headingGrados = headingRad * 180.0 / PI;
    headingGrados += anguloDeclinacion; //Declinación magnética aprox. de Morelos?? es donde es el lanzamiento


      if (headingGrados < 0) headingGrados += 360;
      if (headingGrados >= 360) headingGrados -= 360;

      // Dirección Cardinal
      if (headingGrados > 348.75 || headingGrados <= 11.25) cardinal = "N";
      else if (headingGrados <= 33.75) cardinal = "NNE";
      else if (headingGrados <= 56.25) cardinal = "NE";
      else if (headingGrados <= 78.75) cardinal = "ENE";
      else if (headingGrados <= 101.25) cardinal = "E";
      else if (headingGrados <= 123.75) cardinal = "ESE";
      else if (headingGrados <= 146.25) cardinal = "SE";
      else if (headingGrados <= 168.75) cardinal = "SSE";
      else if (headingGrados <= 191.25) cardinal = "S";
      else if (headingGrados <= 213.75) cardinal = "SSW";
      else if (headingGrados <= 236.25) cardinal = "SW";
      else if (headingGrados <= 258.75) cardinal = "WSW";
      else if (headingGrados <= 281.25) cardinal = "W";
      else if (headingGrados <= 303.75) cardinal = "WNW";
      else if (headingGrados <= 326.25) cardinal = "NW";
      else cardinal = "NNW";
  }
//////////////////////////////////////////////////////////////////////////////////
   
 //--------TELEMETRÍA   
void Telemetria(float altitudActual, float headingDeg, const char* cardinal) {
  Serial.print("Altitud: "); Serial.print(altitudActual);
  Serial.print(" [m] | Max: "); Serial.print(altitudMax); 
  Serial.print(" [m] | Heading: "); Serial.print(headingDeg);
  Serial.print("° | Dirección: "); Serial.println(cardinal);
}

//---------------GESTION DE ESTADO-----------------
void gestionEstados(unsignedd long tiempoActual){

  switch (numestado){

    case ESTADO_1_STANDBY:
    intervaloDeMuestreo = 100; // Work In Progress (WIP)

    //TRANSICIÓN: Aqui la idea es que la variable que detone al cambio de estado sea
    // la aceleración: una aceleración abrupta (lo que indicaría que ya se lanzó el cohete)
    break;

    case ESTADO_2_VUELO:
    intervaloDeMuestreo = 10;


  
    //TRANSICIÓN: Aqui la idea es que la variable que detone al cambio de estado sea
    // EL APOGEO: 
    break;

    case ESTADO_3_DESCENSO:


    break;

    case ESTADO_4_RECOVERY:


    break;
  }


}

  

void setup() {

  Serial.begin(115200);
  Wire.begin();
  SPI.begin();

  //Inicialización de LED y Buzzer
  pinMode(SIGN_LED, OUTPUT)


  // ESTADO 0 Inicialización de sensores

  mpuOK = inicializarMPU();
  hmcOK = inicializarHMC();
  bmeOK = inicializarBME();
  cardOK = inicializarSD();
  fileOK = nuevoArchivo();

  bool todoOK = mpuok & hmcOK & bmeOK & cardOK & fileOK;

    // TRANSICIÓN ESTADO 0 -> ESTADO 1
    if (!todoOK){
      Serial.println("No se han inicializado todos los sensores! No se pasará a la siguiente fase.");
      
      for(int i = 0 ; i < 5 ; i++){ //Señal de ERROR

        digitalWrite(SIGN_LED, HIGH);
        delay(100);
        digitalWrite(SIGN_LED, low);
        delay(100);
      }
    }
    else{
      Serial.printLn("Todos los sistemas inicializados. Se pasará a Standby");
      digitalWrite(SIGN_LED, HIGH);
      delay(500);
      digitalWrite(SIGN_LED, LOW);
      numestado++;
      }

  PresionInicial(); //Calibrando Presión Inicial
}


void loop() {
  
  unsigned long tiempoActual = milis(); // Quiero limitar la tasa de muestreo para ciertos Estados:
                                        // En el Standby: limitar a 10Hz para ahorrar batería. Pero
                                        // Esto quedaría en WIP.

  //AQUÍ DEBERÍA ESTAR LA FUNCIÓN gestionEstado();

  //AQUÍ PONER LAS FUNCIONES QUE ESCRIBAN LOS DATOS EN LA SD Y QUE CALCULEN LA ALTITUD Y VELOCIDAD.
}
