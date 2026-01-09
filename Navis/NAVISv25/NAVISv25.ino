/*
  Version de prueba final del codigo para NAVISv1.1. 
  
  En este codigo se busco minimizar tiempo y buscar optimizar la recoleccion de datos. A su vez de hacer el codigo mas legible para su lectura
  y posible depuracion en caso de que algo llegue a fallar en cierto momento.
  He de aclarar que el unico punto de falla que pudimos observar durante las pruebas realizadas en este codigo son las conexiones de la SD, 
  mas que nada las conexiones fisicas, fuera de la falla por las conexiones de la SD no presenta ningun otro problema persistente.

  Se busco utilizar una estructura de datos en vez de variables separadas, 
  con el objetivo de hacer mas legible lo que se busca guardar. 
  No tendria sentido registrar en la memoria alguna variable de control, como contadores, banderas de estado, etc. 

  Aun falta la parte de calibrar el magnetometro estando en el momento del lanzamiento, esto mismo se podra hacer en otro codigo
  y en este solo utilizamos la implementacion final con la calibracion.
*/


#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>
#include <math.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// --- Declinación magnética --
// Tenango, Morelos: 3.833°
#define ANGULO_DECLINACION_DEG 3.833 

// ----- Pines -----
#define SCL 1
#define SDA 2
#define MOSI 35
#define MISO 37
#define SCK 36
#define CS 11

#define buzzer 14
#define stopButton 20
#define beforeFlightSwitch1 17
#define beforeFlightSwitch2 18
#define voltageMeasurement 16

#define recuIgnitor1 6
#define recuIgnitor2 7

#define ledIndicator 5
#define numPixels 1
#define neoPixel 4

File registro; 
char filename[16]; 
unsigned long lineCount = 0;
int numvuel = 0;

Adafruit_NeoPixel pixels(numPixels, neoPixel, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

bool mpuOk = false;
bool hmcOk = false;
bool bmeOk = false;
bool sdOk = false;
bool fileOk = false;

volatile bool stopRequested = false; 

// ------------------ Estructura de datos ------------------
struct dataPacket {
  unsigned long tiempo;
  float pressure;
  float alturaMSNM, alturaRelativa;
  float alturaMax = 0.0;
  float heading;
  const char* cardinal; // <--- AGREGADO PARA DIRECCIÓN
  float magX, magY, magZ;
  float gyX, gyY, gyZ;
  float accX, accY, accZ;
  float voltajeBateria;
};
dataPacket datosVuelo;

float prom = 0;
float alturaIni = 0.0;
int contadorDescenso = 0;       // variable contador de muestras
const int limiteMuestras = 20;   // limite de muestras para confirmar apogeo
const float alturaMin = 1.0;  // altura minima para activar recuperacion / 1 para pruebas
static float altitudFiltrada = 0.0;

bool apogeoDetectado = false;       // bandera - deteccion de apogeo
bool ignitorActivo = false;       //   bandera - ignitores activos
unsigned long inicioIgnitores = 0; // marca de inicio
const unsigned long duracionIgnitores = 700; // duración del pulso 

// --- Variables de Control de Tiempo ---
unsigned long tiempoUltimoLog = 0;
unsigned long tiempoUltimaImpresion = 0;
bool ledState;



// ---  ISR ---
void IRAM_ATTR ISR_stopButton() {
  stopRequested = true;
}

void setup() {
  pinMode(buzzer,    OUTPUT);
  playStartup();    // musica de inicializacion

  unsigned long t_init0 = millis();
  Serial.begin(115200);

  pixels.begin(); // inicializamos el neopixel 
  pixels.clear();
  pixels.show();

  // Inicialización: Morado
  pixels.setPixelColor(0, pixels.Color(148, 0, 211));
  pixels.show();
  
  Wire.begin(SDA, SCL); // inicializacion del protocolo i2c
  SPI.begin(SCK, MISO, MOSI, CS); // inicializacion del protocolo SPI

  Serial.println("iniciando sensores...");

  // --- INICIALIZACIÓN DE SENSORES ---
  if (!mpu.begin()) {
    Serial.println("MPU no inicializado");
    mpuOk = false;
  } else {
    mpuOk = true;
    Serial.println("MPU Ok");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    
    mpu.setI2CBypass(true); // habilita el bypass para que el magnetómetro sea visible
  }

  if (!mag.begin()) {
    Serial.println("HMC5883L no inicializado");
    hmcOk = false;
  } else {
    Serial.println("HMC5883 OK");
    hmcOk = true;
  }

  if (!bme.begin(0x76)) {
    Serial.println("BME no inicializado");
    bmeOk = false;
  } else {
    Serial.println("BME OK");
    bmeOk = true;
  }

  if(!SD.begin(CS)){
    Serial.println("SD no inicializada");
    sdOk = false;
  } else {
    Serial.println("SD OK");
    sdOk = true;
    fileOk = initFileSD();
  }

  // inicializacion de GPIOS
  pinMode(ledIndicator, OUTPUT);
  
  pinMode(recuIgnitor1, OUTPUT);
  pinMode(recuIgnitor2, OUTPUT);

  pinMode(beforeFlightSwitch1, INPUT); 
  pinMode(beforeFlightSwitch2, INPUT);
  
  // --- CONFIGURACIÓN DE LA INTERRUPCIÓN ---
  pinMode(stopButton, INPUT); 
  attachInterrupt(digitalPinToInterrupt(stopButton), ISR_stopButton, FALLING); // tomaremos el boton de paro como una interrupcion

  digitalWrite(ledIndicator, LOW);
  digitalWrite(recuIgnitor1, LOW);
  digitalWrite(recuIgnitor2, LOW);

  Serial.println("estableciendo altura con respecto al nivel del suelo..");

  // toma de muestras para altura relativa
  for(int i = 0; i < 50; i++){
    prom += bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(10);
  }
  alturaIni = prom / 50;
  
  Serial.printf("altura inicial %f msnm\n", alturaIni);
  
  // Info de calibración
  Serial.print("Declinacion Magnetica Configurada: ");
  Serial.println(ANGULO_DECLINACION_DEG);

  unsigned long t_init1 = millis();
  unsigned long ti = t_init1 - t_init0;
  Serial.printf("tiempo de inicializacion: %lu ms\n", ti);

  bool todoOk = bmeOk && hmcOk && mpuOk && sdOk && fileOk;
  
  if(todoOk == false){
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Rojo si error
    pixels.show();
    playFailure();
    Serial.println("algun sensor no inicializado, esperando reset...");
    esp_deep_sleep_start(); // duerme en caso de fallo
  }
  
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Verde: Sistema OK -> esperando switches
  pixels.show();
  playSuccess();
  
  Serial.println("esperando activación de switches...");

  unsigned long previousMillis = 0;
  const long interval = 250; 

  // mientras ambos botones esten en cero (accionados o que aun no se retira el "remove before flight") ejecutamos una rutina de encendido y apagado del led
  while (digitalRead(beforeFlightSwitch1) == LOW || digitalRead(beforeFlightSwitch2) == LOW) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(ledIndicator, ledState);
    }
  }

  Serial.println("Switches activados. Iniciando grabación de datos.");
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Azul: Grabando
  pixels.show();

  // terminamos fase inicial, pasamos al grabado de datos
}

void loop() {
  unsigned long ahora = millis(); // iniciamos marca de tiempo actual 

  if (stopRequested) {  // preguntamos si la bandera de stop esta en alto
    leerSensores(); // realiza por ultima vez una lectura
    guardarDatosSD(); // guardamos datos
    
    registro.flush(); // se hace un flush para enviar datos faltantes en caso de haber
    registro.close(); // cerramos el archivo
    Serial.println("Boton de paro presionado");

    // Cambiar LED a Verde
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));  
    pixels.show();
    esp_deep_sleep_start(); // dormimos y esperamos reset ...
  }
  
  // tasa de muestreo cada 50ms o 20Hz
  if (ahora - tiempoUltimoLog >= 50) { // checo si entre la ultima vez que lo hice y ahora hay 50ms
    tiempoUltimoLog = ahora; // marca de tiempo dentro de bucle (vez que lo hice)
    
    leerSensores();
    detectarApogeo();
    guardarDatosSD(); 
  }

  // verificamos "a la vez" el estado de los ignitores
  if (ignitorActivo) {
    if (millis() - inicioIgnitores >= duracionIgnitores) { // hacemos la diferencia de tiempos entre cuando se activo y el tiempo actual
      digitalWrite(recuIgnitor1, LOW);
      digitalWrite(recuIgnitor2, LOW);
      ignitorActivo = false;
      Serial.println("ignitores apagados");
    }
  }
  // imprimimos por puerto serie, en implementacion final, se puede obviar esta seccion
  if (ahora - tiempoUltimaImpresion >= 500) {
    tiempoUltimaImpresion = ahora;
   
    Serial.printf("T: %lu | Alt: %.2f | Dir: %s (%.0f°) | Bat: %.2f V | heading: %.2f\n", 
      datosVuelo.tiempo, datosVuelo.alturaRelativa, datosVuelo.cardinal, datosVuelo.heading, datosVuelo.voltajeBateria, datosVuelo.heading);
  }


}

bool initFileSD() {
  uint64_t cardSize = SD.cardSize() / (1024 * 1024); // brindamos el valor de la  bateria en MB
  Serial.printf("Tamaño de la SD: %llu MB\n", cardSize);

  while (numvuel < 1000) { // num vuel inicial -> 0 y verificamos cual numero de vuelo no se ha utilizado hasta el 999
    snprintf(filename, sizeof(filename), "/V%03d.CSV", numvuel); // construimos un nombre de archivo
    if (!SD.exists(filename)) break; // si el archivo no existe, salimos del bucle
    numvuel++;
  }

  if (numvuel >= 1000) { // verificamos que no hayamos pasado el limite de archivos
    Serial.println("Limite archivos alcanzado");
    return false;
  }

  registro = SD.open(filename, FILE_WRITE); // abrimos el archivo con el nombre anteriormente creado
  
  if (!registro) {
    Serial.printf("Error al crear: %s\n", filename);
    return false;
  }

  Serial.printf("Archivo creado: %s\n",filename);

  registro.println("Tiempo [ms],Presion [Pa],Altitud [msnm],Altura Relativa [m],Altura Maxima Relativa [m],Heading [deg],Direccion,AccX [m/s^2],AccY [m/s^2],AccZ [m/s^2],gyX [deg/s],gyY [deg/s],gyZ [deg/s],magX [uT],magY [uT],magZ [uT],voltaje Bateria [V]");
  // registramos el header para cada dato
  registro.flush();// aseguramos escritura

  return true;
}

void leerSensores() {
  datosVuelo.tiempo = millis(); // obtenemos marca de tiempo en datos
  // declaramos  datos como eventos de los sensores (capa de abstraccion libreria de Adafruit unified sensor)
  sensors_event_t a, g, temp;
  sensors_event_t m;

  // obtenemos los datos de la MPU
  mpu.getEvent(&a, &g, &temp); // registros de tempertatura: 65 y 66
  datosVuelo.accX = a.acceleration.x;
  datosVuelo.accY = a.acceleration.y;
  datosVuelo.accZ = a.acceleration.z;

  // datos del giroscopio en deg/s  
  datosVuelo.gyX = g.gyro.x * (180.0 / PI); 
  datosVuelo.gyY = g.gyro.y * (180.0 / PI);
  datosVuelo.gyZ = g.gyro.z * (180.0 / PI);

  // obtenemos los datos del magenetometro
  mag.getEvent(&m);
  datosVuelo.magX = m.magnetic.x;
  datosVuelo.magY = m.magnetic.y;
  datosVuelo.magZ = m.magnetic.z;

  // --- CALCULO DE RUMBO CON DECLINACIÓN---
  float headingRad = atan2(datosVuelo.magY, datosVuelo.magX); 

  // Aplicar la Declinación Magnética de Tenango (3.833 grados convertidos a radianes)
  float declinationRad = ANGULO_DECLINACION_DEG * (PI / 180.0); 
  headingRad += declinationRad;

  // Normalización
  if(headingRad < 0) headingRad += 2*PI;
  if(headingRad > 2*PI) headingRad -= 2*PI;

  // Convertir a Grados
  float headingDeg = headingRad * 180.0 / PI;
  datosVuelo.heading = headingDeg;

  // --- PUNTOS CARDINALES---
  if (headingDeg > 348.75 || headingDeg <= 11.25) datosVuelo.cardinal = "N";
  else if (headingDeg <= 33.75)  datosVuelo.cardinal = "NNE";
  else if (headingDeg <= 56.25)  datosVuelo.cardinal = "NE";
  else if (headingDeg <= 78.75)  datosVuelo.cardinal = "ENE";
  else if (headingDeg <= 101.25) datosVuelo.cardinal = "E";
  else if (headingDeg <= 123.75) datosVuelo.cardinal = "ESE";
  else if (headingDeg <= 146.25) datosVuelo.cardinal = "SE";
  else if (headingDeg <= 168.75) datosVuelo.cardinal = "SSE";
  else if (headingDeg <= 191.25) datosVuelo.cardinal = "S";
  else if (headingDeg <= 213.75) datosVuelo.cardinal = "SSW";
  else if (headingDeg <= 236.25) datosVuelo.cardinal = "SW";
  else if (headingDeg <= 258.75) datosVuelo.cardinal = "WSW";
  else if (headingDeg <= 281.25) datosVuelo.cardinal = "W";
  else if (headingDeg <= 303.75) datosVuelo.cardinal = "WNW";
  else if (headingDeg <= 326.25) datosVuelo.cardinal = "NW";
  else                           datosVuelo.cardinal = "NNW";
  // ----------------------------------------------------

  datosVuelo.pressure = bme.readPressure(); 
  datosVuelo.alturaMSNM   = bme.readAltitude(SEALEVELPRESSURE_HPA); // obtenemos altura directamente de la libreria 
  datosVuelo.alturaRelativa = datosVuelo.alturaMSNM - alturaIni; // calculamos la altura relativa con respecto al suelo.

  uint16_t rawBat = analogRead(voltageMeasurement);
  datosVuelo.voltajeBateria = (rawBat / 4095.0) * 3.3 * 3.0; 
}

void guardarDatosSD() {

  registro.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    datosVuelo.tiempo,  
    datosVuelo.pressure,    
    datosVuelo.alturaMSNM, datosVuelo.alturaRelativa, datosVuelo.alturaMax,
    datosVuelo.heading,
    datosVuelo.cardinal, 
    datosVuelo.accX, datosVuelo.accY, datosVuelo.accZ,
    datosVuelo.gyX,  datosVuelo.gyY,  datosVuelo.gyZ,       
    datosVuelo.magX, datosVuelo.magY, datosVuelo.magZ,
    datosVuelo.voltajeBateria 
  );
  
  registro.flush(); 
}

void detectarApogeo() {

  if (apogeoDetectado || datosVuelo.alturaRelativa < alturaMin) return;

  if (datosVuelo.alturaRelativa > datosVuelo.alturaMax) {
    datosVuelo.alturaMax = datosVuelo.alturaRelativa;
    contadorDescenso = 0; // reiniciamos contador porque seguimos subiendo
  } 
  // verificar descenso (restamos 0.5 para evitar valores inestables)
  else if (datosVuelo.alturaRelativa < (datosVuelo.alturaMax - 0.5)) {
    contadorDescenso++;
  }
  // confirmamos apogeo
  if (contadorDescenso >= limiteMuestras) {
    apogeoDetectado = true;
    Serial.println("APOGEO DETECTADO");
    
    // encendemos ignitores
    digitalWrite(recuIgnitor1, HIGH);
    digitalWrite(recuIgnitor2, HIGH);
    
    ignitorActivo = true;
    inicioIgnitores = millis();
    
    pixels.setPixelColor(0, pixels.Color(255, 165, 0)); // Naranja -> descenso
    pixels.show();
  }
}

// Funciones auxiliares
void playStartup() {
  tone(buzzer, 880, 100);  
  delay(100);
  tone(buzzer, 1109, 100); 
  delay(100);
  tone(buzzer, 1318, 100); 
  delay(100);
  noTone(buzzer);           

  delay(200);
  tone(buzzer, 1760, 400); 
  delay(400);
  noTone(buzzer);
}
void playSuccess() {
  tone(buzzer, 880, 100);  
  delay(200);
  tone(buzzer, 1109, 100); 
  delay(200);
  noTone(buzzer);
}
void playFailure() {
  tone(buzzer, 500, 200); 
  delay(200);
  tone(buzzer, 300, 200); 
  delay(200);
  tone(buzzer, 150, 400); 
  delay(400);
  noTone(buzzer);
}