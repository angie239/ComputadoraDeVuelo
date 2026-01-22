
/*
 * PROYECTO: NAVIS v1.1 - Computadora de Vuelo
 * ORGANIZACIÓN: Curiosity Aerospace (Propulsión UNAM)
 * PLATAFORMA: ESP32-S3
 * * DESCRIPCIÓN:
 * Firmware de adquisición de datos y control de recuperación.
 * Implementa un sistema de registro de datos (Datalogger) a 50Hz y detección 
 * de apogeo barométrico con filtrado de ventana para despliegue de paracaídas.
 * * ESTRUCTURA DE DATOS:
 * Se utiliza 'struct dataPacket' para serialización eficiente y 
 * coherencia en el buffer de escritura.
 * * NOTAS DE HARDWARE:
 * - Sensores: MPU6050 (IMU), HMC5883L (Magnetómetro), BME280 (Barómetro).
 * - Almacenamiento: SD Card via SPI.
 * - Indicadores: NeoPixel (WS2812B) para código de colores de estado.
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

// --- Constantes Ambientales y Geográficas ---
#define SEALEVELPRESSURE_HPA (1013.25)
// Declinación magnética para corrección de rumbo (Tenango, Morelos: ~3.833°)
#define ANGULO_DECLINACION_DEG 3.833

// --- Mapeo de Pines (Hardware Abstraction Layer) ---
#define SCL 1
#define SDA 2
#define MOSI 35
#define MISO 37
#define SCK 36
#define CS 11

// Actuadores y Periféricos
#define buzzer 14
#define stopButton 20
#define beforeFlightSwitch1 17
#define beforeFlightSwitch2 18
#define voltageMeasurement 16

// Ignitores para recuperación
#define recuIgnitor1 6
#define recuIgnitor2 7

// Indicadores de Estado
#define ledIndicator 5
#define numPixels 1
#define neoPixel 4

// --- Gestión de Archivos y Buffers ---
File registro;
char filename[16];
unsigned long lineCount = 0;
int numvuel = 0;

// --- Instancias de Objetos de Sensor ---
Adafruit_NeoPixel pixels(numPixels, neoPixel, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// --- Banderas de Estado del Sistema ---
bool mpuOk = false;
bool hmcOk = false;
bool bmeOk = false;
bool sdOk = false;
bool fileOk = false;

// Variable volátil para acceso seguro dentro de la ISR
volatile bool stopRequested = false;

// --- Estructura de Telemetría ---
// Agrupación de variables para escritura atómica en SD
struct dataPacket {
  unsigned long tiempo;       // Timestamp [ms]
  float pressure;             // Presión atmosférica [Pa]
  float alturaMSNM;           // Altitud absoluta [m]
  float alturaRelativa;       // Altitud AGL (Above Ground Level) [m]
  float alturaMax = 0.0;      // Apogeo registrado [m]
  float heading;              // Rumbo magnético corregido [deg]
  float magX, magY, magZ;     // Campo magnético [uT]
  float gyX, gyY, gyZ;        // Velocidad angular [deg/s]
  float accX, accY, accZ;     // Aceleración lineal [m/s^2]
  float voltajeBateria;       // Tensión de alimentación [V]
  bool flagIgnitor = false ; // Estados lógicos
  bool flagApogeo = false ;
};
dataPacket datosVuelo;

// --- Variables de Lógica de Vuelo ---
float prom = 0;
float alturaIni = 0.0;          // Referencia de altitud base (Ground Level)
int contadorDescenso = 0;       // Acumulador para filtro de detección de apogeo
const int limiteMuestras = 20;  // Ventana de confirmación (muestras consecutivas descendiendo)
const float alturaMin = 20.0;    // Altitud mínima AGL para armar sistema de recuperación
static float altitudFiltrada = 0.0;

// Banderas de Estado de Misión
bool apogeoDetectado = false;
bool ignitorActivo = false;
bool eMatchReady = false;
unsigned long tiempoInicioSuelo = 0;
unsigned long inicioIgnitores = 0;
const unsigned long duracionIgnitores = 750; // Tiempo de activación pirotécnica [ms]



// Control de Tiempos (Scheduler)
unsigned long tiempoUltimoLog = 0;
unsigned long tiempoUltimaImpresion = 0;
bool ledState;

// Inicialización de banderas


/**
 * @brief Rutina de Servicio de Interrupción (ISR) para Parada de Emergencia.
 * Configurada con atributo IRAM_ATTR para residir en RAM y garantizar 
 * ejecución determinista en ESP32.
 */
void IRAM_ATTR ISR_stopButton() {
  stopRequested = true;
}

void setup() {
  // Configuración de Periféricos de Salida
  pinMode(buzzer, OUTPUT);
  playStartup(); 

  unsigned long t_init0 = millis();
  Serial.begin(115200);

  // Inicialización de bus LED direccionable
  pixels.begin();
  pixels.clear();
  pixels.show();
  pixels.setPixelColor(0, pixels.Color(148, 0, 211)); // Estado: INICIO (Morado)
  
  pixels.show();
  

  // Inicialización de Protocolos de Comunicación
  Wire.begin(SDA, SCL);            // I2C
  SPI.begin(SCK, MISO, MOSI, CS);  // SPI

  // --- CONFIGURACIÓN DE SENSORES E INICIALIZACIÓN ---
  
  // 1. IMU (MPU6050)
  if (!mpu.begin()) {
    mpuOk = false;
  } else {
    mpuOk = true;
    // Configuración de rangos y filtro digital paso bajo (DLPF)
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Filtro agresivo para vibración mecánica
    mpu.setI2CBypass(true); // Bypass activado para acceso directo al Magnetómetro auxiliar
  }

  // 2. Magnetómetro (HMC5883L)
  if (!mag.begin()) {
    hmcOk = false;
  } else {
    hmcOk = true;
  }

  // 3. Barómetro (BME280)
  if (!bme.begin(0x76)) {
    bmeOk = false;
  } else {
    bmeOk = true;
  }

  // 4. Almacenamiento (SD Card)
  if (!SD.begin(CS)) {
    sdOk = false;
  } else {
    sdOk = true;
    fileOk = initFileSD(); // Gestión de nombres de archivo
  }

  // Configuración avanzada de sobremuestreo (Oversampling) y filtrado IIR del barómetro
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X1,   // Temperatura
                  Adafruit_BME280::SAMPLING_X4,   // Presión (Alta resolución)
                  Adafruit_BME280::SAMPLING_NONE,   // Humedad
                  Adafruit_BME280::FILTER_X2,     // Filtro IIR
                  Adafruit_BME280::STANDBY_MS_0_5); // Tasa de refresco máxima

  // Configuración de GPIOs
  pinMode(ledIndicator, OUTPUT);
  pinMode(recuIgnitor1, OUTPUT);
  pinMode(recuIgnitor2, OUTPUT);
  pinMode(beforeFlightSwitch1, INPUT);
  pinMode(beforeFlightSwitch2, INPUT);

  // Configuración de Interrupción Externa
  pinMode(stopButton, INPUT);
  attachInterrupt(digitalPinToInterrupt(stopButton), ISR_stopButton, FALLING); 

  // Estado seguro inicial (Actuadores apagados)
  digitalWrite(ledIndicator, LOW);
  digitalWrite(recuIgnitor1, LOW);
  digitalWrite(recuIgnitor2, LOW);

  // --- CALIBRACIÓN DE ALTITUD BASE ---
  
  // Promedio de 50 muestras para establecer altitud cero
  for (int i = 0; i < 50; i++) {
    prom += bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(10);
  }
  alturaIni = prom / 50.0;

  // Diagnóstico de tiempo de arranque
  unsigned long t_init1 = millis();

  // Validación final del sistema
  bool todoOk = bmeOk && hmcOk && mpuOk && sdOk && fileOk;

  if (!todoOk) {
    // ESTADO DE FALLO CRÍTICO
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Rojo Fijo
    pixels.show();
    playFailure();
    esp_deep_sleep_start(); // Apagado de bajo consumo para protección
  }

  // ESTADO: PRE-ARMADO (Sistema OK, esperando switches)
  pixels.setPixelColor(0, pixels.Color(235, 255, 0)); // Amarillo
  pixels.show();
  playSuccess();

  // Bucle de espera bloqueante hasta retirar pines "Remove Before Flight"
  // Implementa parpadeo de LED indicador sin bloquear lógica interna
  unsigned long previousMillis = 0;
  const long interval = 250;

  while (digitalRead(beforeFlightSwitch1) == LOW || digitalRead(beforeFlightSwitch2) == LOW) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(ledIndicator, ledState);
    }
  }

  // ESTADO: GRABACIÓN ACTIVA
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Verde Fijo
  pixels.show();
}

void loop() {
  unsigned long ahora = millis();

  // Gestión de Parada de Emergencia (Interrupción por Software)
  if (stopRequested) {
    cerrarSistema();
    playSaved();
    esp_deep_sleep_start();
  }

  if (apogeoDetectado) {
    // Si estamos por debajo de 10 metros
    if (datosVuelo.alturaRelativa <= 10.0) {
      
      // Si es la primera vez que detectamos <10m, iniciamos el cronómetro
      if (tiempoInicioSuelo == 0) {
        tiempoInicioSuelo = ahora;
      }
      
      // Si ya pasaron 60 segundos (60000 ms) desde que se cumplió la condición
      else if (ahora - tiempoInicioSuelo >= 60000) {
        modoRecuperacion(); // Guardar, cerrar y pitar
      }
      
    } else {
      // Si por alguna razón la altura sube a > 10m (rebote, ruido, ráfaga),
      // reseteamos el contador para asegurar que sean 60s CONTINUOS de suelo.
      tiempoInicioSuelo = 0;
    }
  }
  // --- SCHEDULER DE TAREAS ---
  
  // Tarea 1: Adquisición y Control (50 Hz / 20ms)
  if (ahora - tiempoUltimoLog >= 20) { 
    tiempoUltimoLog = ahora;
    
    leerSensores();     // Lectura de ADC y buses I2C
    detectarApogeo();   // Lógica de navegación
    guardarDatosSD();   // Serialización y guardado
  }

  // Tarea 2: Control de Actuadores
  // Lógica no bloqueante para asegurar pulso exacto de 750ms
  if (ignitorActivo) {
    if (millis() - inicioIgnitores >= duracionIgnitores) {
      digitalWrite(recuIgnitor1, LOW);
      digitalWrite(recuIgnitor2, LOW);
      ignitorActivo = false;
    }
  }

  if(!eMatchReady){
    if(datosVuelo.alturaRelativa >= alturaMin){
      eMatchReady = true;
    }
  }
}

/**
 * @brief Inicializa el sistema de archivos en la tarjeta SD.
 * Busca secuencialmente el siguiente nombre de archivo disponible (V000.CSV -> V999.CSV)
 * para evitar sobrescritura de datos de vuelos anteriores.
 * * @return true si el archivo se creó exitosamente.
 * @return false si hubo error de hardware o límite de archivos.
 */
bool initFileSD() {
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);

  while (numvuel < 1000) {
    snprintf(filename, sizeof(filename), "/V%03d.CSV", numvuel);
    if (!SD.exists(filename)) break;
    numvuel++;
  }

  if (numvuel >= 1000) {
    return false;
  }

  registro = SD.open(filename, FILE_WRITE);

  if (!registro) {
    return false;
  }


  // Escritura de cabecera CSV
  registro.println("Tiempo [ms],Presion [Pa],Altitud [msnm],Altura Relativa [m],Altura Maxima Relativa [m],Heading [deg],AccX [m/s^2],AccY [m/s^2],AccZ [m/s^2],gyX [deg/s],gyY [deg/s],gyZ [deg/s],magX [uT],magY [uT],magZ [uT],voltaje Bateria [V],FlagIgnitor,FlagApogeo");
  registro.flush();

  return true;
}

/**
 * @brief Adquisición de datos brutos y procesamiento primario.
 * Realiza:
 * 1. Lectura de buses I2C (IMU, Mag, Baro).
 * 2. Conversión de unidades (Radianes a Grados, ADC a Voltaje).
 * 3. Fusión de sensores para cálculo de rumbo (Heading) con compensación de declinación.
 */
void leerSensores() {
  datosVuelo.tiempo = millis();
  
  // Estructuras para eventos de sensores Adafruit Unified
  sensors_event_t a, g, temp, m;

  // Lectura IMU (MPU6050)
  mpu.getEvent(&a, &g, &temp);
  datosVuelo.accX = a.acceleration.x;
  datosVuelo.accY = a.acceleration.y;
  datosVuelo.accZ = a.acceleration.z;

  // Conversión de Rad/s a Deg/s
  datosVuelo.gyX = g.gyro.x * (180.0 / PI);
  datosVuelo.gyY = g.gyro.y * (180.0 / PI);
  datosVuelo.gyZ = g.gyro.z * (180.0 / PI);

  // Lectura Magnetómetro
  mag.getEvent(&m);
  datosVuelo.magX = m.magnetic.x;
  datosVuelo.magY = m.magnetic.y;
  datosVuelo.magZ = m.magnetic.z;

  // --- ALGORITMO DE RUMBO (COMPASS HEADING) ---
  float headingRad = atan2(datosVuelo.magY, datosVuelo.magX);

  // Corrección por Declinación Magnética
  float declinationRad = ANGULO_DECLINACION_DEG * (PI / 180.0);
  headingRad += declinationRad;

  // Normalización angular [0, 2*PI]
  if (headingRad < 0) headingRad += 2 * PI;
  if (headingRad > 2 * PI) headingRad -= 2 * PI;

  datosVuelo.heading = headingRad * 180.0 / PI;

  // Lectura Barométrica
  datosVuelo.pressure = bme.readPressure();
  datosVuelo.alturaMSNM = bme.readAltitude(SEALEVELPRESSURE_HPA);
  datosVuelo.alturaRelativa = datosVuelo.alturaMSNM - alturaIni;

  // Lectura de Tensión de Batería (Divisor de voltaje)
  // ADC 12-bit (0-4095) -> Referencia 3.3V -> Factor de escala del divisor (x3.0)
  uint16_t rawBat = analogRead(voltageMeasurement);
  datosVuelo.voltajeBateria = (rawBat / 4095.0) * 3.3 * 3.0;
}

/**
 * @brief Serialización y escritura en búfer SD.
 * Formato CSV estándar para post-procesamiento.
 */
void guardarDatosSD() {
  registro.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
                  datosVuelo.tiempo,
                  datosVuelo.pressure,
                  datosVuelo.alturaMSNM, datosVuelo.alturaRelativa, datosVuelo.alturaMax,
                  datosVuelo.heading,
                  datosVuelo.accX, datosVuelo.accY, datosVuelo.accZ,
                  datosVuelo.gyX, datosVuelo.gyY, datosVuelo.gyZ,
                  datosVuelo.magX, datosVuelo.magY, datosVuelo.magZ,
                  datosVuelo.voltajeBateria,
                  datosVuelo.flagIgnitor,
                  datosVuelo.flagApogeo);
                
  static int contadorFlush = 0;
  contadorFlush++;
  
  if (contadorFlush >= 50 || datosVuelo.flagApogeo) {
    registro.flush();
    contadorFlush = 0;
  }
}

/**
 * @brief Lógica de detección de Apogeo Barométrico.
 * Utiliza un contador de persistencia para filtrar ruido.
 * * Condición de disparo:
 * 1. Altitud actual < (Altitud Máxima - Hysteresis).
 * 2. Condición mantenida por 'limiteMuestras' ciclos consecutivos.
 */
void detectarApogeo() {
  // Guardas de seguridad: no disparar si ya se disparó o si estamos en rampa de lanzamiento (< alturaMin)
  if (apogeoDetectado || !eMatchReady) return;

  // Actualización de máximo global
  if (datosVuelo.alturaRelativa > datosVuelo.alturaMax) {
    datosVuelo.alturaMax = datosVuelo.alturaRelativa;
    contadorDescenso = 0; // Reset de contador si seguimos subiendo
  }
  // Verificación de descenso con histéresis de 1.5m para filtrar ráfagas de viento/ruido
  else if (datosVuelo.alturaRelativa < (datosVuelo.alturaMax - 1.5)) {
    contadorDescenso++;
  }
  else{
    contadorDescenso = 0;
  }

  // Confirmación de evento Apogeo
  if (contadorDescenso >= limiteMuestras) {
    apogeoDetectado = true;

    // Activación de etapa de recuperación
    digitalWrite(recuIgnitor1, HIGH);
    digitalWrite(recuIgnitor2, HIGH);

    ignitorActivo = true;
    inicioIgnitores = millis();
    datosVuelo.flagApogeo = true;
    datosVuelo.flagIgnitor = true;

    pixels.setPixelColor(0, pixels.Color(255, 0, 255)); // Magenta (Descenso)
    pixels.show();
  }
}

/**
 * @brief Cierre del sistema
 * Realiza una ultima lectura de sensores y guardado en SD
 * Hace un ultimo flush para evitar datos en buffer
 * cierra el archivo y setea NeoPixel en azul. 
 */

void cerrarSistema() {
    leerSensores();      
    guardarDatosSD();    
    registro.flush();    
    registro.close();    
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Azul (Guardado)
    pixels.show();
}
/**
 * @brief Modo de recuperacion
 * Reproduce en bucle dos notas en el buzzer y cambia de colores los leds para poder ser encontrado con mayor facilidad.
 */
void modoRecuperacion() {
  cerrarSistema(); // Cierra archivo SD
  
  // Bucle infinito de baliza
  while (true) {
    tone(buzzer, 2000, 200); delay(250);
    tone(buzzer, 2000, 200); delay(250);
    
    // Flash LED Blanco
    pixels.setPixelColor(0, pixels.Color(255, 255, 255)); 
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); 
    pixels.show();

    delay(500); // Pausa larga
    
    // Opción para apagar manualmente con el botón
    if (stopRequested) { 
        playSaved();
        esp_deep_sleep_start();
    }
  }
}
// --- Funciones de Feedback Acústico (Buzzer) ---
void playReading(){

  tone(buzzer, 1047, 120); delay(120); // Do6
  tone(buzzer, 1319, 120); delay(120); // Mi6
  tone(buzzer, 1568, 120); delay(120); // Sol6

}

void playStartup() {
  
  tone(buzzer, 784, 120);  delay(120); // Sol5
  tone(buzzer, 1047, 120); delay(120); // Do6
  tone(buzzer, 1319, 120); delay(120); // Mi6
  tone(buzzer, 1568, 120); delay(400); // Sol6
  tone(buzzer, 1319, 120); delay(120); // Mi6
  tone(buzzer, 1568, 400); delay(400); // Sol6 
  
  noTone(buzzer);
}

void playSuccess() {
  tone(buzzer, 880, 100); delay(200);
  tone(buzzer, 1109, 100); delay(200);
  noTone(buzzer);
}

void playFailure() {
  tone(buzzer, 500, 200); delay(200);
  tone(buzzer, 300, 200); delay(200);
  tone(buzzer, 150, 400); delay(400);
  noTone(buzzer);
}

void playSaved() {
  // --- NYAN CAT SONG ---
  tone(buzzer, 370, 170); delay(220); // FS4
  tone(buzzer, 415, 170); delay(220); // GS4
  tone(buzzer, 294, 85); delay(110);  // D4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 93, 85); delay(110);   // FS2
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 294, 85); delay(110);  // D4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 247, 170); delay(220); // B3
  tone(buzzer, 247, 170); delay(220); // B3
  tone(buzzer, 277, 170); delay(220); // CS4

  tone(buzzer, 294, 170); delay(220); // D4
  tone(buzzer, 294, 85); delay(110);  // D4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 247, 85); delay(110);  // B3
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 370, 85); delay(110);  // FS4
  tone(buzzer, 415, 85); delay(110);  // GS4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 370, 85); delay(110);  // FS4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 247, 85); delay(110);  // B3
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 247, 85); delay(110);  // B3

  tone(buzzer, 311, 170); delay(220); // DS4
  tone(buzzer, 370, 170); delay(220); // FS4
  tone(buzzer, 415, 85); delay(110);  // GS4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 370, 85); delay(110);  // FS4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 247, 85); delay(110);  // B3
  tone(buzzer, 294, 85); delay(110);  // D4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 294, 85); delay(110);  // D4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 247, 85); delay(110);  // B3
  tone(buzzer, 277, 85); delay(110);  // CS4

  tone(buzzer, 294, 170); delay(220); // D4
  tone(buzzer, 247, 85); delay(110);  // B3
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 311, 85); delay(110);  // DS4
  tone(buzzer, 370, 85); delay(110);  // FS4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 294, 85); delay(110);  // D4
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 247, 85); delay(110);  // B3
  tone(buzzer, 277, 85); delay(110);  // CS4
  tone(buzzer, 247, 170); delay(220); // B3
  tone(buzzer, 277, 170); delay(220); // CS4

    noTone(buzzer); // Apagar buzzer al terminar
}
