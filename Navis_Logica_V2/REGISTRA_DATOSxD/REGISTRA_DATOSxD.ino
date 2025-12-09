#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
// #include <Adafruit_BMP085.h> no se va usar; se usa el BME
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>
#include <math.h>

//------------------SPI (ESP32-S3 FSPI)--------------------
#define SCK   36   // FSPICLK
#define MISO  37   // FSPIQ
#define MOSI  35   // FSPID
#define SD_CS 11   // CS 

#define SIGN_LED 4 // LED de señal 
#define BUZZER 14  // Buzzer (libre, no USB/JTAG)

//-------- Botón de finalización de vuelo ------
#define BUTTON_PIN 20   // Botón para terminar loop y cerrar archivo

//-------- Sistema de Recuperación ------
#define PIN_RECUPERACION1 21 
#define PIN_RECUPERACION2 22

#define PIN_BATERIA1 26
#define PIN_BATERIA2 27

// --------------- I2C ------------------ 
#define GY_SDA   41    // SDA 
#define GY_SCL   42    // SCL

//---------ESTADOS-----------
#define ESTADO_1_STANDBY_VUELO      1
#define ESTADO_2_DESCENSO_ATERRIZAJE 2
#define ESTADO_3_RECOVERY           3

//-----------------MISCELANEO --------
#define PresionMarHPa (1013.25)
#define anguloDeclinacion 3.833 // ángulo en Tenango Morelos

File   registro;
char   filename[20];
int    numvuel = 0;
unsigned long lineCount = 0;   // para flush periódico

bool mpuOK = false; // para su inicialización
bool bmeOK = false; // funciona o no
bool magOK = false;
bool cardOK = false;
bool fileOK = false;

bool apogeoDetectado = false;
bool recuperacion    = false;

int numestado = 0;

#define FREQ_STANDBY_HZ   20.0   // parpadeo rápido en STANDBY
#define FREQ_RECOVERY_HZ  1.0    // parpadeo lento en RECOVERY

unsigned long _led_last_toggle   = 0;
unsigned long _led_half_period_ms = 0;  // medio período del parpadeo
bool          _led_state          = false;

unsigned long tiempoMuestreoAnterior = 0;
unsigned long intervaloDeMuestreo = 100;

//------------------ SENSORES
Adafruit_MPU6050 mpu;
Adafruit_BME280  bme;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// ------------------ VARIABLES GLOBALES ------------------
float altitudMax     = 0;
float altitudActual  = 0;
float altitudInicial = 0;
float presionInicial = 0;
float aceleracionTotal = 0;
float voltaje        = 0;

// ------------------ PROTOTIPOS
void gestionEstados(unsigned long tiempoActual);
void logFila(float altitud, float headingDeg, const char* cardinal,
             float ax, float ay, float az);
void Datos(float altitudActual, float headingDeg, const char* cardinal);
void setBlinkHz(float hz);
void updateBlink();
void enableMpu6050Bypass(); // <--- PROTOTIPO AGREGADO

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
    Serial.print("Reintento MPU: ");
    Serial.println(reintentos);
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
    Serial.print("Reintento BME: ");
    Serial.println(reintentos);
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
    Serial.print("Reintento HMC: ");
    Serial.println(reintentos);
    delay(50);
  }
  Serial.println("HMC no responde");
  return false;
}

// ---- Debug SD
void debugSD() {
  uint8_t type = SD.cardType();
  Serial.print("Tipo de tarjeta: ");
  if (type == CARD_NONE)      Serial.println("NINGUNA");
  else if (type == CARD_MMC)  Serial.println("MMC");
  else if (type == CARD_SD)   Serial.println("SDSC");
  else if (type == CARD_SDHC) Serial.println("SDHC/SDXC");
  else                        Serial.println("Desconocida");

  uint64_t sizeMB = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.print("Tam.: ");
  Serial.print(sizeMB);
  Serial.println(" MB");
}

bool inicializarSD() { //--------SDCard(memoria)
  Serial.println("Inicializando Tarjeta SD...");

  // Asegurar CS estable antes de comenzar
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(5);

  // SPI ya configurado en setup con tus pines
  const uint32_t FREQ1 = 4000000;   // 4 MHz
  const uint32_t FREQ2 = 10000000;  // 10 MHz

  if (!SD.begin(SD_CS, SPI, FREQ1)) {
    Serial.println("SD falló a 4 MHz; probando 10 MHz...");
    if (!SD.begin(SD_CS, SPI, FREQ2)) {
      Serial.println("La tarjeta SD NO responde");
      return false;
    }
  }
  Serial.println("SD inicializada");
  debugSD();
  return true;
}

//-------------------------CREACIÓN DE ARCHIVO DE VUELO-----------
//crea, abre registro, escribe encabezado y mantiene abierto.
bool nuevoArchivo() {
  if (!cardOK) {
    Serial.println("No hay tarjeta SD inicializada.");
    return false;
  }

  // Buscar nombre libre: V000.CSV ... V999.CSV
  while (numvuel < 1000) {
    snprintf(filename, sizeof(filename), "/V%03d.CSV", numvuel);
    if (!SD.exists(filename)) break;  // encontrado nombre libre
    numvuel++;
  }

  if (numvuel >= 1000) {
    Serial.println("Se alcanzó el límite de archivos V000..V999");
    return false;
  }

  // Abrir archivo global de registro
  registro = SD.open(filename, FILE_WRITE); // crea y abre para escribir
  if (!registro) {
    Serial.print("Error al crear archivo: ");
    Serial.println(filename);
    return false;
  }

  Serial.print("Creando archivo de registro: ");
  Serial.println(filename);

  // Encabezado CSV
  registro.println("Tiempo [ms],Altitud [m],Heading [deg],Dir,AccX [m/s^2],AccY [m/s^2],AccZ [m/s^2]");
  registro.flush();  // garantizar que el encabezado se escriba
  fileOK   = true;
  lineCount = 0;
  numvuel++;         // siguiente vuelo usará el siguiente índice

  return true;
}

// ---------------------- CALIBRACIÓN ----------------------
float PresionInicial() { //Calculo de Presión Inicial
  if (!bmeOK) return 0.0;

  Serial.println("Calibrando presión inicial...");
  float sumaPresiones = 0;
  const int muestras = 50; //muestras a tomar
  for (int i = 0; i < muestras; i++) {
    sumaPresiones += bme.readPressure(); //Suma Total de Presiones
    delay(10); //0.5 [s] para calculo de presión inicial 
  }
  presionInicial = sumaPresiones / muestras; //Obtención de Presion inicial promedio

  Serial.print("Presión inicial: ");
  Serial.print(presionInicial);
  Serial.println(" [Pa]");
  return presionInicial;
}

////////////////////////////////////////////////

void calculaOrientacion(float &headingDegrees, const char* &cardinal) { //Calibración magnetómetro
  if (!magOK) return;

  sensors_event_t event;
  mag.getEvent(&event);
      
  float headingRad = atan2(event.magnetic.y, event.magnetic.x);
  headingDegrees = headingRad * 180.0 / PI;
  headingDegrees += anguloDeclinacion; //Declinación magnética aprox.

  if (headingDegrees < 0)   headingDegrees += 360;
  if (headingDegrees >= 360) headingDegrees -= 360;

  // Dirección Cardinal
  if (headingDegrees > 348.75 || headingDegrees <= 11.25) cardinal = "N";
  else if (headingDegrees <= 33.75)  cardinal = "NNE";
  else if (headingDegrees <= 56.25)  cardinal = "NE";
  else if (headingDegrees <= 78.75)  cardinal = "ENE";
  else if (headingDegrees <= 101.25) cardinal = "E";
  else if (headingDegrees <= 123.75) cardinal = "ESE";
  else if (headingDegrees <= 146.25) cardinal = "SE";
  else if (headingDegrees <= 168.75) cardinal = "SSE";
  else if (headingDegrees <= 191.25) cardinal = "S";
  else if (headingDegrees <= 213.75) cardinal = "SSW";
  else if (headingDegrees <= 236.25) cardinal = "SW";
  else if (headingDegrees <= 258.75) cardinal = "WSW";
  else if (headingDegrees <= 281.25) cardinal = "W";
  else if (headingDegrees <= 303.75) cardinal = "WNW";
  else if (headingDegrees <= 326.25) cardinal = "NW";
  else                                cardinal = "NNW";
}
//////////////////////////////////////////////////////////////////////////////////
   
float calcularAltitud() {
  if (!bmeOK) return 0.0;

  float presionActual = bme.readPressure();  // Lectura de presión actual
  float ratio = presionActual / presionInicial;
  float altitud = 44330.0f * (1.0f - pow(ratio, 1.0f / 5.255f)); //Calculo de altitud

  if (altitud > altitudMax) {
    altitudMax = altitud; //actualización de altitud
  } 
  altitudActual = altitud;
  return altitudActual;
}

//---------Aceleración------
float leerAceleracion(float &ax, float &ay, float &az){
  sensors_event_t aEvent, gEvent, tempEvent;
  mpu.getEvent(&aEvent, &gEvent, &tempEvent);
  ax = aEvent.acceleration.x;
  ay = aEvent.acceleration.y;
  az = aEvent.acceleration.z;
  aceleracionTotal = sqrt(ax * ax + ay * ay + az * az);
  return aceleracionTotal;
}

//---------APOGEO-------------
void confirmarApogeo(float altitudActual){  //Detección de apogeo 
  if (!apogeoDetectado && altitudActual < altitudMax) {
    apogeoDetectado = true; 
    Serial.print("Apogeo detectado: ");
    Serial.print(altitudMax);
    Serial.println(" [m]");
            
    digitalWrite(PIN_RECUPERACION1, LOW);
    digitalWrite(PIN_RECUPERACION2, LOW);  

    Serial.println("Sistema de recuperación activado");
    digitalWrite(PIN_RECUPERACION1, HIGH); // Activar sistema de recuperación
    digitalWrite(PIN_RECUPERACION2, HIGH); // LED encendido (simulación de carga pirotécnica)
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//------------------ATERRIZAJE----------------
bool confirmarAterrizaje(float altitudActual, float altitudInicial){
  bool altitudCercana = fabs(altitudActual - altitudInicial) < 8.0;

  if (altitudCercana) {
    Serial.println("Cohete ha aterrizado");
    Serial.print("Altitud final: ");
    Serial.println(altitudActual);
    Serial.print("Aceleración final: ");
    Serial.println(aceleracionTotal);
    return true;
  }
  return false;
}

//------------------RECUPERACIÓN------
void procesoRecuperacion() {
  static bool done = false;
  if (done) return;   // solo una vez
  done = true;

  Serial.println("Proceso de recuperación...");

  float headingDeg = 0.0f;
  const char* cardinal = "N/A";
  calculaOrientacion(headingDeg, cardinal);
  Datos(altitudActual, headingDeg, cardinal);

  // Señal visual o sonora para localizar el cohete
  for (int i = 0; i < 5; i++) {
    digitalWrite(SIGN_LED, HIGH);
    delay(200);
    digitalWrite(SIGN_LED, LOW);
    delay(200);
  }

  // Última escritura a SD
  if (cardOK && fileOK && registro) {
    registro.println("---- Fin del vuelo ----");
    registro.flush();
    registro.close();
    fileOK = false;
    Serial.println("Datos finales escritos y archivo cerrado.");
  }

  Serial.println("Cohete en recuperación... fin del registro de datos");
}

//--------Recopilación de Datos (solo Serial)
void Datos(float altitudActual, float headingDeg, const char* cardinal) {
  Serial.print("Altitud: ");
  Serial.print(altitudActual);
  Serial.print(" [m] | Max: ");
  Serial.print(altitudMax); 
  Serial.print(" [m] | Heading: ");
  Serial.print(headingDeg);
  Serial.print("° | Dirección: ");
  Serial.println(cardinal);
}

// ---------------- NIVEL BATERIA ----
float voltajeBateria() {
  int lectura_c1 = analogRead(PIN_BATERIA1);
  int lectura_c2 = analogRead(PIN_BATERIA2); // Leer valor ADC
  float voltaje_c1 = (lectura_c1 * 3.3f / 4095.0f) * (138.0f / 91.0f); // Divisor
  float voltaje_c2 = (lectura_c2 * 3.3f / 4095.0f) * (138.0f / 91.0f); 
  float bateria    = voltaje_c2 + voltaje_c1;

  Serial.print("Voltaje celda1: ");
  Serial.print(voltaje_c1);
  Serial.println(" [V]");
  Serial.print("Voltaje celda2: ");
  Serial.print(voltaje_c2);
  Serial.println(" [V]");
  Serial.print("Voltaje bateria: ");
  Serial.print(bateria);
  Serial.println(" [V]");
    
  return bateria;
}

// ---------- Escribir una fila en el CSV ----------
// Usa archivo global registro y flush periódico
void logFila(float altitud, float headingDeg, const char* cardinal,
             float ax, float ay, float az) {
  if (!cardOK || !fileOK || !registro) return;

  unsigned long t = millis();
  registro.print(t);           registro.print(',');
  registro.print(altitud, 3);  registro.print(',');
  registro.print(headingDeg, 2); registro.print(',');
  registro.print(cardinal);    registro.print(',');
  registro.print(ax, 3);       registro.print(',');
  registro.print(ay, 3);       registro.print(',');
  registro.println(az, 3);

  lineCount++;
  if (lineCount % 25 == 0) {   // flush cada 25 líneas aprox.
    registro.flush();
  }
}

// ---------- Parpadeo LED ----------
void setBlinkHz(float hz) {
  if (hz <= 0.0f) {
    _led_half_period_ms = 0;
    digitalWrite(SIGN_LED, LOW);
    _led_state = false;
    return;
  }
  _led_half_period_ms = (unsigned long)(1000.0f / (2.0f * hz)); // medio periodo
  _led_last_toggle = millis();
}

void updateBlink() {
  if (_led_half_period_ms == 0) return; // desactivado
  unsigned long now = millis();
  if (now - _led_last_toggle >= _led_half_period_ms) {
    _led_last_toggle = now;
    _led_state = !_led_state;
    digitalWrite(SIGN_LED, _led_state ? HIGH : LOW);
  }
}

void onEnterState(int st) {
  switch (st) {
    case ESTADO_1_STANDBY_VUELO:
      setBlinkHz(FREQ_STANDBY_HZ);   // parpadeo rápido
      break;
    case ESTADO_2_DESCENSO_ATERRIZAJE:
      // podrías poner otra frecuencia si lo deseas
      // setBlinkHz(2.0);
      break;
    case ESTADO_3_RECOVERY:
      setBlinkHz(FREQ_RECOVERY_HZ);  // parpadeo lento
      break;
  }
}

void setup() {
  unsigned long t_init0 = millis();
  Serial.begin(115200);

  /* Inicializa I2C (ESP32-S3) */ 
  Wire.begin(GY_SDA, GY_SCL, 400000);

  /* Inicializa SPI (ESP32-S3) */
  SPI.begin(SCK, MISO, MOSI);

  //Inicialización de LED y Buzzer
  pinMode(SIGN_LED, OUTPUT);
  pinMode(BUZZER,   OUTPUT);
  pinMode(PIN_RECUPERACION1, OUTPUT);
  pinMode(PIN_RECUPERACION2, OUTPUT);
  digitalWrite(SIGN_LED, LOW);
  digitalWrite(BUZZER,   LOW);
  digitalWrite(PIN_RECUPERACION1, HIGH);
  digitalWrite(PIN_RECUPERACION2, HIGH);

  // Botón para terminar loop y guardar archivo
  pinMode(BUTTON_PIN, INPUT_PULLUP); // botón a GND

  // ESTADO 0 Inicialización de sensores
  
  // 1. PRIMERO inicializamos MPU
  mpuOK = inicializarMPU();
  
  // 2. ¡CRUCIAL! Activamos el BYPASS justo aquí para que la brújula sea visible
  enableMpu6050Bypass();
  
  // 3. AHORA SÍ inicializamos el HMC
  magOK = inicializarHMC();
  
  bmeOK = inicializarBME();

  // SD al final (con SPI ya listo)
  cardOK = inicializarSD();
  fileOK = cardOK ? nuevoArchivo() : false;

  bool todoOK = mpuOK && magOK && bmeOK && cardOK && fileOK;

  // TRANSICIÓN ESTADO 0 -> ESTADO 1
  if (!todoOK) {
    Serial.println("No se han inicializado todos los sensores! No se pasará a la siguiente fase.");
    for (int i = 0 ; i < 5 ; i++) { //Señal de ERROR
      digitalWrite(SIGN_LED, HIGH);
      delay(100);
      digitalWrite(SIGN_LED, LOW);
      delay(100);
    }
  } else {
    Serial.println("Todos los sistemas inicializados. Se pasará a Standby");
    digitalWrite(SIGN_LED, HIGH);
    delay(500);
    digitalWrite(SIGN_LED, LOW);
    numestado = ESTADO_1_STANDBY_VUELO;
    onEnterState(numestado);
  }

  PresionInicial(); //Calibrando Presión Inicial

  if (!bmeOK) {
    Serial.println("Error al iniciar BME280");
  } else {
    altitudInicial = 44330.0f * (1.0f - pow(presionInicial / 101325.0f, 1.0f / 5.255f));
    Serial.print("Altitud inicial: ");
    Serial.println(altitudInicial);
  }  

  unsigned long t_init1 = millis();
  unsigned long t_init  = t_init1 - t_init0;
  Serial.print("El tiempo de inicialización es : ");
  Serial.println(t_init);
}

void loop() {
  static bool stopRequested = false;
  static int  lastButtonState = HIGH;

  // Lectura del botón (activo en LOW)
  int buttonState = digitalRead(BUTTON_PIN);
  if (!stopRequested && (lastButtonState == HIGH) && (buttonState == LOW)) {
    // Flanco de bajada: se presionó el botón
    Serial.println("Botón presionado: cerrando archivo y deteniendo ejecución.");
    if (fileOK && registro) {
      registro.flush();
      registro.close();
      fileOK = false;
      Serial.println("Archivo CSV cerrado de forma segura.");
    }
    stopRequested = true;
    setBlinkHz(FREQ_RECOVERY_HZ); // parpadeo lento indicando fin
  }
  lastButtonState = buttonState;

  if (stopRequested) {
    // Sistema detenido, solo parpadeo para indicar fin
    updateBlink();
    delay(10);
    return;
  }

  unsigned long tiempo_actual = millis();
  gestionEstados(tiempo_actual);
}

//---------------GESTIÓN DE ESTADOS------
void gestionEstados(unsigned long tiempoActual) {
  updateBlink();
  static unsigned long tiempoAnterior = 0;

  if (tiempoActual - tiempoAnterior < intervaloDeMuestreo) return;
  tiempoAnterior = tiempoActual;

  // Lectura Sensores
  float ax = 0, ay = 0, az = 0;
  float headingDeg = 0.0f;      // valor por defecto
  const char* cardinal = "N/A"; // valor por defecto

  aceleracionTotal = leerAceleracion(ax, ay, az);
  float altitud = calcularAltitud();
  calculaOrientacion(headingDeg, cardinal);

  // ----- Log a SD en cada iteración -----
  logFila(altitud, headingDeg, cardinal, ax, ay, az);

  switch (numestado) {
    //-------------- ESTADO 1---------
    case ESTADO_1_STANDBY_VUELO:
      intervaloDeMuestreo = 50;  
      Datos(altitud, headingDeg, cardinal);
      voltajeBateria();

      //Detectar lanzamiento
      if (aceleracionTotal > 12.0 && altitud > altitudInicial + 1.0) {
        Serial.println("Lanzamiento detectado");
      }

      // Detectar apogeo
      confirmarApogeo(altitudActual);
      
      if (apogeoDetectado) {
        numestado = ESTADO_2_DESCENSO_ATERRIZAJE;
        onEnterState(numestado);
        Serial.println("Transición a descenso");
      }
      break;  

    //-------ESTADO 2: DESCENSO-------------
    case ESTADO_2_DESCENSO_ATERRIZAJE:
      intervaloDeMuestreo = 100;
      Datos(altitud, headingDeg, cardinal);
      voltajeBateria();      

      //Confirmar Aterrizaje
      if (confirmarAterrizaje(altitud, altitudInicial)) {
        Serial.println("Transición a estado de recuperación");
        numestado = ESTADO_3_RECOVERY;
        onEnterState(numestado);
      }
      break;

    case ESTADO_3_RECOVERY:
      intervaloDeMuestreo = 1000;
      voltajeBateria();
      procesoRecuperacion();
      break;
  }
}

// ---BYPASS ---
void enableMpu6050Bypass() {
  // Dirección del MPU6050
  byte mpuAddress = 0x68; 

  // Despertar el MPU6050 
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0x00); // Escribir 0 para despertar
  Wire.endTransmission();
  delay(50);

  //Activar I2C Bypass (Bit 1 del registro 0x37)
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x37); // Registro INT_PIN_CFG
  Wire.write(0x02); // Escribir 0x02 para activar bypass
  Wire.endTransmission();
  delay(50);
  
  Serial.println("Bypass activado en MPU6050.");
}
