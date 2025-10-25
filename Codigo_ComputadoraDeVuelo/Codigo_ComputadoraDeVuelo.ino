  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_HMC5883_U.h>
  // #include <Adafruit_BMP085.h> no se va usar; se usa el BME
  #include <Adafruit_BME280.h>
  #include <Adafruit_MPU6050.h>
  #include <SPI.h>
  #include <SD.h>
  #include <stdio.h>

  //------------------SPI--------------------
  #define SCK   2 
  #define MISO  8
  #define MOSI  7
  #define SD_CS 5  // SD por SPI

  #define SIGN_LED 22 // LED para las señales
  #define BUZZER 23

  //-------- Sistema de Recuperación ------
  #define PIN_RECUPERACION1 20 
  #define PIN_RECUPERACION2 21

  #define PIN_BATERIA1 26
  #define PIN_BATERIA2 27

  // --------------- I2C ------------------ 
  #define GY_SCL   1
  #define GY_SDA   0 


  //------ENUMERACION DE ESTADOS-----
    enum EstadoVuelo {
      ESTADO_0_INI = 0,
      ESTADO_1_STANDBY,
      ESTADO_2_VUELO,
      ESTADO_3_DESCENSO,
      ESTADO_4_RECOVERY,
    }

    EstadoVuelo estadoActual = ESTADO_INI; 

  //-----------------CONSTANTES GLOBALES--------
  #define PresionMarHPa (1013.25)
  #define anguloDeclinacion 3.833 //ángulo en Tenango Morelos
  const float G_ACCEL = 9.81;
  const float UMBRAL_G = 4.0;
  const float UMBRAL_LANZAMIENTO_MS = (G_ACCEL*UMBRAL_G);

  //-----------------MISCELANEO --------

  File registro;
  char filename[20];
  int numvuel = 0;

  bool mpuOK = false; //para su inicialización
  bool bmeOK = false;  //funciona o no
  bool magOK = false;
  bool cardOK = false;
  bool fileOK = false;

  bool Apogeo = false;
  bool apogeoDetectado = false;
  bool recuperacion = false;


  unsigned long tiempoMuestreoAnterior = 0;
  unsigned long intervaloDeMuestreo = 100;

  //------------------ SENSORES
  Adafruit_MPU6050 mpu;
  Adafruit_BME280 bme;
  //  Adafruit_BMP085 bmp; // creo que solo vamos a usar el BME280 y pues este quedaría sin función??
  Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

  //-------------------------VARIABLES
  float altitudMax = 0;
  float altitudActual = 0;
  float altitudInicial = 0;
  float presionInicial = 0;
  float acelActual = 0;

  float voltaje = 0;

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
      float sumaPresiones = 0;
      const int muestras = 50; //muestras a tomar
      for (int i = 0; i < muestras; i++) {
        sumaPresiones += bme.readPressure(); //Suma Total de Presiones
        delay(10); //0.5 [s] para calculo de presión inicial 
      }
      presionInicial = sumaPresiones / muestras; //Obtención de Presion inicial promedio

      Serial.print("Presión inicial: "); Serial.print(presionInicial); Serial.println(" [Pa]");
      return presionInicial;
    } 

  ////////////////////////////////////////////////

    void calculaOrientacion(float &headingDegrees, const char* &cardinal) { //Calibración magnetómetro
      if (!magOK) return;

      Serial.println("Calibrando magnetómetro...");
    
      sensors_event_t event;
      mag.getEvent(&event);
      
      float headingRad = atan2(event.magnetic.y, event.magnetic.x);
      headingDegrees = headingRad * 180.0 / PI;
      headingDegrees += anguloDeclinacion; //Declinación magnética aprox. de Morelos?? es donde es el lanzamiento


        if (headingDegrees < 0) headingDegrees += 360;
        if (headingDegrees >= 360) headingDegrees -= 360;

        // Dirección Cardinal
        if (headingDegrees > 348.75 || headingDegrees <= 11.25) cardinal = "N";
        else if (headingDegrees <= 33.75) cardinal = "NNE";
        else if (headingDegrees <= 56.25) cardinal = "NE";
        else if (headingDegrees <= 78.75) cardinal = "ENE";
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
        else cardinal = "NNW";
    }
  //////////////////////////////////////////////////////////////////////////////////
    
    float calcularAltitud() {
      if (!bmeOK) return 0.0;

      float presionActual = bme.readPressure();  // Lectura de presión actual
        float ratio = presionActual / presionInicial;
        altitudActual = 44330.0 * (1.0 - pow(ratio, 1.0 / 5.255)); //Calculo de altitudActual 

        if (altitudActual > altitudMax){
          altitudMax = altitudActual; //actualización de altitud
        } 
          return altitudActual;
    }

  //---------Aceleración------
    float leerAceleracion(float &ax, float &ay, float &az){
      sensors_event_t aEvent, gEvent, tempEvent;
      mpu.getEvent(&aEvent, &gEvent, &tempEvent);
      ax = aEvent.acceleration.x;
      ay = aEvent.acceleration.y;
      az = aEvent.acceleration.z;
      return sqrt(ax*ax + ay*ay + az*az);
  }

  //---------APOGEO-------------
    bool confirmarApogeo(float altitudActual, float aceleracionTotal){  //Detección de apogeo 

      //Condiciones para confirmar Apogeo
      //Confirmación con aceleración y altitud
      if (!apogeoDetectado && altitudActual < altitudMax &&
          aceleracionTotal > 9  &&  aceleracionTotal < 10.5){ ///falta perfeccionar la calibración de rango
            apogeoDetectado = true; 
            
              Serial.print("Apogeo detectado: "); Serial.println(altitudMax); Serial.println(" [m]");
              
              digitalWrite(PIN_RECUPERACION, LOW);  // Activar sistema de recuperación
              Serial.println("Sistema de recuperación activado");

              delay(2000);  // LED encendido (similación de carga pirotécnica)
              digitalWrite(PIN_RECUPERACION, HIGH);
              return true
          }
    }

  /////////////////////////////////////////////////////////////////////////////////////////

  //--------TELEMETRÍA    
  void Telemetria(float altitudActual, float headingDeg, const char* cardinal) {
    Serial.print("Altitud: "); Serial.print(altitudActual);
    Serial.print(" [m] | Max: "); Serial.print(altitudMax); 
    Serial.print(" [m] | Heading: "); Serial.print(headingDeg);
    Serial.print("° | Dirección: "); Serial.println(cardinal);
  }

  // ---------------- NIVEL BATERIA ----
    float voltajeBateria() {
      int lectura_c1 = analogRead(PIN_BATERIA1);
      int lectura_c2 = analogRead(PIN_BATERIA2); // Leer valor ADC
      float voltaje_c1 = (lectura_c1 * 3.3 / 4095.0)* (138.0/91.0) ; // Divisor de R1= 470k[ohms] ; R2=910k[ohms]
      float voltaje_c2 = (lectura_c2 * 3.3/ 4095.0) * (138.0/91.0); 
      float bateria = voltaje_c2+voltaje_c1;
      Serial.print("Voltaje celda1: "); Serial.print(voltaje_c1); Serial.println(" [V]");
      Serial.print("Voltaje celda2: "); Serial.print(voltaje_c2); Serial.println(" [V]");
      Serial.print("Voltaje bateria: "); Serial.print(bateria); Serial.println(" [V]");
      return voltaje;
    }

  //---------------GESTION DE ESTADO-----------------
  void gestionEstados(unsigned long tiempoActual){

    switch (estadoActual){

      case ESTADO_1_STANDBY:
      //intervaloDeMuestreo = 100; // Work In Progress (WIP)
      digitalWrite(SIGN_LED, HIGH);

      acelActual = leerAceleracion(ax,ay,az);
      if (acelActual > UMBRAL_LANZAMIENTO_MS){
        estadoActual = ESTADO_2_VUELO;
        digitalWrite(SIGN_LED, HIGH);
      
      }
      break;

      case ESTADO_2_VUELO:
      //intervaloDeMuestreo = 10;


    
      //TRANSICIÓN: Aqui la idea es que la variable que detone al cambio de estado sea
      // EL APOGEO
      bool llegadaApogeo = confirmarApogeo();

      if (llegadaApogeo == true){
          estadoActual = ESTADO_3_DESCENSO

      }
      
      break;

      case ESTADO_3_DESCENSO:


      break;

      case ESTADO_4_RECOVERY:


      break;
    }


  }

  void registrarDatos(unsigned long tiempoActual) {
    
    if (tiempoActual - tiempoMuestreoAnterior < intervaloDeMuestreo) {
      return; 
    }

    tiempoMuestreoAnterior = tiempoActual;

    // --- TAREAS PESADAS
    
    // LECTURA GIROSCOPIO
    sensors_event_t aEvent, gEvent, tempEvent;
    mpu.getEvent(&aEvent, &gEvent, &tempEvent);
    gx = gEvent.gyro.x;
    gy = gEvent.gyro.y;
    gz = gEvent.gyro.z;
    
    // LECTURA ACELERACION
    acelActual = leerAceleracion(ax, ay, az); 

    // Magnetómetro y Orientación
    calculaOrientacion(heading, cardinal);
    sensors_event_t magEvent; 
    mag.getEvent(&magEvent);
    mx = magEvent.magnetic.x;
    my = magEvent.magnetic.y;
    mz = magEvent.magnetic.z;
    
    // LECTURA BAROMETRO
    altitudActual = calcularAltitud();
    float presionActual = bme.readPressure();

    // FORMATEO DE DATOS
    char buffer[200];
    snprintf(buffer, 200, "%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            tiempoActual,
            (int)estadoActual, // Guarda el número del estado actual
            altitudActual,
            presionActual,
            ax, ay, az,
            gx, gy, gz,
            mx, my, mz,
            heading
    );

    if (registro) {
      registro.println(buffer);
    }

  }

  void setup() {
    Serial.begin(115200);
    Wire.begin();
    SPI.begin();

    //Inicialización de LED y Buzzer
    pinMode(SIGN_LED, OUTPUT); //LED neopixel?

    pinMode(PIN_RECUPERACION1, OUTPUT);
    pinMode(PIN_RECUPERACION2, OUTPUT);
    digitalWrite(SIGN_LED, LOW); // asegurando que no esten activados
    digitalWrite(PIN_RECUPERACION1, HIGH);
    digitalWrite(PIN_RECUPERACION2, HIGH);

    // ESTADO 0 Inicialización de sensores

    mpuOK = inicializarMPU();
    magOK = inicializarHMC();
    bmeOK = inicializarBME();
    cardOK = inicializarSD();
    fileOK = nuevoArchivo();

    bool todoOK = mpuOK && magOK && bmeOK && cardOK && fileOK;

      // TRANSICIÓN ESTADO 0 -> ESTADO 1
      if (!todoOK){
        Serial.println("No se han inicializado todos los sensores! No se pasará a la siguiente fase.");
        
        for(int i = 0 ; i < 5 ; i++){ //Señal de ERROR

          digitalWrite(SIGN_LED, HIGH);
          delay(100);
          digitalWrite(SIGN_LED, LOW);
          delay(100);
        }
      }
      else{
        Serial.println("Todos los sistemas inicializados. Se pasará a Standby");
        digitalWrite(SIGN_LED, HIGH);
        delay(500);
        digitalWrite(SIGN_LED, LOW);
        estadoActual = ESTADO_1_STANDBY;
        }

    PresionInicial(); //Calibrando Presión Inicial

      if (!bmeOK) {
      Serial.println("Error al iniciar BME280");
      } else {
      altitudInicial = 44330.0 * (1.0 - pow(presionInicial / 101325.0, 1.0 / 5.255));
      Serial.print("Altitud inicial: ");
      Serial.println(altitudInicial);
      }  

  }


  void loop() {
    
    unsigned long tiempoActual = millis(); // Quiero limitar la tasa de muestreo para ciertos Estados:
                                          // En el Standby: limitar a 10Hz para ahorrar batería. Pero
                                          // Esto quedaría en WIP.

    gestionEstados(tiempoActual);

    //AQUÍ PONER LAS FUNCIONES QUE ESCRIBAN LOS DATOS EN LA SD Y QUE CALCULEN LA ALTITUD Y VELOCIDAD.
    registrarDatos(tiempoActual);
  }
