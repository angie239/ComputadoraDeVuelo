/*
 * Servidor Web en ESP32 para listar y descargar archivos de una Tarjeta SD
 * usando pines SPI personalizados.
 * * ATENCIÓN:
 * Pines GPIO 34, 35, 36, 39 son SOLO ENTRADA en el ESP32 estándar.
 * Este código ASUME que usas un ESP32-S2, S3, o un ESP32 con capacidad de
 * remapear estos pines (lo cual NO es el caso del WROOM-32 estándar).
 * * Conexiones SPI personalizadas:
 * SCK:  Pin 36
 * MISO: Pin 37
 * MOSI: Pin 35
 * CS:   Pin 10
 */

#include <WiFi.h>
#include <SD.h>      // Librería para la tarjeta SD
#include <SPI.h>     // Librería para la comunicación SPI
#include <WebServer.h> // Librería para el servidor web

// --- Configuración de Red ---
const char* ssid = "A55 de Zahul";
const char* password = "ingeniebros";

// --- Configuración de Pines SPI Personalizados ---
#define SCK_PIN  36
#define MISO_PIN 37
#define MOSI_PIN 35
#define CS_PIN   10

// --- Variables Globales ---
WebServer server(80); // Inicia el servidor en el puerto 80 (HTTP)

// --- Prototipos de Funciones (Handlers) ---
void handleRoot();
void handleDownload();
void handleNotFound();

void setup() {
  Serial.begin(115200);
  Serial.println("\nIniciando servidor web de archivos...");

  // 1. Inicializar el bus SPI con los pines personalizados
  // Esta línea es CRUCIAL. Le dice al ESP32 que use tus pines
  // en lugar de los pines SPI por defecto.
  Serial.println("Configurando SPI con pines personalizados...");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // 2. Inicializar la tarjeta SD
  Serial.print("Inicializando tarjeta SD (CS Pin: ");
  Serial.print(CS_PIN);
  Serial.println(")...");
  
  if (!SD.begin(CS_PIN)) {
    Serial.println("Error al inicializar la tarjeta SD");
    Serial.println("Verifica las conexiones, el formato (FAT32) y los pines (especialmente 35 y 36).");
    while (1); // Detiene la ejecución
  }
  Serial.println("Tarjeta SD inicializada correctamente.");

  // 3. Conectarse a la Red Wi-Fi
  Serial.print("Conectando a la red: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifi_retries++;
    if (wifi_retries > 20) {
      Serial.println("\nNo se pudo conectar. Reiniciando...");
      ESP.restart();
    }
  }
  
  Serial.println("\n¡Conectado a la red Wi-Fi!");
  Serial.print("Servidor corriendo en IP: ");
  Serial.println(WiFi.localIP()); // Imprime la IP

  // 4. Configurar las rutas del Servidor Web
  server.on("/", handleRoot);          // Ruta raíz: Muestra lista de archivos
  server.on("/download", handleDownload); // Ruta de descarga
  server.onNotFound(handleNotFound);  // Ruta no encontrada (Error 404)

  // 5. Iniciar el servidor
  server.begin();
  Serial.println("Servidor HTTP iniciado.");
}

void loop() {
  // Maneja las peticiones de los clientes
  server.handleClient();
}

// --- Implementación de Funciones "Handler" ---

/**
 * @brief Maneja la petición a la raíz "/".
 * Genera un HTML que lista los archivos de la SD.
 */
void handleRoot() {
  String html = "<html><head><title>Archivos ESP32 SD</title>";
  html += "<style>body { font-family: sans-serif; } a { text-decoration: none; color: #0078d4; }";
  html += "a:hover { text-decoration: underline; }</style>";
  html += "</head><body>";
  html += "<h1>Archivos en la Tarjeta SD</h1><ul>";

  File root = SD.open("/");
  if (!root) {
    html += "<li>Error al abrir el directorio raíz.</li>";
  } else {
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) { // Si no es un directorio
        html += "<li><a href=\"/download?file=";
        html += file.name(); // file.name() incluye el "/" inicial
        html += "\">";
        html += file.name();
        html += "</a> (";
        html += String(file.size()); // Muestra el tamaño
        html += " bytes)</li>";
      }
      file.close();
      file = root.openNextFile(); // Pasa al siguiente
    }
    root.close();
  }
  
  html += "</ul></body></html>";
  server.send(200, "text/html", html); // Envía la página web
}

/**
 * @brief Maneja la petición "/download".
 * Requiere un argumento 'file' (ej: /download?file=/mi_log.txt)
 * Envía el archivo al cliente para su descarga.
 */
void handleDownload() {
  if (!server.hasArg("file")) { // Si no se pasó el argumento 'file'
    server.send(400, "text/plain", "Error: Falta el argumento 'file'.");
    return;
  }

  String filename = server.arg("file"); // Obtiene el nombre del archivo
  
  if (!filename.startsWith("/")) {
    filename = "/" + filename; // Asegura que tenga el "/" al inicio
  }

  if (!SD.exists(filename)) { // Verifica si el archivo existe
    server.send(404, "text/plain", "Error: Archivo no encontrado.");
    return;
  }
  
  File file = SD.open(filename, FILE_READ); // Abre el archivo en modo lectura
  if (!file) {
    server.send(500, "text/plain", "Error al abrir el archivo para lectura.");
    return;
  }

  // Prepara las cabeceras para forzar la descarga en el navegador
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename.substring(1));
  
  // Envía el archivo. streamFile maneja todo por nosotros
  server.streamFile(file, "application/octet-stream"); // Tipo genérico
  
  file.close(); // Cierra el archivo
}

/**
 * @brief Maneja cualquier otra petición (Error 404).
 */
void handleNotFound() {
  server.send(404, "text/plain", "404: Ruta no encontrada.");
}