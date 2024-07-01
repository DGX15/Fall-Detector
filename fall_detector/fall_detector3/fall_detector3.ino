#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include <TinyGPS++.h>  // Librería para el manejo del GPS

MPU6050 mpu;
TinyGPSPlus gps;  // Objeto para manejar los datos del GPS

// Definición de pines
const int buzzerPin = 23;
const int buttonPin = 19;

// Variables para detectar caída y manejar el buzzer
const float threshold = 1.5;  // Umbral de aceleración para detectar caída
unsigned long fallDetectedTime = 0;  // Tiempo en que se detectó la caída
bool alertActive = false;  // Indica si la alerta de caída está activa
bool alertStarted = false;  // Indica si ya han pasado los 5 segundos y se inició la alerta
bool coordPrinted = false;  // Indica si las coordenadas del GPS ya se han impreso

// Configuración de WiFi
const char* ssid = "Red Genesys";  // Nombre de tu red WiFi
const char* password = "Genesys01";  // Contraseña de tu red WiFi

// Configuración de la zona horaria para Yucatán (UTC-6, sin horario de verano)
const char* timezone = "CST6";  // Yucatán no observa horario de verano

// Variables para el manejo de la hora y fecha
struct tm timeinfo;
bool timeUpdated = false;

// Variables para almacenar las coordenadas
float latitude = 0.0;
float longitude = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Botón conectado a GND
  
  // Conectar a WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando el dispositivo a la red WiFi...");
  }
  
  // Configurar la zona horaria y sincronización de tiempo con otro servidor NTP
  configTime(0, 0, "time.nist.gov", "time.google.com");
  setenv("TZ", timezone, 1);  // Establecer la zona horaria
  tzset();  // Actualizar la configuración de la zona horaria
  
  // Inicializar Serial para el GPS (Serial2 en ESP32)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2 y TX2, ajusta los pines según tu ESP32
  
  Serial.println("Dispositivo conectado.");
}

void loop() {
  // Lectura del botón
  int buttonState = digitalRead(buttonPin);
  
  if (buttonState == LOW) {
    // Si se presiona el botón, cancelar la alerta si está activa
    if (alertActive) {
      alertActive = false;
      alertStarted = false;
      digitalWrite(buzzerPin, LOW);  // Apagar el buzzer
      Serial.println("Alerta cancelada por el usuario. Todo está bien.");
    }
  }
  
  // Obtener y mostrar la fecha y hora actuales si han pasado al menos 10 segundos desde la última actualización
  if (!timeUpdated || millis() - fallDetectedTime > 10000) {
    getTime();
    fallDetectedTime = millis();  // Actualizar el tiempo de detección de caída
  }
  
  // Lectura del MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calcular la magnitud de la aceleración
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az) / 16384.0; // 16384 LSB/g (sensor range +/-2g)
  
  // Detectar caída (umbral de aceleración superado y no hay alerta activa)
  if (accelMagnitude > threshold && !alertActive) {
    // Activar alerta de caída y comenzar el conteo de los 5 segundos
    alertActive = true;
    fallDetectedTime = millis();  // Tiempo en que se detectó la caída
    
    // Obtener las coordenadas GPS si están disponibles
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (gps.location.isValid()) {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          coordPrinted = true;  // Marcar que las coordenadas han sido obtenidas
          break; // Salir del bucle una vez que se obtengan las coordenadas
        }
      }
    }
  }
  
  // Comenzar la alerta después de 5 segundos de detectar la caída
  if (alertActive && !alertStarted && millis() - fallDetectedTime >= 5000) {
    alertStarted = true;
    
    // Iniciar la alerta
    Serial.print("¡Caída detectada! Fecha y hora: ");
    printCurrentDateTime();
    Serial.print(" Coordenadas de la caída: ");
    printCoordinatesDecimal();
    Serial.println();
    digitalWrite(buzzerPin, HIGH);  // Encender el buzzer
    
    // Abrir Google Maps con las coordenadas
    openGoogleMaps(latitude, longitude);
  }
  
  // Manejo de la alerta (si está activa)
  if (alertActive && alertStarted) {
    // Emitir sonido del buzzer cada segundo
    unsigned long currentTime = millis();
    static unsigned long lastAlertTime = currentTime;
    
    if (currentTime - lastAlertTime >= 1000) {
      lastAlertTime = currentTime;
      digitalWrite(buzzerPin, !digitalRead(buzzerPin));  // Invertir el estado del buzzer (generar sonido)
    }
  } else {
    // Si no hay alerta activa, asegurarse de que el buzzer esté apagado
    digitalWrite(buzzerPin, LOW);
  }
  
  delay(50); // Pequeña pausa para evitar lecturas demasiado rápidas
}

// Función para obtener la fecha y hora actual
void getTime() {
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Error al obtener la hora del servidor NTP");
    return;
  }
  timeUpdated = true;
}

// Función para imprimir la fecha y hora actual en el monitor serial
void printCurrentDateTime() {
  char output[30];
  strftime(output, 30, "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.print(output);
}

// Función para imprimir las coordenadas en formato decimal
void printCoordinatesDecimal() {
  Serial.print(latitude, 6);
  Serial.print(", ");
  Serial.print(longitude, 6);
}

// Función para abrir Google Maps con las coordenadas especificadas
void openGoogleMaps(float lat, float lng) {
  String mapLink = "https://www.google.com/maps/search/?api=1&query=" + String(lat, 6) + "," + String(lng, 6);
  Serial.print("Presiona el enlace para visualizar en el mapa: ");
  Serial.println(mapLink);
  delay(100);
  Serial.println("Recuerda tener los números de emergencia a la mano para cualquier situación.");
}
