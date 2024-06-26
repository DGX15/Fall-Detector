#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>

MPU6050 mpu;

// Definición de pines
const int buzzerPin = 23;
const int buttonPin = 19;

// Variables para detectar caída y manejar el buzzer
const float threshold = 1.5;  // Umbral de aceleración para detectar caída
unsigned long fallDetectedTime = 0;  // Tiempo en que se detectó la caída
bool alertActive = false;  // Indica si la alerta de caída está activa
bool alertStarted = false;  // Indica si ya han pasado los 5 segundos y se inició la alerta

// Configuración de WiFi
const char* ssid = "Red Genesys";
const char* password = "Genesys01";

// Configuración de la zona horaria para Yucatán (UTC-6, sin horario de verano)
const char* timezone = "CST6";  // Yucatán no observa horario de verano

// Variables para el manejo de la hora y fecha
struct tm timeinfo;
bool timeUpdated = false;

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
    Serial.println("Conectando a WiFi...");
  }
  
  // Configurar la zona horaria y sincronización de tiempo con otro servidor NTP
  configTime(0, 0, "time.nist.gov", "time.google.com");
  setenv("TZ", timezone, 1);  // Establecer la zona horaria
  tzset();  // Actualizar la configuración de la zona horaria
  
  Serial.println("MPU6050 inicializado correctamente");
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
  
  }
  
  // Comenzar la alerta después de 5 segundos de detectar la caída
  if (alertActive && !alertStarted && millis() - fallDetectedTime >= 5000) {
    alertStarted = true;
    
    // Iniciar la alerta
    Serial.print("¡Caída detectada! Fecha y hora: ");
    printCurrentDateTime();
    Serial.println();
    digitalWrite(buzzerPin, HIGH);  // Encender el buzzer
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
