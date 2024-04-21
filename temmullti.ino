#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Galaxy";
const char* password = "1234567890";
const char* mqtt_server = "srv502440.hstgr.cloud";
const char* mqtt_user = "esp32";
const char* mqtt_password = "1234";

WiFiClient espClient;
PubSubClient client(espClient);

const int lm35Pin = 34;
float temperaturaCelsius;
float temperaturaFahrenheit;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado al servidor MQTT");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int lectura = analogRead(lm35Pin);
  temperaturaCelsius = (lectura * 3.3) / 4095 * 100;
  temperaturaFahrenheit = (temperaturaCelsius * 9 / 5) + 32;

  // Crear un objeto JSON
  StaticJsonDocument<100> jsonDoc;
  jsonDoc["temperaturaCelsius"] = temperaturaCelsius;


  // Convertir el objeto JSON a una cadena
  char buffer[100];
  serializeJson(jsonDoc, buffer);

  // Publicar la cadena JSON en el tópico MQTT
  client.publish("/MedicHealth/sensores/temperatura", buffer);
  
  delay(1000);
}

