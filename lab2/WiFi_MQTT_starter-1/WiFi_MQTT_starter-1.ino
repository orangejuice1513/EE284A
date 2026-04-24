#include <WiFi.h>
#include <PubSubClient.h>

const char *ssid = "EE-IoT";            // Hidden SSID in our labs
const char *password = "350Serra";    // Change this to the secret WiFi password
IPAddress serverIP(10, 136, 92, 10);               // the class MQTT server

// MQTT client using the WiFi TCP client
WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMqttPublish = 0;
const unsigned long mqttPublishInterval = 10000; // 10 seconds

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    String message;
    for (int i=0;i<length;i++) {
        Serial.print((char)payload[i]);
        message += (char)payload[i];
    }
    Serial.println();

    // Simple topic handling: allow remote control of built-in LED
    if ((strcmp(topic, "inTopic") == 0) || (strcmp(topic, "device/led") == 0)) {
        message.trim();
        if (message == "ON" || message == "1" || message == "H") {
            digitalWrite(LED_BUILTIN, HIGH);
            Serial.println("LED turned ON via MQTT");
        } else if (message == "OFF" || message == "0" || message == "L") {
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("LED turned OFF via MQTT");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("MQTT_WIFI_PM Example Starting...");

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println("******************************************************");
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Configure built-in LED
    pinMode(LED_BUILTIN, OUTPUT);

    // Configure MQTT client
    mqttClient.setServer(serverIP, 1883);
    mqttClient.setCallback(callback);

}

void mqttReconnect() {
    // Loop until we're reconnected.  WiFi web page won't work until MQTT is connected.
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqttClient.connect("esp32Clien_j")) {
            Serial.println("connected");
            mqttClient.publish("test/topic/", "Julia");
            mqttClient.subscribe("test/topic/");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}


void loop()
{
    // Ensure MQTT connection and service incoming messages
    if (!mqttClient.connected()) {
        mqttReconnect();
    }
    mqttClient.loop();

    // Periodic publish (heartbeat)
    if (millis() - lastMqttPublish >= mqttPublishInterval) {
        if (mqttClient.connected()) {
            mqttClient.publish("outTopic", "heartbeat");
        }
        lastMqttPublish = millis();
    }

    Serial.print(".");
    delay(1000);
}
