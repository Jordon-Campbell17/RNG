#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// ==== PIN DEFINITIONS ====
// 7-seg pins
#define a  15
#define b  32
#define c  33
#define d  25
#define e  26
#define f  27
#define g  14
#define dp 12  // decimal point (optional)

// LEDs and button
#define LED_A 16
#define LED_B 17
#define BTN_A 4

// ==== NETWORK SETTINGS ====
const char* ssid = "MonaConnect";
const char* password = "";

const char* mqtt_server = "www.yanacreations.com";   // MQTT broker IP
const int mqtt_port = 1883;

const char* pub_topic = "rng/data";
const char* sub_topic = "rng/control";

String student_id = "620162191";   

// ==== GLOBALS ====
WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 1000);

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Digit segments: a-g for digits 0-9
const byte digits[10][7] = {
  {1,1,1,1,1,1,0}, // 0
  {0,1,1,0,0,0,0}, // 1
  {1,1,0,1,1,0,1}, // 2
  {1,1,1,1,0,0,1}, // 3
  {0,1,1,0,0,1,1}, // 4
  {1,0,1,1,0,1,1}, // 5
  {1,0,1,1,1,1,1}, // 6
  {1,1,1,0,0,0,0}, // 7
  {1,1,1,1,1,1,1}, // 8
  {1,1,1,1,0,1,1}  // 9
};

// ==== UTILITY FUNCTIONS ====

// Display a digit on the 7-seg display
void Display(unsigned char number) {
  digitalWrite(a, digits[number][0]);
  digitalWrite(b, digits[number][1]);
  digitalWrite(c, digits[number][2]);
  digitalWrite(d, digits[number][3]);
  digitalWrite(e, digits[number][4]);
  digitalWrite(f, digits[number][5]);
  digitalWrite(g, digits[number][6]);
}

// Get LED state (0=LOW, 1=HIGH)
int8_t getLEDStatus(int8_t LED) {
  return digitalRead(LED);
}

// Set LED state
void setLEDState(int8_t LED, int8_t state) {
  digitalWrite(LED, state);
}

// Toggle LED state
void toggleLED(int8_t LED) {
  int currentState = digitalRead(LED);
  digitalWrite(LED, !currentState);
}

// Generate, Display, Publish random number
void GDP() {
  int number = random(0, 10);
  Serial.print("Generated number: ");
  Serial.println(number);

  Display(number);

  StaticJsonDocument<200> doc;
  doc["id"] = student_id;
  doc["timestamp"] = timeClient.getEpochTime();
  doc["number"] = number;
  doc["ledA"] = getLEDStatus(LED_A);
  doc["ledB"] = getLEDStatus(LED_B);

  char buffer[256];
  serializeJson(doc, buffer);

  client.publish(pub_topic, buffer);

  Serial.print("Published JSON: ");
  Serial.println(buffer);
}

// ==== MQTT CALLBACK ====

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\nMessage received on topic: %s\n", topic);

  char *received = new char[length + 1]{0};
  for (unsigned int i = 0; i < length; i++) {
    received[i] = (char)payload[i];
  }
  Serial.printf("Payload: %s\n", received);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, received);
  delete[] received;

  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  String type = doc["type"] | "";
  String device = doc["device"] | "";

  if (type == "toggle") {
    if (device == "LED A") {
      toggleLED(LED_A);
      Serial.println("Toggled LED_A");
    } else if (device == "LED B") {
      toggleLED(LED_B);
      Serial.println("Toggled LED_B");
    }
  }
}

// ==== WIFI + MQTT SETUP ====

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries++;

    if (retries > 40) {   // ~20 seconds timeout
      Serial.println("\nWiFi FAILED. Restarting...");
      ESP.restart();
    }
  }

  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(sub_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

// ==== SETUP ====

void setup() {
  Serial.begin(115200);
  

  // Setup pins
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(d, OUTPUT);
  pinMode(e, OUTPUT);
  pinMode(f, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(dp, OUTPUT);

  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BTN_A, INPUT_PULLUP);

  // Init outputs to LOW
  Display(8);  // Show '8' at startup
  setLEDState(LED_A, LOW);
  setLEDState(LED_B, LOW);

  // Seed RNG
  randomSeed(analogRead(0));

  // Connect WiFi and MQTT
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  timeClient.begin();
}

// ==== MAIN LOOP ====

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  timeClient.update();

  static bool buttonPressed = false;

  int reading = digitalRead(BTN_A);
  Serial.print("Button state: ");
  Serial.println(reading);

  if (reading == LOW && !buttonPressed && (millis() - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = millis();

    Serial.println("Button pressed! Running GDP...");
    GDP();
  }
  if (reading == HIGH) {
    buttonPressed = false;
  }

  delay(100); // Slow down Serial prints
}


