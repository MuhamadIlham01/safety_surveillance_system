#include <WiFi.h>
#include <PubSubClient.h>
#include <max6675.h>

// WiFi Configuration
const char* ssid = "ilham";
const char* password = "kontolodon";

// MQTT Configuration
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

// Pin Definitions
#define MAX6675_SO_PIN 12  
#define MAX6675_CS_PIN 14  
#define MAX6675_SCK_PIN 27 
#define MQ2_PIN 34
#define TRIGGER_PIN 5
#define ECHO_PIN 18
#define PIR_PIN 19
#define BUZZER_PIN 23
#define LED_PIN 22

// Thresholds
#define TEMP_THRESHOLD 40.0
#define GAS_THRESHOLD 1500
#define DISTANCE_THRESHOLD 50.0

// Alert Timing
#define BUZZER_ON_TIME 500    // milliseconds
#define BUZZER_OFF_TIME 500   // milliseconds
#define LED_BLINK_INTERVAL 200 // milliseconds
#define MOTION_DEBOUNCE 200   // milliseconds

WiFiClient espClient;
PubSubClient client(espClient);
MAX6675 thermocouple(MAX6675_SCK_PIN, MAX6675_CS_PIN, MAX6675_SO_PIN);

// Sensor Variables
float temp = 0;
int gasValue = 0;
float distance = 0;
bool motionDetected = false;

// Alert State Variables
bool alertActive = false;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;
unsigned long lastLedToggle = 0;
bool ledState = false;

// Motion Sensor Debounce
unsigned long lastMotionChange = 0;
bool lastPIRState = false;

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe to topics if needed
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

bool readMotion() {
  bool currentPIR = digitalRead(PIR_PIN);
  unsigned long now = millis();
  
  if (currentPIR != lastPIRState) {
    lastMotionChange = now;
  }
  
  if ((now - lastMotionChange) > MOTION_DEBOUNCE) {
    if (currentPIR != motionDetected) {
      motionDetected = currentPIR;
      if (motionDetected) {
        Serial.println("Motion detected!");
      } else {
        Serial.println("Motion ended");
      }
    }
  }
  
  lastPIRState = currentPIR;
  return motionDetected;
}

float readDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // cm
}

void handleAlerts() {
  bool tempAlert = temp > TEMP_THRESHOLD;
  bool gasAlert = gasValue > GAS_THRESHOLD;
  bool distanceAlert = distance < DISTANCE_THRESHOLD;
  bool motionAlert = readMotion();
  
  bool newAlertStatus = tempAlert || gasAlert || distanceAlert || motionAlert;

  // Debug output
  Serial.print("Alerts - Temp:");
  Serial.print(tempAlert);
  Serial.print(" Gas:");
  Serial.print(gasAlert);
  Serial.print(" Dist:");
  Serial.print(distanceAlert);
  Serial.print(" Motion:");
  Serial.println(motionAlert);

  // Only update if status changed
  if (newAlertStatus != alertActive) {
    alertActive = newAlertStatus;
    client.publish("surveillance/alert", alertActive ? "WARNING" : "SAFE");
    Serial.println(alertActive ? "ALERT ACTIVATED" : "ALERT CLEARED");
  }

  // Handle buzzer and LED
  unsigned long currentMillis = millis();
  
  if (alertActive) {
    // Buzzer pattern
    if (currentMillis - lastBuzzerToggle >= (buzzerState ? BUZZER_ON_TIME : BUZZER_OFF_TIME)) {
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      lastBuzzerToggle = currentMillis;
    }
    
    // LED pattern
    if (currentMillis - lastLedToggle >= LED_BLINK_INTERVAL) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLedToggle = currentMillis;
    }
  } else {
    // Ensure both are off
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    buzzerState = false;
    ledState = false;
  }
}

void publishSensorData() {
  char tempStr[8], gasStr[8], distStr[8], motionStr[8];
  dtostrf(temp, 6, 2, tempStr);
  dtostrf(distance, 6, 2, distStr);
  sprintf(gasStr, "%d", gasValue);
  sprintf(motionStr, "%d", motionDetected);

  client.publish("surveillance/temperature", tempStr);
  client.publish("surveillance/gas", gasStr);
  client.publish("surveillance/distance", distStr);
  client.publish("surveillance/motion", motionStr);

  Serial.print("Temperature: "); Serial.print(temp);
  Serial.print("°C, Gas: "); Serial.print(gasValue);
  Serial.print(", Distance: "); Serial.print(distance);
  Serial.print("cm, Motion: ");
  Serial.println(motionDetected ? "Detected" : "None");
}

void setup() {
  Serial.begin(115200);
  delay(500); // Allow MAX6675 to stabilize

  pinMode(MQ2_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initialize outputs to LOW
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Initial sensor check
  Serial.println("Initial sensor readings:");
  Serial.print("PIR: "); Serial.println(digitalRead(PIR_PIN));
  Serial.print("Gas: "); Serial.println(analogRead(MQ2_PIN));
  temp = thermocouple.readCelsius() - 12.0;
  Serial.print("Temp: "); Serial.println(temp);
  Serial.print("Distance: "); Serial.println(readDistance());

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastUpdateTime = 0;
  if (millis() - lastUpdateTime >= 500) {
    lastUpdateTime = millis();

    // Read sensors
    temp = thermocouple.readCelsius() - 12.0; // Calibration offset
    gasValue = analogRead(MQ2_PIN);
    distance = readDistance();
    readMotion(); // Updates motionDetected

    // Validate temperature reading
    if (isnan(temp)) {
      Serial.println("Thermocouple error!");
      temp = -1;
    }

    publishSensorData();
    handleAlerts();
  }
}