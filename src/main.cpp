#include "WiFi.h"
// #include "ESPAsyncWebServer.h"
// #include "SPIFFS.h"
#include "DHT20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>

// DHT20 Sensor
DHT20 dht20;

constexpr char WIFI_SSID[] = "TingleJungle";
constexpr char WIFI_PASSWORD[] = "HexagonIQ";

// to understand how to obtain an access token
constexpr char TOKEN[] = "qLe5Tvx2IaxPaYDyEZDY";
// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port,
// whereas 8883 would be the default encrypted SSL MQTT port
#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

// Baud rate for the debugging serial connection.
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

uint32_t previousStateChange;
constexpr int16_t telemetrySendInterval = 5000U;
uint32_t previousDataSend;

// Declare pins here
const int LED_PIN = 48;    // Using built-in LED on pin 48
const int FAN_PIN = 8;

// LED and FAN state variables
volatile bool ledState = false;
volatile int fanLevel = 0;
volatile bool ledStateChanged = false;
volatile bool fanLevelChanged = false;
constexpr const char LED_STATE_ATTR[] = "led_state";
constexpr const char FAN_LEVEL_ATTR[] = "fan_level";

#if ENCRYPTED
// See https://comodosslstore.com/resources/what-is-a-root-ca-certificate-and-how-do-i-download-it/
// on how to get the root certificate of the server we want to communicate with,
// this is needed to establish a secure connection and changes depending on the website.
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";
#endif

constexpr const char RPC_LED_METHOD[] = "setStateLED";
constexpr const char RPC_FAN_METHOD[] = "setStateFAN";
// constexpr const char RPC_LED_KEY[] = "state";
// constexpr const char RPC_FAN_KEY[] = "level";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 2U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

// Initialize underlying client, used to establish a connection
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize used apis
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation *, 1U> apis = {
    &rpc};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// Statuses for subscribing to rpc
bool subscribed = false;

/// @brief Processes function for RPC call "led_control"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processLEDControl(const JsonVariantConst &data, JsonDocument &response)
{
  // Process data
  ledState = data;
  Serial.print("Received set led state RPC. New state: ");
  Serial.println(ledState);
    
  StaticJsonDocument<1> response_doc;
  // Returning current state as response
  response_doc["newState"] = (int)ledState;
  response.set(response_doc);

  ledStateChanged = true;
}

/// @brief Processes function for RPC call "fan_control"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processFanControl(const JsonVariantConst &data, JsonDocument &response)
{
  // Process data
  fanLevel = data;
  Serial.print("Received set fan level RPC. New state: ");
  Serial.println(fanLevel);

  StaticJsonDocument<1> response_doc;
  // Returning current state as response
  response_doc["newLevel"] = (int)fanLevel;
  response.set(response_doc);
}

// Task 1: Connect to WiFi
void wifiTask(void *pvParameters) {
  while (1) {
    // Check WiFi status
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Connecting to AP ...");

      // Attempt to connect to WiFi
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
      // Wait for connection with periodic status updates
      while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
      }

      Serial.print("\nConnected to: ");
      Serial.println(WiFi.localIP());

#if ENCRYPTED
      espClient.setCACert(ROOT_CERT);
#endif
    }

    // Periodically check WiFi status
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task 2: ThingsBoard connection and RPC subscription
void thingsboardTask(void *pvParameters) {
  while (1) {
    if (!tb.connected()) {
      Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
    }

    if (!subscribed) {
      Serial.println("Subscribing for RPC...");
      const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
        RPC_Callback{RPC_LED_METHOD, processLEDControl},
        RPC_Callback{RPC_FAN_METHOD, processFanControl}
      };
      if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }
      Serial.println("Subscribe done");
      subscribed = true;
    }

    tb.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task 3: DHT20 sensor reading and telemetry sending
void dht20Task(void *pvParameters) {
  char buffer[64]; // Buffer for formatted output
  while (1) {
    if (tb.connected()) {
      if (millis() - previousDataSend > telemetrySendInterval) {
        // Retry reading sensor up to 3 times
        float temperature = NAN;
        float humidity = NAN;
        for (int i = 0; i < 3 && (isnan(temperature) || isnan(humidity)); i++) {
          dht20.read();
          temperature = dht20.getTemperature();
          humidity = dht20.getHumidity();
          if (isnan(temperature) || isnan(humidity)) {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Brief delay before retry
          }
        }

        if (isnan(temperature) || isnan(humidity)) {
          Serial.println("Failed to read from DHT20 sensor!");
        } else {
          // Format the entire message into a buffer
          snprintf(buffer, sizeof(buffer), "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);
          Serial.println(buffer);
        }

        if (!isnan(temperature) && !isnan(humidity)) {
          tb.sendTelemetryData("temperature", temperature);
          tb.sendTelemetryData("humidity", humidity);
        }

        tb.sendAttributeData("rssi", WiFi.RSSI());
        tb.sendAttributeData("channel", WiFi.channel());
        tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());

        previousDataSend = millis();
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task 4: LED Control Task (monitors ThingsBoard commands)
void ledControlTask(void *pvParameters) {
  while (1) {
    if (ledStateChanged) {
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
      tb.sendAttributeData(LED_STATE_ATTR, ledState);
      ledStateChanged = false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Faster polling
  }
}

// Task 5: FAN Control Task (monitors ThingsBoard commands)
void fanControlTask(void *pvParameters) {
  while (1) {
    if (fanLevelChanged) {
      Serial.print("FAN level is set to: ");
      Serial.println(fanLevel);
      tb.sendAttributeData(FAN_LEVEL_ATTR, fanLevel);
      fanLevelChanged = false;
    }
    analogWrite(FAN_PIN, fanLevel);
    vTaskDelay(10 / portTICK_PERIOD_MS); // Faster polling
  }
}

void setup()
{
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(3000);

  // Initialize I2C foro DHT20
  Wire.begin(GPIO_NUM_11, GPIO_NUM_12);
  dht20.begin();

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  xTaskCreate(wifiTask, "WiFi Task", 4096, NULL, 2, NULL);
  xTaskCreate(thingsboardTask, "ThingsBoard Task", 8192, NULL, 1, NULL);
  xTaskCreate(dht20Task, "DHT20 Task", 16384, NULL, 0, NULL);
  xTaskCreate(ledControlTask, "LED Control Task", 32768, NULL, 0, NULL);
  xTaskCreate(fanControlTask, "FAN Control Task", 65536, NULL, 0, NULL);
}

void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Keep the main loop alive but idle
}
