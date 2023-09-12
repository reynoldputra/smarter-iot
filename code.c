#include <WiFi.h>
#include <PubSubClient.h>
#include <FastLED.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"      // SD Card ESP32
#include "SD_MMC.h"  // SD Card ESP32
#include "driver/rtc_io.h"
#include <EEPROM.h>  // read and write from flash memory

#define EEPROM_SIZE 1
#define PIN_LED 12
#define LED_USED 18
#define LED_TOTAL 143
#define LED_GAP 8
#define PIN_RELAY 13

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
//========================================

// LED Flash PIN (GPIO 4)
#define FLASH_LED_PIN 4
#define LED_TTL 5000
#define FASTLED_FORCE_SOFTWARE_SPI

// Define the array of leds
CRGB leds[LED_TOTAL];
int ledOnMillis = 0;

// WiFi
const char *ssid = "";            // Enter your Wi-Fi name
const char *password = "";  // Enter Wi-Fi password
const char *lineNotifyToken = "";

// MQTT Broker
const char *mqtt_broker = "34.101.181.147";
const char *topic = "trigger";
const char *mqtt_username = "smarter";
const char *mqtt_password = "smarter";
const int mqtt_port = 1883;

// ESPCAM
// String endpoint2 = "http://192.168.137.253:8000/upload-image";
String endpoint2 = "http://34.101.45.178:5000/predict-binary";
bool LED_Flash_ON = false;
unsigned long previousMillis = 0;
const int Interval = 10000;  //--> Photo capture every 20 seconds.
camera_config_t config;
int pictureNumber = 0;

//WIFI and MQTT
WiFiClient espClient;
PubSubClient client(espClient);
HTTPClient http;

void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Set software serial baud to 115200;
  Serial.begin(115200);
  pinMode(FLASH_LED_PIN, OUTPUT);
  WiFi.mode(WIFI_STA);

  // Connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the Wi-Fi network");
  String lineMessage = "Wifi is connected";
  sendLineNotify(lineMessage);

  //ESPCAM CONFIG
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = 10;  //--> 0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = 8;  //--> 0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera initz
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    sendLineNotify("camera init failed");
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_HVGA);
  // s->set_gain_ctrl(s, 1);                       // auto gain on
  // s->set_exposure_ctrl(s, 1);                   // auto exposure on
  // s->set_awb_gain(s, 1);                        // Auto White Balance enable (0 or 1)
  // s->set_brightness(s, 1);
  capturePhoto();

  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "mqttjs01_subs";
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Smarter MQTT has connected");
      sendLineNotify("Mqtt has connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      sendLineNotify("Mqtt fail to connect");
      delay(2000);
    }
  }
  client.subscribe(topic);

  // LED
  FastLED.addLeds<NEOPIXEL, PIN_LED>(leds, LED_TOTAL);

  //Relay
  pinMode(PIN_RELAY, OUTPUT);
}

void loop() {
  Serial.println("[APP] Free memory: " + String(esp_get_free_heap_size()) + " bytes");
  client.loop();
  unsigned long currentMillis = millis();

  // Serial.println(ledOnMillis - currentMillis);
  if (ledOnMillis > 0 && ledOnMillis < currentMillis) {
    ledOnMillis = 0;
    offMultipleLed();
    FastLED.show();
  } else {
    if (currentMillis - previousMillis >= Interval) {
      previousMillis = currentMillis;
      capturePhoto();
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  String lineMessage = "Message arrived in topic: smarter";
  Serial.println("Message received");
  sendLineNotify(lineMessage);
  unsigned long currentMillis = millis();


  // Convert the payload to a string
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  // Try to parse the payload as an integer
  int payloadInt = payloadStr.toInt();

  if (payloadInt != 0) {
    String payloadMessage = "Message : " + payloadStr;
    sendLineNotify(payloadMessage);
    if (ledOnMillis == 0) {
      // LED ON;
      // ledOnMillis = cu0rrentMillis + LED_TTL;
      ledOnMillis = currentMillis + payloadInt;
      onMultipleLed();
      FastLED.show();
      // Mistmaker ON
      MistMaker();
    }
  } else {
    Serial.println("Wrong payload");
    sendLineNotify(lineMessage);
  }
}

//MIST MAKER
void MistMaker() {
  sendLineNotify("Mist Maker on");
  digitalWrite(PIN_RELAY, HIGH);
  delay(1000);
  digitalWrite(PIN_RELAY, LOW);
  sendLineNotify("Mist Maker off");
}

//LED
void LED() {
  onMultipleLed();
  FastLED.show();
  // Now turn the LED off, then pause
  // offMultipleLed();
  // FastLED.show();
  // delay(5000);
}

void onMultipleLed() {
  for(int i = 0; i < LED_TOTAL; i += LED_GAP){
    leds[i] = CRGB::OrangeRed;
  }
}

void offMultipleLed() {
  for (int i = 0; i < LED_TOTAL; i += LED_GAP){
    leds[i] = CRGB::Black;
  }
}



//ESPCAM
void capturePhoto() {
   for (int i = 0; i <= 5; i++) {
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
     if(!fb) {
        Serial.println("Camera capture failed");
        Serial.println("Restarting the ESP32 CAM.");
        delay(1000);
        ESP.restart();
        return;
      } 
    esp_camera_fb_return(fb);
    delay(200);
  }
  camera_fb_t *fb = NULL;

  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    sendLineNotify("Camera capture failed");
    return;
  }

  sendLineNotify("Camera capture success");
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/espcam.jpg";

  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  } else {
    file.write(fb->buf, fb->len);  // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();

  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);

  HTTPClient http;

  Serial.print("[HTTP] begin...\n");
  // configure traged server and url

  http.begin(endpoint2);  //HTTP
  http.setTimeout(10000);

  Serial.print("[HTTP] POST...\n");
  // start connection and send HTTP header
  int httpCode = http.sendRequest("POST", fb->buf, fb->len);  // we simply put the whole image in the post body.

  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);

    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
      sendLineNotify(payload);
    }
  } else {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    sendLineNotify(http.errorToString(httpCode).c_str());
  }
  http.end();
  esp_camera_fb_return(fb);
}



//LINE
void sendLineNotify(String message) {
  http.begin("https://notify-api.line.me/api/notify");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Authorization", "Bearer " + String(lineNotifyToken));

  String postData = "message=" + message + "&notificationDisabled=true";

  int httpResponseCode = http.POST(postData);

  if (httpResponseCode > 0) {
    String response = http.getString();
  } else {
    Serial.print("Line Notify Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}
