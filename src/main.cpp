#include <Arduino.h>
#include <esp_camera.h>
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "soc/soc.h"           // Disable brownour problems
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include <WiFi.h>
#include <base64.h>

//LED
#define FLASH 4

//WiFi
const char *ssid = "narhoa"; 
const char *password = "nhathuduc123";
WiFiClient wifi_client;

//MQTT
const char *broker_addr = "192.168.1.210";
int broker_port = 1883;
const char *device_id = "esp32cam";
const char *up_topic = "/esp32cam/up";
const char *down_topic = "/esp32cam/down";
PubSubClient mqtt_client(wifi_client);

#define GET "GET"

//Camera pins
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void camera_init();
void wifi_connect();
void mqtt_connect();
void connect();
void message_received(char* topic, byte* payload, unsigned int length);
String serialize_json(camera_fb_t *fb);
void mqtt_public_frame(camera_fb_t *fb);

void setup() {
    Serial.begin(115200);
    pinMode(FLASH, OUTPUT);
    camera_init();
    connect();
}

void loop() {
    if (!mqtt_client.connected() || !wifi_client.connected())
    {
      connect();
    }
    mqtt_client.loop();
}

void camera_init()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    camera_config_t config;
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
    config.pixel_format = PIXFORMAT_JPEG; //Format of the pixel data: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG

    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.
    if(psramFound()){
        config.frame_size = FRAMESIZE_XGA; //FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
        config.jpeg_quality = 10; //Quality of JPEG output. 0-63 lower means higher quality
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    //frame settings
    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
}

void wifi_connect()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Wifi connect to: ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
}

void mqtt_connect()
{
  mqtt_client.setServer(broker_addr, broker_port);
  Serial.print("\nbroker connecting...");
  while (!mqtt_client.connect(device_id)) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nconnected!");
  
  mqtt_client.subscribe(down_topic);
  mqtt_client.setCallback(message_received);
}

void connect()
{
  wifi_connect();
  mqtt_connect();
}

String serialize_json(camera_fb_t *fb)
{   
    String encoded_frame;
    String base64_frame = base64::encode(fb->buf, fb->len);
    DynamicJsonDocument json_package(base64_frame.length() + 1000);
    json_package["encoded_frame"] = base64_frame;
    serializeJson(json_package, encoded_frame);
    return encoded_frame;
}

void mqtt_public_frame(camera_fb_t *fb)
{   
    String encoded_frame = serialize_json(fb);
    mqtt_client.beginPublish("/esp32cam/up", encoded_frame.length(), false);
    mqtt_client.print(encoded_frame);
    mqtt_client.endPublish();
}

void message_received(char* topic, byte* payload, unsigned int length)
{   
    char *decoded_payload = (char *)malloc(length + 1);
    memcpy(decoded_payload, payload, length);
    decoded_payload[length] = '\0'; //add \0 because payload dont have \0

    if (!strcmp(decoded_payload, GET))
    {   
        digitalWrite(FLASH, HIGH);
        camera_fb_t *fb = esp_camera_fb_get();
        if(fb)
        {
            Serial.print("taken successfully! len: ");
            Serial.println(fb->len);
            //mqtt publish
            mqtt_public_frame(fb);
        }
        esp_camera_fb_return(fb);
        digitalWrite(FLASH, LOW);
    }
    free(decoded_payload);
}