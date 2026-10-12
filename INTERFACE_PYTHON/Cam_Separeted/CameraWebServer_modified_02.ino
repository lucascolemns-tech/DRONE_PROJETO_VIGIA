
#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>

// Endereço do computador que executa o Python
IPAddress serverIP(192, 168, 1, 57);
const uint16_t serverPort = 1224;

WiFiClient tcpClient;

// Envia todos os bytes, mesmo que sejam necessários vários envios
bool writeAll(WiFiClient &client, const uint8_t *data, size_t len) {
  size_t sent = 0;

  while (sent < len) {
    if (!client.connected()) {
      return false;
    }

    size_t n = client.write(data + sent, len - sent);

    if (n == 0) {
      return false;
    }

    sent += n;
  }

  return true;
}

// Modelo da câmera
#include "board_config_02.h"

const char *ssid = "CTS- RODRIGUES";
const char *password = "148281re";

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 12;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
      config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Inicializa a câmera
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

#if defined(CAMERA_MODEL_M5STACK_WIDE) || \
    defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  // Conecta ao Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Conectando ao Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi conectado.");
  Serial.print("IP da ESP32-CAM: ");
  Serial.println(WiFi.localIP());

  tcpClient.setNoDelay(true);
}

void loop() {
  // Verifica a conexão Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    delay(500);
    return;
  }

  // Conecta ou reconecta ao servidor Python
  if (!tcpClient.connected()) {
    tcpClient.stop();

    Serial.println("Conectando ao servidor Python...");

    if (!tcpClient.connect(serverIP, serverPort)) {
      Serial.println("Falha na conexão TCP. Tentando novamente...");
      delay(1000);
      return;
    }

    tcpClient.setNoDelay(true);
    Serial.println("Conexão TCP estabelecida.");
  }

  // Captura um frame JPEG
  camera_fb_t *fb = esp_camera_fb_get();

  if (fb == nullptr) {
    Serial.println("Falha na captura da câmera.");
    delay(50);
    return;
  }

  // Cabeçalho de 4 bytes com o tamanho da imagem em big-endian
  uint32_t len = fb->len;

  uint8_t header[4] = {
    (uint8_t)(len >> 24),
    (uint8_t)(len >> 16),
    (uint8_t)(len >> 8),
    (uint8_t)len
  };

  // Envia primeiro o cabeçalho e depois a imagem
  bool ok = writeAll(tcpClient, header, sizeof(header));

  if (ok) {
    ok = writeAll(tcpClient, fb->buf, fb->len);
  }

  // Libera o buffer da câmera
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("Falha na transmissão do frame.");
    tcpClient.stop();
    delay(100);
  }
}
