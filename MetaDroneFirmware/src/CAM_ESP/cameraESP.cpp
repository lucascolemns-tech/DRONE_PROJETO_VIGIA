#include "cameraESP.h"

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_http_server.h>

// AI-Thinker ESP32-CAM camera connector pinout.
static constexpr int CAMERA_PIN_PWDN = 32;
static constexpr int CAMERA_PIN_RESET = -1;
static constexpr int CAMERA_PIN_XCLK = 0;
static constexpr int CAMERA_PIN_SIOD = 26;
static constexpr int CAMERA_PIN_SIOC = 27;
static constexpr int CAMERA_PIN_Y9 = 35;
static constexpr int CAMERA_PIN_Y8 = 34;
static constexpr int CAMERA_PIN_Y7 = 39;
static constexpr int CAMERA_PIN_Y6 = 36;
static constexpr int CAMERA_PIN_Y5 = 21;
static constexpr int CAMERA_PIN_Y4 = 19;
static constexpr int CAMERA_PIN_Y3 = 18;
static constexpr int CAMERA_PIN_Y2 = 5;
static constexpr int CAMERA_PIN_VSYNC = 25;
static constexpr int CAMERA_PIN_HREF = 23;
static constexpr int CAMERA_PIN_PCLK = 22;

// AUX_ESP reserves timer 0 for ESC PWM; keep camera XCLK on timer 3/channel 7.
static constexpr ledc_timer_t CAMERA_XCLK_TIMER = LEDC_TIMER_3;
static constexpr ledc_channel_t CAMERA_XCLK_CHANNEL = LEDC_CHANNEL_7;

static httpd_handle_t streamServer = nullptr;

static const char STREAM_CONTENT_TYPE[] = "multipart/x-mixed-replace;boundary=frame";
static const char STREAM_BOUNDARY[] = "\r\n--frame\r\n";
static const char STREAM_PART[] = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t *request)
{
    esp_err_t result = httpd_resp_set_type(request, STREAM_CONTENT_TYPE);
    if (result != ESP_OK)
        return result;

    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");

    char partHeader[64];
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == nullptr)
            return ESP_FAIL;

        result = httpd_resp_send_chunk(request, STREAM_BOUNDARY, sizeof(STREAM_BOUNDARY) - 1);
        if (result == ESP_OK)
        {
            const int headerLength = snprintf(partHeader, sizeof(partHeader), STREAM_PART,
                                              static_cast<unsigned>(frame->len));
            if (headerLength < 0 || headerLength >= static_cast<int>(sizeof(partHeader)))
                result = ESP_FAIL;
            else
                result = httpd_resp_send_chunk(request, partHeader, headerLength);
        }
        if (result == ESP_OK)
            result = httpd_resp_send_chunk(request, reinterpret_cast<const char *>(frame->buf),
                                           frame->len);

        esp_camera_fb_return(frame);
        if (result != ESP_OK)
            break;

        delay(1);
    }

    return result;
}

bool cameraESP_iniciar()
{
    camera_config_t config = {};
    config.ledc_channel = CAMERA_XCLK_CHANNEL;
    config.ledc_timer = CAMERA_XCLK_TIMER;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_sccb_sda = CAMERA_PIN_SIOD;
    config.pin_sccb_scl = CAMERA_PIN_SIOC;
    config.pin_d7 = CAMERA_PIN_Y9;
    config.pin_d6 = CAMERA_PIN_Y8;
    config.pin_d5 = CAMERA_PIN_Y7;
    config.pin_d4 = CAMERA_PIN_Y6;
    config.pin_d3 = CAMERA_PIN_Y5;
    config.pin_d2 = CAMERA_PIN_Y4;
    config.pin_d1 = CAMERA_PIN_Y3;
    config.pin_d0 = CAMERA_PIN_Y2;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound())
    {
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    if (esp_camera_init(&config) != ESP_OK)
        return false;

    httpd_config_t serverConfig = HTTPD_DEFAULT_CONFIG();
    serverConfig.server_port = 81;
    serverConfig.ctrl_port = 32769;

    if (httpd_start(&streamServer, &serverConfig) != ESP_OK)
    {
        esp_camera_deinit();
        streamServer = nullptr;
        return false;
    }

    httpd_uri_t streamUri = {};
    streamUri.uri = "/stream";
    streamUri.method = HTTP_GET;
    streamUri.handler = streamHandler;
    streamUri.user_ctx = nullptr;

    if (httpd_register_uri_handler(streamServer, &streamUri) != ESP_OK)
    {
        httpd_stop(streamServer);
        streamServer = nullptr;
        esp_camera_deinit();
        return false;
    }

    return true;
}
