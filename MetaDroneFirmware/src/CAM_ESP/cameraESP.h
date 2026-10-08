#ifndef CAMERA_ESP_H
#define CAMERA_ESP_H

// Initializes the AI-Thinker ESP32-CAM camera and starts the MJPEG stream at
// http://<ESP32-CAM-IP>:81/stream. Wi-Fi must be initialized first.
bool cameraESP_iniciar();

#endif
