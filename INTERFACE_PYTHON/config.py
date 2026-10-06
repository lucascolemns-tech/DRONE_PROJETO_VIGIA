
import os

DIRETORIO_RAIZ = os.path.dirname(os.path.abspath(__file__))

ESP_IP         = os.environ.get("META_DRONE_ESP_HOST", os.environ.get("META_DRONE_ESP_IP", "10.85.164.132"))
ESP_PORT       = 1244
SOCKET_TIMEOUT = 0.25
CAMERA_URL     = "http://192.168.1.19:81/stream"

CONTROLE_EXE = os.path.join(DIRETORIO_RAIZ, "ferramentas", "controle_precision.exe")
ICONE_IMG    = os.path.join(DIRETORIO_RAIZ, "VIGIA.ico")
PASTA_LOGS   = os.path.join(DIRETORIO_RAIZ, "logs")     # opcional

MAX_ROLL, MAX_PITCH, MAX_YAW = 20.0, 20.0, 30.0
MAX_THROTTLE = 1.0
THROTTLE_ARM_AXIS = 0.05
CONTROLE_DEADZONE = 0.035
CONTROLE_EXPO = 0.25
STICK_MIN, STICK_MAX = -1000, 1000

WINDOW_W, WINDOW_H = 950, 600
WINDOW_TITULO = "INTERFACE DRONE"

KP_PADRAO = "5.0"
KI_PADRAO = "0.03"
KD_PADRAO = "0.8"

os.makedirs(PASTA_LOGS, exist_ok=True)
