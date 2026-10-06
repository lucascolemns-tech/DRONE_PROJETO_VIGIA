# nucleo/controle_hardware.py
import subprocess
import math

class ControleHardware:
    def __init__(self, caminho_exe, max_roll, max_pitch, max_yaw,
                 stick_min=-1000, stick_max=1000, max_throttle=1.0,
                 throttle_arm_axis=0.05, controle_deadzone=0.035,
                 controle_expo=0.25):
        #recebe dados globais de "INTERFACE_PYTHON/config"
        self.caminho_exe = caminho_exe
        self.max_roll = max_roll
        self.max_pitch = max_pitch
        self.max_yaw = max_yaw
        self.stick_min = stick_min
        self.stick_max = stick_max
        self.max_throttle = max_throttle
        self.throttle_arm_axis = throttle_arm_axis
        self.controle_deadzone = max(0.0, min(0.25, controle_deadzone))
        self.controle_expo = max(0.0, min(1.0, controle_expo))
        self.processo = None

    def iniciar(self):
        try:
            self.processo = subprocess.Popen(
                [self.caminho_exe],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                text=True
            )
            return True, f"Controle iniciado: {self.caminho_exe}"
        except Exception as e:
            self.processo = None
            return False, f"Erro ao abrir controle.exe: {e}"

    def _mapear(self, valor, entrada_min, entrada_max, saida_min, saida_max):
        valor = max(entrada_min, min(entrada_max, valor))
        return (valor - entrada_min) * (saida_max - saida_min) / (entrada_max - entrada_min) + saida_min

    def _mapear_eixo(self, valor, limite):
        normalizado = self._mapear(valor, self.stick_min, self.stick_max, -1.0, 1.0)
        magnitude = abs(normalizado)
        if magnitude <= self.controle_deadzone:
            return 0.0
        magnitude = (magnitude - self.controle_deadzone) / (1.0 - self.controle_deadzone)
        magnitude = (1.0 - self.controle_expo) * magnitude + self.controle_expo * magnitude ** 3
        return math.copysign(magnitude * limite, normalizado)

    def ler_comandos(self):
        if self.processo is None or self.processo.stdout is None:
            return None

        if self.processo.poll() is not None and self.processo.stdout.closed:
            return None

        linha = self.processo.stdout.readline(256)
        if not linha:
            return None

        partes = linha.strip().split(",")
        if len(partes) != 4:
            return None

        try:
            lx = float(partes[0])
            ly = float(partes[1])
            rx = float(partes[2])
            ry = float(partes[3])

            if not all(math.isfinite(valor) for valor in (lx, ly, rx, ry)):
                return None

            roll = self._mapear_eixo(rx, self.max_roll)
            pitch = self._mapear_eixo(ry, self.max_pitch)
            yaw = self._mapear_eixo(lx, self.max_yaw)
            throttle_axis = self._mapear(ly, self.stick_min, self.stick_max, 0.0, 1.0)
            limite_armar = max(0.0, min(0.25, self.throttle_arm_axis))
            if throttle_axis <= limite_armar:
                throttle = 0.0
            else:
                throttle = self._mapear(throttle_axis, limite_armar, 1.0,
                                        0.0, self.max_throttle)

            return roll, pitch, yaw, throttle

        except ValueError:
            return None

    def fechar(self):
        if self.processo:
            if self.processo.poll() is None:
                self.processo.terminate()
                try:
                    self.processo.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.processo.kill()
                    self.processo.wait()
            if self.processo.stdout is not None:
                self.processo.stdout.close()
            self.processo = None
