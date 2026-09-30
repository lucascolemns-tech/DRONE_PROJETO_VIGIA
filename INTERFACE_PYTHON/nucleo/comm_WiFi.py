import math
import socket
import threading
import time


class ESPCom:
    def __init__(self, ip, porta, timeout=2.0):
        self.ip = ip
        self.porta = porta
        self.timeout = timeout
        self.sock = None
        self.buffer = bytearray()
        self._lock = threading.RLock()
        self._ultima_tentativa = 0.0
        self._intervalo_reconexao = 2.0

    def conectar(self, forcar=False):
        with self._lock:
            if self.sock is not None:
                return True

            agora = time.monotonic()
            if not forcar and agora - self._ultima_tentativa < self._intervalo_reconexao:
                return False
            self._ultima_tentativa = agora

            novo_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                novo_sock.settimeout(self.timeout)
                novo_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                novo_sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                novo_sock.connect((self.ip, self.porta))
            except OSError as erro:
                novo_sock.close()
                print(f"Falha ao conectar em {self.ip}:{self.porta}: {erro}")
                return False

            self.sock = novo_sock
            self.buffer.clear()
            print(f"Conectado ao ESP32 em {self.ip}:{self.porta}")
            return True

    def enviar_setpoint(self, roll, pitch, yaw, throttle):
        valores = (roll, pitch, yaw, throttle)
        try:
            valores = tuple(float(valor) for valor in valores)
        except (TypeError, ValueError, OverflowError):
            return False
        if not all(math.isfinite(valor) for valor in valores):
            return False

        roll, pitch, yaw, throttle = valores
        if abs(roll) > 45.0 or abs(pitch) > 45.0 or abs(yaw) > 180.0 or not 0.0 <= throttle <= 1.0:
            return False

        if not self.conectar():
            return False

        try:
            msg = f"{roll:.2f},{pitch:.2f},{yaw:.2f},{throttle:.3f}\n"
            with self._lock:
                sock = self.sock
                if sock is None:
                    return False
                sock.sendall(msg.encode("ascii"))
            return True
        except OSError as erro:
            print(f"Falha ao enviar comando ao ESP32: {erro}")
            if sock is not None:
                self._fechar_socket(sock)
            return False

    def solicitar_calibracao_magnetometro(self):
        if not self.conectar():
            return False

        sock = None
        try:
            with self._lock:
                sock = self.sock
                if sock is None:
                    return False
                sock.sendall(b"CAL_MAG\n")
            return True
        except OSError as erro:
            print(f"Falha ao solicitar calibracao do magnetometro: {erro}")
            if sock is not None:
                self._fechar_socket(sock)
            return False

    def receber_dados(self):
        if not self.conectar():
            return None

        with self._lock:
            sock = self.sock
            linha = self._extrair_linha()
        if linha is not None:
            return linha
        if sock is None:
            return None

        try:
            dados = sock.recv(1024)
            if not dados:
                self._fechar_socket(sock)
                return None

            with self._lock:
                self.buffer.extend(dados)
                if len(self.buffer) > 8192:
                    ultima_linha = self.buffer.rfind(b"\n")
                    if ultima_linha >= 0:
                        del self.buffer[:ultima_linha + 1]
                    else:
                        self.buffer.clear()
                    print("Buffer TCP excedeu o limite; dados antigos descartados.")
                return self._extrair_linha()
        except socket.timeout:
            return None
        except OSError as erro:
            print(f"Falha ao receber telemetria: {erro}")
            self._fechar_socket(sock)
            return None

    def _extrair_linha(self):
        separador = self.buffer.find(b"\n")
        if separador < 0:
            return None

        linha = bytes(self.buffer[:separador]).rstrip(b"\r")
        del self.buffer[:separador + 1]
        try:
            return linha.decode("ascii")
        except UnicodeDecodeError:
            return None

    def _fechar_socket(self, sock):
        with self._lock:
            if self.sock is sock:
                self.sock = None
                self.buffer.clear()
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def fechar(self):
        with self._lock:
            sock = self.sock
            self.sock = None
            self.buffer.clear()

        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
