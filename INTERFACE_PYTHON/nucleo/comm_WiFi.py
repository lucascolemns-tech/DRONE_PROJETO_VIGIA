import math
import re
import socket
import threading
import time

from .telemetria import _recuperar_frames_colados


_PROGRESSO_COMPLETO = re.compile(r"-?\d+\.\d")


class ESPCom:
    def __init__(self, ip, porta, timeout=2.0):
        self.ip = ip
        self.porta = porta
        self.timeout = timeout
        self.sock = None
        self.buffer = bytearray()
        self._lock = threading.RLock()
        self._ultima_tentativa = 0.0
        self._intervalo_reconexao = 1.0
        self._ultima_linha = 0.0
        # A rede pode entregar vários frames em rajadas; não derrube a sessão
        # por um intervalo curto sem uma linha completa.
        self._limite_sem_linha = 10.0
        self._sem_linha_reportado = False
        self._bytes_desde_linha = 0

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
                # Allow for a lost SYN/ACK on the ESP32 Wi-Fi link. Two seconds
                # was short enough to abandon otherwise recoverable handshakes.
                novo_sock.settimeout(max(self.timeout, 5.0))
                novo_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                novo_sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                novo_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
                novo_sock.connect((self.ip, self.porta))
                novo_sock.settimeout(self.timeout)
            except OSError as erro:
                novo_sock.close()
                print(f"Falha ao conectar em {self.ip}:{self.porta}: {erro}")
                return False

            self.sock = novo_sock
            self.buffer.clear()
            self._ultima_linha = time.monotonic()
            self._sem_linha_reportado = False
            self._bytes_desde_linha = 0
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
            with self._lock:
                ainda_atual = self.sock is sock
            if ainda_atual:
                print(f"Falha ao enviar comando ao ESP32: {erro}")
                self._fechar_socket(sock, reconectar_imediato=True)
            return False

    def enviar_pid(self, kp, ki, kd):
        try:
            kp, ki, kd = (float(valor) for valor in (kp, ki, kd))
        except (TypeError, ValueError, OverflowError):
            return False
        if not all(math.isfinite(valor) for valor in (kp, ki, kd)):
            return False
        if not 0.0 <= kp <= 10.0 or not 0.0 <= ki <= 1.0 or not 0.0 <= kd <= 2.0:
            return False
        return self._enviar_linha(f"PID,{kp:.4f},{ki:.4f},{kd:.4f}\n")

    def enviar_referencia(self, tipo, amplitude, periodo, offset):
        tipos = {"Constante": "CONST", "Senoidal": "SENO", "Quadrada": "QUAD", "Triangular": "TRI"}
        try:
            amplitude, periodo, offset = (float(valor) for valor in (amplitude, periodo, offset))
        except (TypeError, ValueError, OverflowError):
            return False
        if tipo not in tipos or not all(math.isfinite(valor) for valor in (amplitude, periodo, offset)):
            return False
        if periodo < 0.5 or periodo > 300.0 or abs(amplitude) + abs(offset) > 45.0:
            return False
        return self._enviar_linha(
            f"REF,{tipos[tipo]},{amplitude:.3f},{periodo:.3f},{offset:.3f}\n"
        )

    def desativar_referencia(self):
        return self._enviar_linha("REF_OFF\n")

    def _enviar_linha(self, msg):
        if not self.conectar():
            return False
        sock = None
        try:
            with self._lock:
                sock = self.sock
                if sock is None:
                    return False
                sock.sendall(msg.encode("ascii"))
            return True
        except OSError as erro:
            with self._lock:
                ainda_atual = self.sock is sock
            if ainda_atual:
                print(f"Falha ao enviar comando ao ESP32: {erro}")
                self._fechar_socket(sock, reconectar_imediato=True)
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
        if self._reiniciar_sessao_se_ociosa(sock):
            return None

        try:
            dados = sock.recv(1024)
            if not dados:
                print("Conexão TCP encerrada pelo ESP32.")
                self._fechar_socket(sock)
                return None

            with self._lock:
                self.buffer.extend(dados)
                self._bytes_desde_linha += len(dados)
                if len(self.buffer) > 8192:
                    ultima_linha = self.buffer.rfind(b"\n")
                    if ultima_linha >= 0:
                        del self.buffer[:ultima_linha + 1]
                    else:
                        self.buffer.clear()
                    print("Buffer TCP excedeu o limite; dados antigos descartados.")
                linha = self._extrair_linha()
            if linha is None:
                self._reiniciar_sessao_se_ociosa(sock)
            return linha
        except socket.timeout:
            self._reiniciar_sessao_se_ociosa(sock)
            return None
        except OSError as erro:
            with self._lock:
                ainda_atual = self.sock is sock
            if ainda_atual:
                print(f"Falha ao receber telemetria: {erro}")
                self._fechar_socket(sock, reconectar_imediato=True)
            return None

    def _reiniciar_sessao_se_ociosa(self, sock):
        with self._lock:
            if (self.sock is not sock or self._sem_linha_reportado or
                    time.monotonic() - self._ultima_linha < self._limite_sem_linha):
                return False
            bytes_buffer = self._bytes_desde_linha
            tamanho_buffer = len(self.buffer)
            amostra = bytes(self.buffer[:96]).decode("ascii", errors="replace")
            self._sem_linha_reportado = True
            if bytes_buffer == 0 and tamanho_buffer == 0:
                print("TCP conectado, mas o ESP32 ainda não enviou telemetria; mantendo a sessão aberta.")
                return False

        print(
            f"Sem frame TCP completo por {self._limite_sem_linha:.1f} s; "
            f"bytes_recebidos={bytes_buffer}, buffer={tamanho_buffer}, "
            f"amostra={amostra!r}; reiniciando a sessão TCP."
        )
        self._fechar_socket(sock, reconectar_imediato=True)
        return True

    def _extrair_linha(self):
        separador = self.buffer.find(b"\n")
        if separador >= 0:
            linha = bytes(self.buffer[:separador]).rstrip(b"\r")
            del self.buffer[:separador + 1]
            try:
                linha_decodificada = linha.decode("ascii")
            except UnicodeDecodeError:
                return None
        else:
            # Alguns firmwares deixam de enviar LF. O protocolo atual tem 16 campos:
            # o progresso final sempre usa uma casa decimal e fecha o frame.
            try:
                bruto = self.buffer.decode("ascii")
            except UnicodeDecodeError:
                return None

            partes = bruto.split(",")
            if len(partes) == 16 and _PROGRESSO_COMPLETO.fullmatch(partes[-1]):
                linha_decodificada = bruto
                self.buffer.clear()
            elif len(partes) > 16:
                recuperadas = _recuperar_frames_colados(partes, manter_todos=True)
                if len(recuperadas) < 16 or not _PROGRESSO_COMPLETO.fullmatch(recuperadas[15]):
                    return None
                linha_decodificada = ",".join(recuperadas[:16])
                restante = ",".join(recuperadas[16:])
                self.buffer = bytearray(restante.encode("ascii"))
            else:
                return None

        self._ultima_linha = time.monotonic()
        self._sem_linha_reportado = False
        self._bytes_desde_linha = 0
        return linha_decodificada

    def _fechar_socket(self, sock, reconectar_imediato=False):
        with self._lock:
            if self.sock is not sock:
                return
            self.sock = None
            self.buffer.clear()
            if reconectar_imediato:
                # Permite que a próxima leitura abra uma conexão nova.
                # Se ela falhar, conectar() mantém o intervalo normal entre tentativas.
                self._ultima_tentativa = 0.0
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
