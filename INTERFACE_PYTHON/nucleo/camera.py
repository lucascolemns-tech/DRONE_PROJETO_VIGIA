import threading
import time

import cv2
from PIL import Image, ImageTk


class Camera:
    def __init__(self, url):
        self.url = url
        self.cap = None
        self._frame = None
        self._erro = None
        self._lock = threading.Lock()
        self._parar = threading.Event()
        self._thread = None

    def iniciar(self):
        if self._thread is not None and self._thread.is_alive():
            return True
        self._parar.clear()
        self._erro = None
        self._thread = threading.Thread(target=self._capturar, daemon=True)
        self._thread.start()
        return True

    def _abrir_captura(self):
        captura = cv2.VideoCapture()
        parametros = [
            getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", 53), 2000,
            getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", 54), 1000,
        ]
        try:
            aberto = captura.open(self.url, cv2.CAP_ANY, parametros)
        except (TypeError, cv2.error):
            aberto = captura.open(self.url)
        if not aberto:
            captura.release()
            captura = cv2.VideoCapture(self.url)
            if not captura.isOpened():
                captura.release()
                return None
        captura.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return captura

    def _capturar(self):
        try:
            captura = self._abrir_captura()
        except Exception as erro:
            self._erro = str(erro)
            return
        if captura is None:
            self._erro = f"Não foi possível abrir {self.url}"
            return

        with self._lock:
            self.cap = captura

        intervalo_frame = 1.0 / 15.0
        proximo_frame = time.monotonic()
        while not self._parar.is_set():
            espera = proximo_frame - time.monotonic()
            if espera > 0 and self._parar.wait(espera):
                break
            try:
                ret, frame = captura.read()
            except cv2.error as erro:
                self._erro = str(erro)
                break
            proximo_frame = time.monotonic() + intervalo_frame
            if ret:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.02)

        captura.release()
        with self._lock:
            if self.cap is captura:
                self.cap = None

    def erro(self):
        return self._erro

    def ler_frame(self, largura, altura):
        with self._lock:
            frame = None if self._frame is None else self._frame.copy()
        if frame is None:
            return None

        frame = cv2.resize(frame, (largura, altura))
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imagem = Image.fromarray(frame_rgb)
        return ImageTk.PhotoImage(imagem)

    def fechar(self):
        self._parar.set()
        if self._thread is not None:
            self._thread.join(timeout=0.1)
        with self._lock:
            self._frame = None
