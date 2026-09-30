import cv2
from PIL import Image, ImageTk

class Camera:
    def __init__(self, url):
        self.url = url
        self.cap = None

    def iniciar(self):
        self.cap = cv2.VideoCapture(self.url)
        return self.cap.isOpened()

    def ler_frame(self, largura, altura):
        if self.cap is None: return None
        
        ret, frame = self.cap.read()
        if not ret: return None

        frame = cv2.resize(frame, (largura, altura))
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        imagem = Image.fromarray(frame_rgb)
        return ImageTk.PhotoImage(imagem)

    def fechar(self):
        if self.cap:
            self.cap.release()
            self.cap = None