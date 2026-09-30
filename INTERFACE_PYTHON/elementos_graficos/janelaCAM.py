import customtkinter as ctk
from nucleo import Camera

def abrir_janela_camera(parent, url):
    cam = Camera(url)
    if not cam.iniciar():
        print(f"ERRO: não foi possível abrir câmera em {url}")
        return

    janela = ctk.CTkToplevel(parent)
    janela.title("Câmera")
    janela.geometry("640x480")

    label = ctk.CTkLabel(janela, text="")
    label.pack(fill="both", expand=True)

    def atualizar():
        w = janela.winfo_width() or 640
        h = janela.winfo_height() or 480
        img = cam.ler_frame(w, h)
        if img is not None:
            label.configure(image=img)
            setattr(label, "image", img)
        janela.after(30, atualizar)

    def fechar():
        cam.fechar()
        janela.destroy()

    janela.protocol("WM_DELETE_WINDOW", fechar)
    atualizar()