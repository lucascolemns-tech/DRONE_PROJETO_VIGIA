import customtkinter as ctk
from config import KP_PADRAO, KI_PADRAO, KD_PADRAO
from .tema import COR_BORDA

def abrir_janela_pid(parent, esp):
    janela = ctk.CTkToplevel(parent)
    janela.title("PID")
    janela.geometry("300x340")
    janela.grab_set()
    janela.attributes("-toolwindow", True)

    campos = {}
    for nome, padrao in (("KP (Proporcional)", KP_PADRAO),
                          ("KI (Integral)",     KI_PADRAO),
                          ("KD (Derivativo)",   KD_PADRAO)):
        ctk.CTkLabel(janela, text=f"{nome}:",
                     font=("Arial", 14)).pack(pady=5)
        entry = ctk.CTkEntry(janela, width=200, border_color=COR_BORDA)
        entry.pack(pady=5)
        entry.insert(0, padrao)
        campos[nome] = entry

    erro = ctk.CTkLabel(
        janela,
        text="Os ganhos só podem ser alterados com o drone desarmado.",
        text_color="red",
        wraplength=260,
    )
    erro.pack(pady=8)

    def enviar_pid():
        kp = campos["KP (Proporcional)"].get()
        ki = campos["KI (Integral)"].get()
        kd = campos["KD (Derivativo)"].get()
        if esp.enviar_pid(kp, ki, kd):
            erro.configure(text="Solicitação enviada; o drone só aceita ganhos desarmado.", text_color="green")
        else:
            erro.configure(text="Falha ao enviar. Verifique os valores e a conexão.", text_color="red")

    ctk.CTkButton(janela, text="ENVIAR GANHOS PID", command=enviar_pid, corner_radius=0,
                  text_color="black", font=("Arial Black", 16),
                  fg_color="#FFFFFF", border_width=3, border_color=COR_BORDA,
                  hover_color="#ffffff"
                  ).pack(fill="x", side="bottom")
