import customtkinter as ctk
from ..tema import estilo_botao


class TabPID(ctk.CTkFrame):
    def __init__(self, master, app):
        super().__init__(master, fg_color="transparent")
        self.app = app

        esq = ctk.CTkFrame(self, corner_radius=8, fg_color="#FFFFFF",
                            border_width=3, border_color="#979DA2",
                            width=200, height=650)
        esq.pack(fill="both", expand=True, padx=15, pady=15, side="left")
        esq.pack_propagate(False)

        dir_ = ctk.CTkFrame(self, width=550, height=650, corner_radius=8,
                             fg_color="#FFFFFF",
                             border_width=3, border_color="#979DA2")
        dir_.pack(fill="both", expand=True, padx=15, pady=15, side="right")

        e = estilo_botao()

        self.btn_graficos = ctk.CTkButton(
            esq, text="def gráficos", state="disabled",
            command=app.abrir_graficos, **e)
        self.btn_parametros = ctk.CTkButton(
            esq, text="def parametros", state="disabled",
            command=app.abrir_pid, **e)

        self.btn_graficos.pack(fill="x")
        self.btn_parametros.pack(fill="x")

    def atualizar_estado(self, conectado: bool):
        estado = "normal" if conectado else "disabled"
        self.btn_graficos.configure(state=estado)
        self.btn_parametros.configure(state=estado)
