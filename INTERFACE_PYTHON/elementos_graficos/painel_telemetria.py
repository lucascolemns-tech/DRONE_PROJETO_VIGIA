import customtkinter as ctk
from .tema import COR_BORDA, FONTE_TELEMETRIA, FONTE_SECAO, FONTE_ROTULO, FONTE_VALOR, FONTE_MOTOR

class PainelTelemetria(ctk.CTkFrame):
    def __init__(self, master):
        super().__init__(master, fg_color="#FFFFFF",
                         corner_radius=8, border_width=3,
                         border_color=COR_BORDA)
        
        self.labels = {}
        self.barras = {}

        ctk.CTkLabel(self, text="TELEMETRIA", font=FONTE_TELEMETRIA,
                     text_color="#000000").pack(pady=(6, 2))
        self.link_status = ctk.CTkLabel(self, text="AGUARDANDO STM32", text_color="#B45309")
        self.link_status.pack(pady=(0, 4))

        corpo = ctk.CTkFrame(self, fg_color="transparent")
        corpo.pack(fill="both", expand=True, padx=12, pady=(0, 6))
        self.corpo = corpo

        #atitude
        self._secao("ATITUDE")
        self._linha_valores("roll",  "Roll",  "pitch", "Pitch")
        self._linha_valores("yaw",   "Yaw (mag)", None, None)
        self.mag_status = ctk.CTkLabel(self.corpo, text="Bússola: sem calibração",
                                       font=FONTE_ROTULO, text_color="#B45309", anchor="w",
                                       wraplength=460, justify="left")
        self.mag_status.pack(fill="x", pady=(2, 0))
        self.mag_progress = ctk.CTkProgressBar(self.corpo, height=8)
        self.mag_progress.set(0)
        self.mag_progress.pack(fill="x", pady=(2, 4))
        #Kalman
        self._secao("KALMAN")
        self._linha_valores("alt", "Alt", "vel", "Vel")
        #Baromêtro
        self._secao("BARÔMETRO")
        self._linha_valores("temp", "Temp", None, None)
        self._secao("OUTROS SENSORES")
        self._linha_valores("tensao", "Bateria", "gps_alt", "GPS alt")
        #Motores
        self._secao("MOTORES")
        self._linha_motores_duplos("m1", "m2")
        self._linha_motores_duplos("m3", "m4")

    def _secao(self, titulo):
        ctk.CTkLabel(self.corpo, text=titulo,
                     font=FONTE_SECAO,
                     text_color="#4B5563",
                     anchor="w").pack(fill="x", pady=(4, 1))

    def _linha_valores(self, k1, l1, k2, l2):
        frame = ctk.CTkFrame(self.corpo, fg_color="transparent")
        frame.pack(fill="x", pady=0)

        cel1 = ctk.CTkFrame(frame, fg_color="transparent")
        cel1.pack(side="left", expand=True, fill="x")
        self._celula(cel1, k1, l1)

        if k2:
            cel2 = ctk.CTkFrame(frame, fg_color="transparent")
            cel2.pack(side="left", expand=True, fill="x")
            self._celula(cel2, k2, l2)

    def _celula(self, parent, chave, rotulo):
        ctk.CTkLabel(parent, text=f"{rotulo}:",
                     font=FONTE_ROTULO,
                     text_color="#6B7280",
                     width=76, anchor="w").pack(side="left")
        lbl = ctk.CTkLabel(parent, text="--",
                           font=FONTE_VALOR,
                           text_color="#000000",
                           anchor="w")
        lbl.pack(side="left", padx=(2, 0))
        self.labels[chave] = lbl

    def _linha_motores_duplos(self, chave_a, chave_b):
        frame = ctk.CTkFrame(self.corpo, fg_color="transparent")
        frame.pack(fill="x", pady=2)

        cel_a = ctk.CTkFrame(frame, fg_color="transparent")
        cel_a.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self._motor_celula(cel_a, chave_a)

        cel_b = ctk.CTkFrame(frame, fg_color="transparent")
        cel_b.pack(side="left", expand=True, fill="x", padx=(4, 0))
        self._motor_celula(cel_b, chave_b)

    def _motor_celula(self, parent, chave):
        ctk.CTkLabel(parent, text=chave.upper(),
                     font=FONTE_MOTOR,
                     text_color="#1F2937",
                     width=30, anchor="w").pack(side="left")

        barra = ctk.CTkProgressBar(parent, height=10,
                                    progress_color="#4B5563",
                                    fg_color="#E5E7EB",
                                    corner_radius=4)
        barra.set(0)
        barra.pack(side="left", fill="x", expand=True, padx=6)
        self.barras[chave] = barra

        valor = ctk.CTkLabel(parent, text="0",
                             font=FONTE_MOTOR,
                             text_color="#000000",
                             width=38, anchor="e")
        valor.pack(side="left")
        self.labels[chave] = valor

    def limpar(self, status="TELEMETRIA DESATUALIZADA"):
        self.link_status.configure(text=status, text_color="#B91C1C")
        self.mag_status.configure(text="Bússola: telemetria desatualizada", text_color="#B91C1C")
        self.mag_progress.set(0)
        for label in self.labels.values():
            label.configure(text="--")
        for barra in self.barras.values():
            barra.set(0)

    def atualizar(self, dados):
        status_mag = dados.get("mag_status", 4)
        progresso_mag = max(0.0, min(100.0, dados.get("mag_progress", 0.0)))
        self.mag_progress.set(progresso_mag / 100.0 if status_mag == 1 else (1.0 if status_mag == 2 else 0.0))
        if status_mag == 1:
            texto_mag, cor_mag = f"Calibrando bússola: {progresso_mag:.0f}%", "#B45309"
        elif status_mag == 2:
            texto_mag, cor_mag = "Bússola calibrada · rumo magnético", "#15803D"
        elif status_mag == 3:
            texto_mag, cor_mag = "Calibração falhou · repita em todos os eixos", "#B91C1C"
        elif status_mag == 4:
            texto_mag, cor_mag = "Magnetômetro não detectado", "#B91C1C"
        else:
            texto_mag, cor_mag = "Bússola sem calibração", "#B45309"
        self.mag_status.configure(text=texto_mag, text_color=cor_mag)

        if not dados.get("link_ok", False):
            self.limpar("STM32 SEM TELEMETRIA")
            return
        if not dados.get("sistema_pronto", False):
            self.limpar("SENSORES INDISPONÍVEIS")
            self.labels["tensao"].configure(text="N/A" if dados["tensao"] < 0 else f"{dados['tensao']:.2f}V")
            return
        self.link_status.configure(text="SENSORES OK", text_color="#15803D")

        # formato: ang_x, ang_y, ang_z, alt, vel, temp, m1, m2, m3, m4
        self.labels["roll"].configure(text=f"{dados['ang_x']:+.2f}°")
        self.labels["pitch"].configure(text=f"{dados['ang_y']:+.2f}°")
        self.labels["yaw"].configure(text=f"{dados['ang_z']:+.2f}°")
        self.labels["alt"].configure(text=f"{dados['alt']:.3f}m")
        self.labels["vel"].configure(text=f"{dados['vel']:+.2f}m/s")
        self.labels["temp"].configure(text=f"{dados['temp']:.1f}°C")
        self.labels["tensao"].configure(text="N/A" if dados["tensao"] < 0 else f"{dados['tensao']:.2f}V")
        self.labels["gps_alt"].configure(text=f"{dados['gps_alt']:.1f}m")

        for m in ("m1", "m2", "m3", "m4"):
            v = dados[m]
            self.labels[m].configure(text=f"{v:.0f}")
            self.barras[m].set(max(0.0, min(1.0, (v - 1000.0) / 1000.0)))
