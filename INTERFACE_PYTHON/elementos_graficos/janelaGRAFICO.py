import tkinter as tk
from tkinter import ttk
import csv
import math
import os
import time

import customtkinter as ctk
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from config import KP_PADRAO, KI_PADRAO, KD_PADRAO, PASTA_LOGS
from .tema import COR_BORDA, estilo_botao


def gerar_referencia(xs, tipo, amplitude, periodo, offset, origem=0.0):
    xs = np.asarray(xs, dtype=float)
    fase = 2 * np.pi * (xs - origem) / periodo
    if tipo == "Senoidal":
        onda = np.sin(fase)
    elif tipo == "Quadrada":
        onda = np.where(np.sin(fase) >= 0, 1.0, -1.0)
    elif tipo == "Triangular":
        onda = (2.0 / np.pi) * np.arcsin(np.sin(fase))
    else:
        onda = np.zeros_like(xs)
    return offset + amplitude * onda

class JanelaGraficos(ctk.CTkToplevel):
    def __init__(self, parent, esp):
        super().__init__(parent)
        self.title("Análise PID")
        self.geometry("1200x700")
        self.esp = esp

        #buffers do gráfico
        self.xs = []
        self.ys = []
        self.tamanho = 200
        self._plot_dirty = True
        self.inicio = time.monotonic()
        self.inicio_referencia = 0.0

        self.amplitude = 15
        self.xbase = 0
        self.periodo = 15
        self.resposta = "Constante"

        self.var_kp = tk.StringVar(value=KP_PADRAO)
        self.var_ki = tk.StringVar(value=KI_PADRAO)
        self.var_kd = tk.StringVar(value=KD_PADRAO)
        self.var_onda = tk.StringVar(value="20,15,0")
        self.var_tipo = tk.StringVar(value="Constante")

        self._construir_interface()
        self._loop_desenho()

    def _construir_interface(self):
        #coluna esquerda: controles
        ctrl = ctk.CTkFrame(self, fg_color="#FFFFFF",
                            corner_radius=8, border_width=3,
                            border_color=COR_BORDA)
        ctrl.pack(side="left", fill="y", padx=10, pady=10)

        ctk.CTkLabel(ctrl, text="ANÁLISE PID",
                     font=("Arial Black", 18)).pack(pady=(15, 10))
        for rotulo, var in (("KP", self.var_kp),
                            ("KI", self.var_ki),
                            ("KD", self.var_kd)):
            ctk.CTkLabel(ctrl, text=rotulo,
                         font=("Arial Black", 12)).pack(pady=(8, 2))
            ctk.CTkEntry(ctrl, textvariable=var, width=200,
                         border_color=COR_BORDA).pack(pady=(0, 4))

        e = estilo_botao()
        ctk.CTkButton(ctrl, text="Enviar ganhos PID", command=self._enviar_pid, **e).pack(pady=12)

        #setpoint
        ctk.CTkLabel(ctrl, text="DEFINIÇÃO DE SETPOINT", font=("Arial Black", 12)).pack(pady=(15, 5))

        ttk.Combobox(
            ctrl,
            textvariable=self.var_tipo,
            values=["Constante", "Senoidal", "Quadrada", "Triangular"],
            state="readonly", width=22,
        ).pack(pady=4)

        ctk.CTkLabel(ctrl,
                     text="amplitude, período (s), offset\nEx: 20,15,0",
                     font=("Arial", 11)).pack(pady=(10, 4))
        ctk.CTkEntry(ctrl, textvariable=self.var_onda, width=200,
                     border_color=COR_BORDA).pack(pady=4)

        ctk.CTkButton(ctrl, text="Atualizar referência",
                      command=self._atualizar_referencia,
                      **e).pack(pady=12)

        ctk.CTkButton(ctrl, text="Enviar referência ao drone",
                      command=self._enviar_referencia_drone,
                      **e).pack(pady=6)

        ctk.CTkButton(ctrl, text="Parar referência no drone",
                      command=self._parar_referencia_drone,
                      **e).pack(pady=6)

        ctk.CTkButton(ctrl, text="Salvar gráfico",
                      command=self._salvar_grafico,
                      **e).pack(pady=6)

        #status
        self.lbl_status = ctk.CTkLabel(ctrl, text="Aguardando telemetria",
                                       font=("Arial Black", 11))
        self.lbl_status.pack(pady=15)

        #coluna direita: gráfico
        fr_graf = ctk.CTkFrame(self, fg_color="#FFFFFF",
                               corner_radius=8, border_width=3,
                               border_color=COR_BORDA)
        fr_graf.pack(side="left", fill="both", expand=True,
                     padx=10, pady=10)

        self.fig = Figure(figsize=(8, 5), dpi=85)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Roll medido x referência")
        self.ax.set_xlabel("Tempo (s)")
        self.ax.set_ylabel("Ângulo (°)")
        self.ax.grid(True)

        self.linha_dados, = self.ax.plot([], [], linestyle='-',
                                         color="#1408BD", label="Roll medido")
        self.linha_sp, = self.ax.plot([], [], color="#DC2626",
                                      label="Referência")
        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.fig, master=fr_graf)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, fr_graf).update()

    #funções
    def _enviar_pid(self):
        if self.esp.enviar_pid(self.var_kp.get(), self.var_ki.get(), self.var_kd.get()):
            self.lbl_status.configure(text="Ganhos enviados; aceitos só desarmado")
        else:
            self.lbl_status.configure(text="Falha ao enviar ganhos PID")

    def _atualizar_referencia(self):
        self.resposta = self.var_tipo.get()
        try:
            a, p, x = (float(valor) for valor in self.var_onda.get().split(","))
            if not all(math.isfinite(valor) for valor in (a, p, x)) or p < 0.5:
                raise ValueError
            if abs(a) > 180 or abs(x) > 180:
                raise ValueError
            self.amplitude, self.periodo, self.xbase = a, p, x
            self.inicio_referencia = self.xs[-1] if self.xs else 0.0
            self._plot_dirty = True
        except (TypeError, ValueError, OverflowError):
            self.lbl_status.configure(text="Parâmetros inválidos")
            return False
        self.lbl_status.configure(text=f"Referência simulada: {self.resposta}")
        return True

    def _enviar_referencia_drone(self):
        if not self._atualizar_referencia():
            return
        if self.esp.enviar_referencia(self.resposta, self.amplitude, self.periodo, self.xbase):
            self.inicio_referencia = self.xs[-1] if self.xs else 0.0
            self.lbl_status.configure(text="Referência enviada; use o controle de throttle")
        else:
            self.lbl_status.configure(text="Referência fora da faixa ou sem conexão")

    def _parar_referencia_drone(self):
        if self.esp.desativar_referencia():
            self.lbl_status.configure(text="Referência desativada no drone")
        else:
            self.lbl_status.configure(text="Falha ao desativar a referência")

    def _salvar_grafico(self):
        try:
            if not self.xs:
                self.lbl_status.configure(text="Sem amostras para salvar")
                return
            caminho = os.path.join(PASTA_LOGS, "pid_roll.csv")
            with open(caminho, "a", newline="", encoding="utf-8") as arquivo:
                gravador = csv.writer(arquivo)
                if arquivo.tell() == 0:
                    gravador.writerow(("tempo_s", "roll_graus", "referencia_graus"))
                referencia = gerar_referencia(self.xs, self.resposta, self.amplitude,
                                              self.periodo, self.xbase,
                                              self.inicio_referencia)
                gravador.writerows(zip(self.xs, self.ys, referencia))
            self.lbl_status.configure(text="Dados salvos em logs/pid_roll.csv")
        except Exception as e:
            self.lbl_status.configure(text=f"Erro ao salvar: {e}")

    #recebemos dados
    def adicionar_dado(self, valor, instante=None):
        """App chama isto a cada pacote recebido do ESP32."""
        valor = float(valor)
        if not math.isfinite(valor):
            return
        instante = time.monotonic() if instante is None else instante
        tempo = instante - self.inicio
        if tempo < 0:
            return
        self.xs.append(tempo)
        self.ys.append(valor)
        if len(self.ys) > self.tamanho:
            self.xs.pop(0)
            self.ys.pop(0)
        self._plot_dirty = True

    #desenhar gráficos
    def _loop_desenho(self):
        if not self.winfo_exists():
            return
        if self._plot_dirty:
            self._atualizar_plot()
        self.after(100, self._loop_desenho)

    def _atualizar_plot(self):
        if len(self.xs) < 2:
            self._plot_dirty = False
            return

        xs = self.xs
        ys = self.ys

        #atualiza todas as linhas
        self.linha_dados.set_data(xs, ys)
        self.linha_sp.set_data(xs, gerar_referencia(xs, self.resposta, self.amplitude,
                                                    self.periodo, self.xbase,
                                                    self.inicio_referencia))
        self.linha_sp.set_drawstyle("steps-post" if self.resposta == "Quadrada" else "default")
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()
        self._plot_dirty = False
