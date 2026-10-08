import tkinter as tk
import csv
import math
import os
import time

import customtkinter as ctk
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from config import KP_PADRAO, KI_PADRAO, KD_PADRAO, PASTA_LOGS, ICONE_IMG
from .tema import FONTE_ROTULO, FONTE_TITULO, estilo_botao


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
        if os.path.isfile(ICONE_IMG):
            try:
                self.iconbitmap(ICONE_IMG)
            except Exception:
                pass
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
                            border_color="#979DA2", width=300, height=650)
        ctrl.pack(side="left", fill="y", padx=15, pady=15)
        ctrl.pack_propagate(False)

        controles = ctk.CTkScrollableFrame(
            ctrl, fg_color="transparent", corner_radius=0,
            scrollbar_button_color="#D1D5DB",
            scrollbar_button_hover_color="#9CA3AF",
        )
        controles.pack(fill="both", expand=True, padx=10, pady=(10, 4))

        ctk.CTkLabel(
            controles, text="Análise PID",
            font=FONTE_TITULO, text_color="#111827",
        ).pack(anchor="w", pady=(4, 2))
        ctk.CTkLabel(
            controles, text="Ajuste os ganhos e acompanhe a resposta do roll.",
            font=("Arial", 11), text_color="#6B7280",
            wraplength=245, justify="left",
        ).pack(anchor="w", pady=(0, 12))

        ctk.CTkLabel(
            controles, text="GANHOS PID",
            font=FONTE_ROTULO, text_color="#374151",
        ).pack(anchor="w", pady=(2, 3))

        for rotulo, var in (("KP", self.var_kp),
                            ("KI", self.var_ki),
                            ("KD", self.var_kd)):
            ctk.CTkLabel(
                controles, text=rotulo,
                font=FONTE_ROTULO, text_color="#4B5563",
            ).pack(anchor="w", pady=(7, 3))
            ctk.CTkEntry(
                controles, textvariable=var, height=36,
                border_width=1, border_color="#D1D5DB",
                corner_radius=6,
            ).pack(fill="x")

        estilo = estilo_botao()
        estilo.update(
            border_width=2,
            border_color="#979DA2",
            corner_radius=6,
            height=42,
            font=("Arial Black", 13),
            hover_color="#E5E7EB",
        )
        ctk.CTkButton(
            controles, text="Enviar ganhos PID",
            command=self._enviar_pid, **estilo,
        ).pack(fill="x", pady=(12, 16))

        ctk.CTkLabel(
            controles, text="REFERÊNCIA DE TESTE",
            font=FONTE_ROTULO, text_color="#374151",
        ).pack(anchor="w", pady=(0, 7))

        ctk.CTkOptionMenu(
            controles,
            variable=self.var_tipo,
            values=["Constante", "Senoidal", "Quadrada", "Triangular"],
            height=36,
            fg_color="#4B5563",
            button_color="#374151",
            button_hover_color="#1F2937",
            text_color="#FFFFFF",
            font=("Arial", 12),
            dropdown_fg_color="#FFFFFF",
            dropdown_text_color="#111827",
            dropdown_hover_color="#E5E7EB",
        ).pack(fill="x", pady=(0, 9))

        ctk.CTkLabel(
            controles,
            text="Amplitude, período (s), offset\nExemplo: 20,15,0",
            font=("Arial", 11), text_color="#6B7280",
            justify="left",
        ).pack(anchor="w", pady=(0, 5))
        ctk.CTkEntry(
            controles, textvariable=self.var_onda, height=36,
            border_width=1, border_color="#D1D5DB",
            corner_radius=6,
        ).pack(fill="x")

        ctk.CTkButton(
            controles, text="Atualizar referência",
            command=self._atualizar_referencia, **estilo,
        ).pack(fill="x", pady=(10, 5))

        ctk.CTkButton(
            controles, text="Enviar referência ao drone",
            command=self._enviar_referencia_drone, **estilo,
        ).pack(fill="x", pady=5)

        ctk.CTkButton(
            controles, text="Parar referência no drone",
            command=self._parar_referencia_drone, **estilo,
        ).pack(fill="x", pady=5)

        ctk.CTkButton(
            controles, text="Salvar gráfico",
            command=self._salvar_grafico, **estilo,
        ).pack(fill="x", pady=(5, 10))

        self.lbl_status = ctk.CTkLabel(
            ctrl, text="Aguardando telemetria",
            font=("Arial", 11), text_color="#4B5563",
            fg_color="#F3F4F6", corner_radius=6,
            wraplength=250, justify="left", anchor="w",
            height=42,
        )
        self.lbl_status.pack(fill="x", padx=12, pady=(4, 12))

        #coluna direita: gráfico
        fr_graf = ctk.CTkFrame(
            self, fg_color="#FFFFFF", corner_radius=8,
            border_width=3, border_color="#979DA2",
        )
        fr_graf.pack(side="left", fill="both", expand=True,
                     padx=(0, 15), pady=15)

        ctk.CTkLabel(
            fr_graf, text="Acompanhamento do roll",
            font=("Arial Black", 15), text_color="#111827",
        ).pack(anchor="w", padx=18, pady=(14, 0))

        self.fig = Figure(figsize=(9, 6), dpi=85, facecolor="#FFFFFF")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#FFFFFF")
        self.ax.set_title("Roll medido x referência",
                          fontsize=12, color="#111827", pad=12)
        self.ax.set_xlabel("Tempo (s)", fontsize=10, color="#374151")
        self.ax.set_ylabel("Ângulo (°)", fontsize=10, color="#374151")
        self.ax.tick_params(colors="#4B5563", labelsize=9)
        self.ax.grid(True, color="#E5E7EB", linewidth=0.8)
        self.ax.set_axisbelow(True)
        for borda in self.ax.spines.values():
            borda.set_color("#D1D5DB")

        self.linha_dados, = self.ax.plot(
            [], [], linestyle="-", color="#2563EB",
            linewidth=1.8, label="Roll medido",
        )
        self.linha_sp, = self.ax.plot(
            [], [], color="#EF4444", linewidth=1.8,
            label="Referência",
        )
        self.ax.legend(
            loc="upper right", frameon=True, fontsize=9,
            facecolor="#FFFFFF", edgecolor="#D1D5DB",
        )

        self.canvas = FigureCanvasTkAgg(self.fig, master=fr_graf)
        self.canvas.get_tk_widget().pack(
            fill="both", expand=True, padx=14, pady=(4, 0),
        )

        barra_grafico = ctk.CTkFrame(
            fr_graf, fg_color="#F3F4F6", corner_radius=6, height=38,
        )
        barra_grafico.pack(fill="x", padx=14, pady=10)
        toolbar = NavigationToolbar2Tk(
            self.canvas, barra_grafico, pack_toolbar=False,
        )
        toolbar.update()
        toolbar.pack(side="left", padx=4, pady=2)

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
        if self._plot_dirty and self.winfo_viewable():
            self._atualizar_plot()
        self.after(150, self._loop_desenho)

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

