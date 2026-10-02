import os
import queue
import threading
import time
import customtkinter as ctk

from config import (
    ESP_IP, ESP_PORT, SOCKET_TIMEOUT, CONTROLE_EXE, CAMERA_URL,
    MAX_ROLL, MAX_PITCH, MAX_YAW, MAX_THROTTLE, THROTTLE_ARM_AXIS,
    WINDOW_W, WINDOW_H, WINDOW_TITULO, ICONE_IMG,
)
from nucleo import ESPCom, ControleHardware, processar_dados_esp
from .tema import (COR_TAB_FUNDO, COR_TAB_NAO_SEL, COR_TAB_HOVER,
                    COR_TAB_SEL, COR_TAB_SEL_HOV, COR_TAB_TEXTO)
from .tabs import TabOperacao, TabPID
from .janelaPID import abrir_janela_pid
from .janelaCAM import abrir_janela_camera
from .janelaGRAFICO import JanelaGraficos


class App:
    def __init__(self):
        self.esp = ESPCom(ESP_IP, ESP_PORT, SOCKET_TIMEOUT)
        self.controle = ControleHardware(CONTROLE_EXE, MAX_ROLL, MAX_PITCH, MAX_YAW,
                                         max_throttle=MAX_THROTTLE,
                                         throttle_arm_axis=THROTTLE_ARM_AXIS)
        self.ativo = False
        self._fila_telemetria = queue.Queue(maxsize=32)

        # referência à janela de gráficos (None quando fechada)
        self._janela_graficos = None

        ctk.set_appearance_mode("Light")
        ctk.set_default_color_theme("green")

        self.window = ctk.CTk()
        self.window.title(WINDOW_TITULO)
        self.window.protocol("WM_DELETE_WINDOW", self._fechar_aplicacao)
        self._ultima_telemetria = 0.0
        self._telemetria_limpa = True
        self._calibracao_pendente = False
        self._tempo_pedido_calibracao = 0.0
        self._status_mag_no_pedido = 0
        self._ultimo_status_mag = 0
        self._centralizar()
        self._aplicar_icone()

        barra = ctk.CTkTabview(
            master=self.window, height=15, corner_radius=8,
            fg_color=COR_TAB_FUNDO,
            segmented_button_fg_color=COR_TAB_NAO_SEL,
            segmented_button_selected_color=COR_TAB_SEL,
            segmented_button_selected_hover_color=COR_TAB_SEL_HOV,
            segmented_button_unselected_color=COR_TAB_NAO_SEL,
            segmented_button_unselected_hover_color=COR_TAB_HOVER,
            text_color=COR_TAB_TEXTO, anchor="w", state="normal",
        )
        tab1 = barra.add("FUNCIONAMENTO")
        tab2 = barra.add("REGULAGEM PID")
        barra.pack(fill="x", side="top", pady=10)

        self.tab_func = TabOperacao(tab1, self)
        self.tab_func.pack(fill="both", expand=True)

        self.tab_pid = TabPID(tab2, self)
        self.tab_pid.pack(fill="both", expand=True)
        self.window.after(50, self._processar_fila_telemetria)

    #função para a janela principal abrir exatamente no centro da sua tela do monitor
    def _centralizar(self):
        lx = self.window.winfo_screenwidth()
        ly = self.window.winfo_screenheight()
        largura = min(WINDOW_W, max(640, lx - 40))
        altura = min(WINDOW_H, max(480, ly - 80))
        self.window.geometry(
            f"{largura}x{altura}+{(lx - largura)//2}+{(ly - altura)//2}"
        )

    def _aplicar_icone(self):
        if os.path.isfile(ICONE_IMG):
            try:
                self.window.iconbitmap(ICONE_IMG)
            except Exception:
                pass

    #comunicação Wifi
    def iniciar_comunicacao(self):
        if self.ativo:
            return
        if not self.esp.conectar(forcar=True):
            print("ERRO: não foi possível conectar ao ESP32.")
            return
        ok, msg = self.controle.iniciar()
        if not ok:
            print(f"ERRO: {msg}")
            self.esp.fechar()
            return
        self.ativo = True
        print("WiFi conectado. Iniciando controle...")
        threading.Thread(target=self._loop_controle,  daemon=True).start()
        threading.Thread(target=self._loop_receptor, daemon=True).start()
        self._atualizar_botoes(True)

    def finalizar_comunicacao(self):
        self.ativo = False
        self._calibracao_pendente = False
        self.controle.fechar()
        self.esp.fechar()
        print("Comunicação finalizada.")
        self._atualizar_botoes(False)

    def _fechar_aplicacao(self):
        self.ativo = False
        self._calibracao_pendente = False
        self.controle.fechar()
        self.esp.fechar()
        self.window.destroy()

    def iniciar_calibracao_magnetometro(self):
        if not self.ativo or self._calibracao_pendente:
            return

        self._calibracao_pendente = True
        self._tempo_pedido_calibracao = time.monotonic()
        self._status_mag_no_pedido = self._ultimo_status_mag
        self.tab_func.definir_calibracao_pendente(True)
        threading.Thread(target=self._enviar_pedido_calibracao, daemon=True).start()

    def _enviar_pedido_calibracao(self):
        sucesso = self.esp.solicitar_calibracao_magnetometro()
        try:
            self.window.after(0, lambda: self._calibracao_enviada(sucesso))
        except Exception:
            pass

    def _calibracao_enviada(self, sucesso):
        if not sucesso:
            self._calibracao_pendente = False
            self.tab_func.definir_calibracao_pendente(False)

    #comunicação camera
    def abrir_camera(self):
        abrir_janela_camera(self.window, CAMERA_URL)

    def abrir_pid(self):
        abrir_janela_pid(self.window, self.esp)

    def abrir_graficos(self):
        if self._janela_graficos is not None and self._janela_graficos.winfo_exists():
            self._janela_graficos.lift()
            self._janela_graficos.focus()
            return
        self._janela_graficos = JanelaGraficos(self.window, self.esp)

    #comunicação controle xbox
    def _loop_controle(self):
        ultimo_envio = 0.0
        while self.ativo:
            cmd = self.controle.ler_comandos()
            if cmd is None:
                time.sleep(0.01)
                continue

            agora = time.monotonic()
            if agora - ultimo_envio < 0.02:
                continue

            roll, pitch, yaw, throttle = cmd
            if self.esp.enviar_setpoint(roll, pitch, yaw, throttle):
                ultimo_envio = agora
            else:
                time.sleep(0.01)

    #receber dados do esp32 e chamar respectivas funções
    def _loop_receptor(self):
        while self.ativo:
            dados = self.esp.receber_dados()
            if dados:
                msg, valores = processar_dados_esp(dados)
                if valores is not None:
                    self._ultima_telemetria = time.monotonic()
                    self._telemetria_limpa = False
                    try:
                        self._fila_telemetria.put_nowait((valores, time.monotonic()))
                    except queue.Full:
                        try:
                            self._fila_telemetria.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            self._fila_telemetria.put_nowait((valores, time.monotonic()))
                        except queue.Full:
                            pass
                else:
                    print(f"ESP32: {msg}")
            time.sleep(0.05)

    def _processar_fila_telemetria(self):
        if not self.window.winfo_exists():
            return

        dados = None
        amostras_grafico = []
        while True:
            try:
                valores, instante = self._fila_telemetria.get_nowait()
                dados = valores
                if valores["link_ok"] and valores["sistema_pronto"]:
                    amostras_grafico.append((valores["ang_x"], instante))
            except queue.Empty:
                break

        if dados is not None:
            self.tab_func.painel.atualizar(dados)
            self._ultimo_status_mag = dados["mag_status"]
            self.tab_func.definir_magnetometro_ativo(dados["mag_status"] not in (4, 5))
            if self._calibracao_pendente and (
                    dados["mag_status"] == 1 or
                    (dados["mag_status"] in (3, 4) and dados["mag_status"] != self._status_mag_no_pedido) or
                    time.monotonic() - self._tempo_pedido_calibracao > 3.0):
                self._calibracao_pendente = False
            self.tab_func.definir_calibracao_pendente(
                self._calibracao_pendente or dados["mag_status"] == 1)

        if (self._janela_graficos is not None and self._janela_graficos.winfo_exists()):
            for angulo, instante in amostras_grafico:
                self._janela_graficos.adicionar_dado(angulo, instante)

        if (not self._telemetria_limpa and self._ultima_telemetria
                and time.monotonic() - self._ultima_telemetria > 0.5):
            self.tab_func.painel.limpar("TELEMETRIA DESATUALIZADA")
            self._telemetria_limpa = True

        self.window.after(50, self._processar_fila_telemetria)

    def _atualizar_botoes(self, conectado):
        self.tab_func.atualizar_estado(conectado)
        self.tab_pid.atualizar_estado(conectado)

    def run(self):
        self.window.mainloop()
