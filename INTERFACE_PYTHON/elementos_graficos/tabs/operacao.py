import customtkinter as ctk
from ..tema import estilo_botao
from ..painel_telemetria import PainelTelemetria

class TabOperacao(ctk.CTkFrame):
    def __init__(self, master, app):
        super().__init__(master, fg_color="transparent")
        self.app = app

        # guarda o estado de cada botão que pode ser ligado/desligado
        self._habilitados = {
            "conectar":  True,     # começa ativo
            "manual":    False,
            "teste":     False,
            "calibrar":  False,
            "finalizar": False,
            "camera":    True,
        }

        #coluna esquerda: botões
        # width/height + pack_propagate(False) → moldura estica igual ao PID,
        # independente de quantos botões tem dentro
        esq = ctk.CTkFrame(self, corner_radius=8, fg_color="#FFFFFF",
                            border_width=3, border_color="#979DA2",
                            width=200, height=650)
        esq.pack(fill="both", expand=True, padx=15, pady=15, side="left")
        esq.pack_propagate(False)

        #coluna direita: console
        dir_ = ctk.CTkFrame(self, width=550, height=650, corner_radius=8,fg_color="#FFFFFF", border_width=3, border_color="#979DA2")
        dir_.pack(fill="both", expand=True, padx=15, pady=15, side="right")

        self.painel = PainelTelemetria(dir_)
        self.painel.pack(fill="both", expand=True)

        e = estilo_botao()
        # **e não demonstra erro no IntelliSense mas funciona perfeitamente, puxa os atributos do botão
        self.btn_conectar = ctk.CTkButton(esq, text="estabelecer comm", command=self._clicou_conectar, **e)
        self.btn_manual = ctk.CTkButton(esq, text="controle manual", command=self._clicou_manual, **e)
        self.btn_teste = ctk.CTkButton(esq, text="teste motores", command=self._clicou_teste, **e)
        self.btn_manual.configure(state="disabled")
        self.btn_teste.configure(state="disabled")
        self.btn_calibrar = ctk.CTkButton(esq, text="calibrar bússola",
                                           command=self._clicou_calibrar, **e)
        self.btn_calibrar.configure(state="disabled")
        self.btn_finalizar = ctk.CTkButton(esq, text="finalizar comm", command=self._clicou_finalizar, **e)
        self.btn_camera = ctk.CTkButton(esq, text="exibir camera", command=self._clicou_camera, **e)

        #implementarpara cada botão
        for b in (self.btn_conectar, self.btn_manual, self.btn_teste, self.btn_calibrar,
                  self.btn_finalizar, self.btn_camera):
            b.pack(fill="x")

        self._atualizar_aparencia()

    #funções botões
    def _clicou_conectar(self):
        if self._habilitados["conectar"]:
            self.app.iniciar_comunicacao()

    def _clicou_manual(self):
        if self._habilitados["manual"]:
            pass   # sem função no original

    def _clicou_teste(self):
        if self._habilitados["teste"]:
            pass   # sem função no original

    def _clicou_calibrar(self):
        if self._habilitados["calibrar"]:
            self.app.iniciar_calibracao_magnetometro()

    def _clicou_finalizar(self):
        if self._habilitados["finalizar"]:
            self.app.finalizar_comunicacao()

    def _clicou_camera(self):
        if self._habilitados["camera"]:
            self.app.abrir_camera()

    #atualizar estados e aparência
    def _atualizar_aparencia(self):
        mapa = {
            "conectar":  self.btn_conectar,
            "manual":    self.btn_manual,
            "teste":     self.btn_teste,
            "calibrar":  self.btn_calibrar,
            "finalizar": self.btn_finalizar,
            "camera":    self.btn_camera,
        }

        for chave, botao in mapa.items():
            if self._habilitados[chave]:
                botao.configure(text_color="#000000",
                                hover_color="#D1D5DB")
            else:
                botao.configure(text_color="#9CA3AF",
                                hover_color="#FFFFFF")

    def atualizar_estado(self, conectado: bool):
        self._habilitados["conectar"]  = not conectado
        self._habilitados["manual"]    = False
        self._habilitados["teste"]     = False
        self._habilitados["calibrar"]  = conectado
        self._habilitados["finalizar"] = conectado
        self.btn_calibrar.configure(state="normal" if conectado else "disabled")
        # btn_camera permanece sempre habilitado, igual ao monolítico
        self._atualizar_aparencia()

    def definir_calibracao_pendente(self, pendente):
        self.btn_calibrar.configure(state="disabled" if pendente or not self._habilitados["calibrar"] else "normal")
