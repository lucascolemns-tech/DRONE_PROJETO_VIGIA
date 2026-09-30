COR_FUNDO        = "#FFFFFF"
COR_BORDA        = "#292525"
COR_BOTAO_HOVER  = "#D1D5DB"
COR_TEXTO        = "#000000"

COR_TAB_FUNDO    = "#FFFFFF"
COR_TAB_NAO_SEL  = "#E5E7EB"
COR_TAB_HOVER    = "#D1D5DB"
COR_TAB_SEL      = "#4B5563"
COR_TAB_SEL_HOV  = "#6B7280"
COR_TAB_TEXTO    = "#1F2937"

FONTE_BOTAO  = ("Arial Black", 20)
FONTE_LABEL  = ("Arial", 14)
FONTE_TITULO = ("Arial Black", 20)
FONTE_CONSOLE = ("Consolas", 12)
FONTE_SECAO  = ("Arial Black", 10)
FONTE_ROTULO = ("Arial Black", 10)
FONTE_VALOR  = ("Arial Black", 13)
FONTE_MOTOR  = ("Arial Black", 11)
FONTE_TELEMETRIA = ("Arial Black", 14)

def estilo_botao():
    return dict(
        fg_color=COR_FUNDO,
        border_width=3,
        border_color=COR_BORDA,
        hover_color=COR_BOTAO_HOVER,
        text_color=COR_TEXTO,
        corner_radius=6,
        height=60,
        font=FONTE_BOTAO,
    )
