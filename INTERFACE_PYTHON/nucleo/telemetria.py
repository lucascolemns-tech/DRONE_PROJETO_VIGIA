import math
import re


_FIM_E_INICIO_COLADOS = re.compile(r"(-?\d+\.\d)(-?\d+\.\d{2})")


def _recuperar_frames_colados(partes, manter_todos=False):
    """Separa frames de 16 campos quando faltou o LF entre dois frames."""
    recuperadas = []
    for parte in partes:
        if len(recuperadas) % 16 == 15:
            junção = _FIM_E_INICIO_COLADOS.fullmatch(parte)
            if junção:
                recuperadas.extend(junção.groups())
                continue
        recuperadas.append(parte)

    if manter_todos:
        return recuperadas
    if len(recuperadas) >= 32 and len(recuperadas) % 16 == 0:
        # A interface usa o mais recente; o receptor TCP pode precisar enfileirar todos.
        return recuperadas[-16:]
    return recuperadas


def processar_dados_esp(dados_brutos):
    try:
        partes = dados_brutos.strip().split(",")
        if len(partes) > 16:
            partes = _recuperar_frames_colados(partes)
        if len(partes) not in (10, 16):
            return f"Formato não reconhecido: {dados_brutos}", None

        valores = [float(parte) for parte in partes]
        if not all(math.isfinite(valor) for valor in valores):
            return f"Telemetria contém valor inválido: {dados_brutos}", None

        if len(valores) == 10:
            # Compatibilidade com o sketch ESP32 antigo: 10 campos, sem estados dos sensores.
            valores.extend((-1.0, 0.0, 1.0, 1.0, 5.0, 0.0))

        if (valores[12] not in (0.0, 1.0) or valores[13] not in (0.0, 1.0) or
                valores[14] not in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0) or
                not 0.0 <= valores[15] <= 100.0):
            return f"Indicador de telemetria inválido: {dados_brutos}", None

        dados = {
            "sistema_pronto": bool(valores[12]),
            "link_ok": bool(valores[13]),
            "ang_x": valores[0],
            "ang_y": valores[1],
            "ang_z": valores[2],
            "alt":   valores[3],
            "vel":   valores[4],
            "temp":  valores[5],
            "m1":    valores[6],
            "m2":    valores[7],
            "m3":    valores[8],
            "m4":    valores[9],
            "tensao": valores[10],
            "gps_alt": valores[11],
            "mag_status": int(valores[14]),
            "mag_progress": valores[15],
        }

        if not dados["link_ok"]:
            return "STM32 sem telemetria válida", dados

        msg = (
            f"Ângulos -> X:{dados['ang_x']:.2f} | Y:{dados['ang_y']:.2f} | Z:{dados['ang_z']:.2f} | "
            f"Kalman -> Alt:{dados['alt']:.3f}m | Vel:{dados['vel']:.3f}m/s | "
            f"Barômetro -> Temp:{dados['temp']:.2f}°C | "
            f"Motores -> M1:{dados['m1']:.0f} | M2:{dados['m2']:.0f} | M3:{dados['m3']:.0f} | M4:{dados['m4']:.0f}"
        )

        return msg, dados

    except (ValueError, OverflowError):
        return f"conversão invalida: {dados_brutos}", None
