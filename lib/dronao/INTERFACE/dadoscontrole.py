import subprocess
import time

IP = "192.168.43.201"
PORT = 1244

MAX_ROLL = 30.0
MAX_PITCH = 30.0
MAX_YAW = 45.0
THROTTLE = 100.0

controle = r"C:\Users\lucas\Desktop\META_DRONE\INTERFACE_PYTHON\controle.exe"

def mapear(valor, entrada_min, entrada_max, saida_min, saida_max):
    return (valor - entrada_min) * (saida_max - saida_min) / (entrada_max - entrada_min) + saida_min

def main():

    try:
        processo = subprocess.Popen(
            [controle],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            text=True
            )   

        print(f"Processo {controle} iniciado com sucesso.")

    except FileNotFoundError:
        print(f"Erro: O arquivo '{controle}' não foi encontrado.")
        return

    except Exception as e:
        print(f"Ocorreu um erro ao iniciar o processo: {e}")
        return

    print("Aguardando dados do controle...\n")
    print("Pressione Ctrl+C para encerrar o programa.\n")

    try:
        if processo.stdout is None:
            print("ERRO")
            return

        for linha in processo.stdout:

            linha = linha.strip() #remove espaços em branco no inicio e no final da linha

            if not linha:
                continue #ignora espaços em branco

            partes = linha.split(",") #retira os valores separados por vírgula

            if len(partes) != 4:
                print(f"Formato de dados inválido: {linha}")
                continue #mapeia os 4 valores fornecidos pelo controle.exe

            try:
                ry = float(partes[3])
                rx = float(partes[2])
                ly = float(partes[1])
                lx = float(partes[0])

            except ValueError:
                print(f"Erro ao converter dados para float: {linha}")
                continue

            # Limites
            roll = mapear(lx, -1000, 1000, -MAX_ROLL, MAX_ROLL)
            pitch = mapear(ly, -1000, 1000, -MAX_PITCH, MAX_PITCH)
            yaw = mapear(rx, -1000, 1000, -MAX_YAW, MAX_YAW)
            #throttle é a aceleração 0% é desligado e 100% é ligado, 50% é pairar
            throttle = mapear(ry, -1000, 1000, 0, 100) 
            
            print(
                f"Roll: {roll:.2f} | "
                f"Pitch: {pitch:.2f} | "
                f"Yaw: {yaw:.2f} | "
                f"Throttle: {throttle:.2f}"
            )
            
    except KeyboardInterrupt:
        print("\nPrograma encerrado pelo usuário.") #ctrl+c 

    finally:
        processo.terminate()
        processo.wait()
        print(f"Processo {controle} encerrado.")

#executa o main() quando o arquivo é executado diretamente, mas não quando é importado
#logo funciona localmente :)
if __name__ == "__main__":
    main()