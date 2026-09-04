#lembretes para os testes de hoje: instalar pc do Samuel pip install customtkinter pillow
#alterar o IP, linha 14 ,se necessário alterar a porta
#alterar os caminhos do incone vigia edo .exe do controle, pq ta em relação ao meu pc

import customtkinter as ctk
import tkinter as tk
import socket
import subprocess
import threading
import time

#classe para comunicação WiFi    
class ESPCom:
    def __init__(self, ip="192.168.18.201", porta=1244):
        self.ip = ip
        self.porta = porta
        self.sock = None
        self.buffer = ""  

    def conectar(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # CORREÇÃO: Aumentei o timeout para 2 segundos para dar tempo do ESP32 responder
            self.sock.settimeout(2.0) 
            self.sock.connect((self.ip, self.porta))
            print(f"SUCESSO: Conectado em {self.ip}:{self.porta}")
            return True
        except socket.timeout:
            print(f"ERRO: Timeout ao conectar em {self.ip}:{self.porta}. O ESP32 está ligado, mas o servidor não respondeu.")
            self.sock = None
            return False
        except ConnectionRefusedError:
            print(f"ERRO: Conexão recusada em {self.ip}:{self.porta}. O IP está certo, mas o ESP32 NÃO iniciou o servidor TCP.")
            self.sock = None
            return False
        except OSError as e:
            print(f"ERRO DE REDE: {e}")
            self.sock = None
            return False
        except Exception as e:
            print(f"ERRO GERAL: {e}")
            self.sock = None
            return False

    def enviar_setpoint(self, roll, pitch, yaw, throttle):
        if self.sock is None:
            return False

        try:
            msg = f"{roll:.2f},{pitch:.2f},{yaw:.2f},{throttle:.2f}\n"
            self.sock.sendall(msg.encode())
            return True
        except:
            return False

    def receber_dados(self):
        if self.sock is None:
            return None
        try:
            dados = self.sock.recv(1024)
            if dados:
                self.buffer += dados.decode("UTF-8")
                
                if "\n" in self.buffer:
                    linha, self.buffer = self.buffer.split("\n", 1)
                    
                    if linha.strip():
                        return linha
                    
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Erro ao receber: {e}")
        return None
    
    def fechar(self):
        if self.sock:
            self.sock.close()
            self.sock = None

#configs interface
ctk.set_appearance_mode("Light")
ctk.set_default_color_theme("green")
largura_janela = 1000
altura_janela = 500

window = ctk.CTk()
window.geometry(f"{largura_janela}x{altura_janela}")
window.title("INTERFACE DRONE")

#icone do projeto VIGIA
window.wm_iconbitmap(r"C:\Users\lucas\Desktop\META_DRONE\INTERFACE_PYTHON\  VIGIA.ico")

esp = ESPCom()

# Centralizar janela
largura_tela = window.winfo_screenwidth()
altura_tela = window.winfo_screenheight()
pos_x = (largura_tela - largura_janela) // 2
pos_y = (altura_tela - altura_janela) // 2
window.geometry(f"{largura_janela}x{altura_janela}+{pos_x}+{pos_y}")

# Barra de Tarefas
barra_tarefas = ctk.CTkTabview(master=window, 
                            height=15, corner_radius=8, fg_color="#FFFFFF", 
                            segmented_button_fg_color="#D1D5DB",  
                            segmented_button_selected_color="#4B5563",
                            segmented_button_selected_hover_color="#6B7280",  
                            segmented_button_unselected_color="#E5E7EB",  
                            segmented_button_unselected_hover_color="#D1D5DB",  
                            text_color="#1F2937", anchor="w", state="normal")
    
tab1 = barra_tarefas.add("FUNCIONAMENTO")
tab2 = barra_tarefas.add("REGULAGEM PID")
tab3 = barra_tarefas.add("DIAGNOSTICO")
barra_tarefas.pack(fill="x", side="top", pady=10) 

# Construção da janela1
abas = ctk.CTkFrame(master=tab1, corner_radius=8, fg_color="#ffffff", border_width=3)
abas.pack(fill="both", expand=True, padx=15, pady=15, side="left")

retangulo_arredondado = ctk.CTkFrame(master=tab1, width=550, height=1000, corner_radius=8, fg_color="#FFFFFF", border_width=3)
retangulo_arredondado.pack(fill="both", expand=True, padx=15, pady=15, side="right")

# Console
console_texto = ctk.CTkTextbox(master=retangulo_arredondado, width=400, height=700, corner_radius=8, border_width=3, font=("Consolas", 12), border_color="#292525")
console_texto.pack(fill="both", expand=True, padx=10, pady=10)

def escrever_log(mensagem):
    console_texto.insert("end", mensagem + "\n")
    console_texto.see("end")

# Variáveis globais para as threads
controle_ativo = False
processo_controle = None

# Função de mapeamento (usada pela thread do controle)
def mapear(valor, entrada_min, entrada_max, saida_min, saida_max):
    return (valor - entrada_min) * (saida_max - saida_min) / (entrada_max - entrada_min) + saida_min

#ler controle
def thread_controle():
    global processo_controle, controle_ativo
    caminho_exe = r"C:\Users\lucas\Desktop\META_DRONE\INTERFACE_PYTHON\controle.exe"
    try:
        processo_controle = subprocess.Popen([caminho_exe], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1, text=True)
        escrever_log(f"Controle iniciado: {caminho_exe}")
    except:
        escrever_log("Erro: não foi possível abrir controle.exe")
        controle_ativo = False
        return

    while controle_ativo:
        try:
            if processo_controle.stdout is None: break
            linha = processo_controle.stdout.readline()
            if not linha: break
            
            partes = linha.strip().split(",")
            if len(partes) != 4: continue
            
            lx = float(partes[0])
            ly = float(partes[1])
            rx = float(partes[2])
            ry = float(partes[3])
            
            MAX_ROLL, MAX_PITCH, MAX_YAW = 20.0, 20.0, 30.0
            roll = mapear(lx, -1000, 1000, -MAX_ROLL, MAX_ROLL)
            pitch = mapear(ly, -1000, 1000, -MAX_PITCH, MAX_PITCH)
            yaw = mapear(rx, -1000, 1000, -MAX_YAW, MAX_YAW)
            throttle = mapear(ry, -1000, 1000, 0, 100) 

            print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Throttle: {throttle:.2f}")
            esp.enviar_setpoint(roll, pitch, yaw, throttle)
            
        except Exception as e:
            escrever_log(f"Erro na leitura do controle: {e}")
            time.sleep(0.1)

#função para dados do ESP32
def processar_dados_esp(dados):
    try:
        partes = dados.strip().split(",")
        
        # Verifica se recebeu os 10 valores esperados
        if len(partes) >= 10:
            ang_x = float(partes[0])
            ang_y = float(partes[1])
            ang_z = float(partes[2])
            alt = float(partes[3])
            vel = float(partes[4])
            temp = float(partes[5])
            m1 = float(partes[6])
            m2 = float(partes[7])
            m3 = float(partes[8])
            m4 = float(partes[9])

            msg = (
                f"Ângulos -> X:{ang_x:.2f} | Y:{ang_y:.2f} | Z:{ang_z:.2f} | "
                f"Kalman -> Alt:{alt:.3f}m | Vel:{vel:.3f}m/s | "
                f"Barômetro -> Temp:{temp:.2f}°C | "
                f"Motores -> M1:{m1:.0f} | M2:{m2:.0f} | M3:{m3:.0f} | M4:{m4:.0f}"
            )
            escrever_log(f"ESP32: {msg}")

        else:
            escrever_log(f"ESP32 (formato não reconhecido): {dados}")

    except ValueError:
        escrever_log(f"Erro ao converter dados do ESP32: {dados}")

#receber os dados e mostrar no monitor que tem no nosso programa
def thread_receptor():
    contador = 0
    global controle_ativo
    while controle_ativo:
        dados = esp.receber_dados()
        if dados:
            processar_dados_esp(dados)
            contador += 1
            if contador % 5 == 0:
                escrever_log(f"Pacote {contador} recebido")
        time.sleep(0.05)

# Botões
def permissao1():
    global controle_ativo
    btn1.configure(state="disabled")
    btn2.configure(state="normal")
    btn3.configure(state="normal")
    btn4.configure(state="normal")
    btn11.configure(state="normal")
    btn22.configure(state="normal")
    
    if esp.conectar():
        controle_ativo = True
        escrever_log("WiFi Conectado! Iniciando controle...")
        threading.Thread(target=thread_controle, daemon=True).start()
        threading.Thread(target=thread_receptor, daemon=True).start()
    else:
        escrever_log("ERRO: Não foi possível conectar ao ESP32. Verifique o IP.")

def permissao2():
    global controle_ativo
    controle_ativo = False
    if processo_controle:
        processo_controle.terminate()
    esp.fechar()
    
    btn1.configure(state="normal")
    btn2.configure(state="disabled")
    btn3.configure(state="disabled")
    btn4.configure(state="disabled")
    btn11.configure(state="disabled")
    btn22.configure(state="disabled")
    escrever_log("Comunicação finalizada.")

btn1 = ctk.CTkButton(master=abas, fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#D1D5DB", text_color="black", text="estabelecer comm", corner_radius=6, height=60, font=("Arial Black", 20), state="normal", command=permissao1)
btn2 = ctk.CTkButton(master=abas, fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#D1D5DB", text_color="black", text="controle manual", corner_radius=6, height=60, font=("Arial Black", 20), state="disabled")
btn3 = ctk.CTkButton(master=abas, fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#D1D5DB", text_color="black", text="teste motores", corner_radius=6, height=60, font=("Arial Black", 20), state="disabled")
btn4 = ctk.CTkButton(master=abas, fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#D1D5DB", text_color="black", text="finalizar comm", corner_radius=6, height=60, font=("Arial Black", 20), state="disabled", command=permissao2)

btn1.pack(fill="x")
btn2.pack(fill="x")
btn3.pack(fill="x")
btn4.pack(fill="x")

abas2 = ctk.CTkFrame(master=tab2, corner_radius=8, fg_color="#FFFFFF", border_width=3)
abas2.pack(fill="both", expand=True, padx=15, pady=15, side="left")
retangulo_arredondado2 = ctk.CTkFrame(master=tab2, width=550, height=1000, corner_radius=8, fg_color="#FFFFFF", border_width=3, border_color="#292525")
retangulo_arredondado2.pack(fill="both", expand=True, padx=15, pady=15, side="right")

def abrir_janela_parametros():
    janela_parametros = ctk.CTkToplevel(window)
    janela_parametros.title("PID")
    janela_parametros.geometry("300x300")
    janela_parametros.grab_set()
    janela_parametros.overrideredirect(False)
    janela_parametros.attributes("-toolwindow", True)

    ctk.CTkLabel(janela_parametros, text="KP (Proporcional):", font=("Arial", 14)).pack(pady=5)
    entry_kp = ctk.CTkEntry(janela_parametros, width=200, placeholder_text="Ex: 1.0", border_color="#292525")
    entry_kp.pack(pady=5)
    entry_kp.insert(0, "1.0")

    ctk.CTkLabel(janela_parametros, text="KI (Integral):", font=("Arial", 14)).pack(pady=5)
    entry_ki = ctk.CTkEntry(janela_parametros, width=200, placeholder_text="Ex: 0.1")
    entry_ki.pack(pady=5)
    entry_ki.insert(0, "0.1")

    ctk.CTkLabel(janela_parametros, text="KD (Derivativo):", font=("Arial", 14)).pack(pady=5)
    entry_kd = ctk.CTkEntry(janela_parametros, width=200, placeholder_text="Ex: 0.01")
    entry_kd.pack(pady=5)
    entry_kd.insert(0, "0.01")

    def enviar_pid():
        try:
            kp = float(entry_kp.get().replace(",", "."))
            ki = float(entry_ki.get().replace(",", "."))
            kd = float(entry_kd.get().replace(",", "."))

            if esp.sock:
                esp.sock.sendall(f"{kp};{ki};{kd}\n".encode())
                escrever_log(f"PID Enviado: Kp={kp}, Ki={ki}, Kd={kd}")
                janela_parametros.destroy()
        except:
            ctk.CTkLabel(janela_parametros, text="Erro! Digite apenas números.", text_color="red").pack(pady=5)

    ctk.CTkButton(janela_parametros, text="ENVIAR", corner_radius=0, text_color="black", font=("Arial Black", 16), fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#ffffff", command=enviar_pid).pack(fill="x", side="bottom")

btn11 = ctk.CTkButton(master=abas2, fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#D1D5DB", text_color="black", text="def gráficos", corner_radius=6, height=60, font=("Arial Black", 20), state="disabled")
btn22 = ctk.CTkButton(master=abas2, fg_color="#FFFFFF", border_width=3, border_color="#292525", hover_color="#D1D5DB", text_color="black", text="def parametros", corner_radius=6, height=60, font=("Arial Black", 20), state="disabled", command=abrir_janela_parametros)

btn11.pack(fill="x")
btn22.pack(fill="x")

window.mainloop()