import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import time
import threading
import os
from collections import deque
import serial
import struct

# Configura a comunicação serial
ser = serial.Serial('COM8', 115200, timeout=1)  # Ajuste 'COM8' conforme necessário

def ler_dados():
    if ser.in_waiting >= 7:  # Garante que há pelo menos 7 bytes no buffer
        dados = ser.read(7)  # Lê 7 bytes (1 cabeçalho + 6 dados)

        if dados[0] == 0xAA:  # Verifica se o cabeçalho está correto
            # Desempacota 3 valores inteiros (2 bytes cada)
            p, r, pos = struct.unpack('>hhh', dados[1:])

            # Converte os valores para float
            p = p / 100.0
            r = r / 100.0
            pos = pos / 100.0

            print(f"Pitch: {p:.2f}, Roll: {r:.2f}, Position: {pos:.2f}")
            return p, r, pos  # Retorna os três valores convertidos
        else:
            print("Cabeçalho incorreto, descartando pacote!")
            ser.reset_input_buffer()  # Limpa o buffer para evitar leituras erradas
    return None  # Retorna None se não houver leitura válida

class DynamicPlotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Gráficos Dinâmicos")

        # Variáveis para gravação
        self.recording = False
        self.paused = False
        self.data = []

        # Configurar interface
        self.create_widgets()
        tamanho_maximo = 1000000

        # Variáveis simuladas
        self.x_data = deque(maxlen=tamanho_maximo)
        self.y1_data = deque(maxlen=tamanho_maximo)
        self.y2_data = deque(maxlen=tamanho_maximo)
        self.y3_data = deque(maxlen=tamanho_maximo)
        self.gx_data = deque(maxlen=tamanho_maximo)  # Não está sendo usado
        self.gy_data = deque(maxlen=tamanho_maximo)  # Não está sendo usado
        self.gz_data = deque(maxlen=tamanho_maximo)  # Não está sendo usado

        # Threading lock para segurança
        self.lock = threading.Lock()

    def create_widgets(self):
        # Botões
        self.start_read = ttk.Button(self.root, text="START", command=self.start_read)
        self.start_read.grid(row=0, column=2, padx=5, pady=5)

        self.start_button = ttk.Button(self.root, text="Iniciar Gravação", command=self.start_recording)
        self.start_button.grid(row=0, column=0, padx=5, pady=5)

        self.pause_button = ttk.Button(self.root, text="Pausar/Retomar", command=self.pause_recording)
        self.pause_button.grid(row=0, column=1, padx=5, pady=5)

        self.stop_button = ttk.Button(self.root, text="Encerrar Gravação", command=self.stop_recording)
        self.stop_button.grid(row=1, column=2, padx=5, pady=5)

        # Figura e gráficos
        self.fig, self.axes = plt.subplots(2, 2, figsize=(10, 8))
        self.fig.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, columnspan=2)

        # Configurar eixos
        self.ax1, self.ax2, self.ax3, self.ax4 = self.axes.flatten()
        self.ax1.set_title("X")
        self.ax2.set_title("Y")
        self.ax3.set_title("Z")
        self.ax4.set_title("Giroscópio")

    def start_read(self):
        ser.write(b"R")
        print("Comando R enviado ao transmissor")

    def start_recording(self):
        ser.write(b"P")  # Envia o comando "START" para o transmissor
        print("P Enviado")
        self.recording = True
        self.paused = False
        threading.Thread(target=self.update_data, daemon=True).start()  # Thread para atualizar dados

    def pause_recording(self):
        self.paused = not self.paused  # Alterna entre pausado e retomado
        if self.paused:
            ser.write(b"R")
            print("Grafico Pausado")
        else:
            ser.write(b"P")
            print("Grafico Retomado")

    def stop_recording(self):
        self.recording = False
        self.save_data()

    def update_data(self):
        cont = 0
        while self.recording:
            if not self.paused:
                leitura = ler_dados()
                if leitura:
                    p, r, pos = leitura

                    with self.lock:  # Garante acesso seguro aos dados
                        self.x_data.append(cont)
                        self.y1_data.append(p)
                        self.y2_data.append(r)
                        self.y3_data.append(pos)
                        cont += 1

                    self.update_plots()
            time.sleep(0.5)  # Ajuste o intervalo conforme necessário

    def update_plots(self):
        with self.lock:  # Garante acesso seguro aos dados
            if self.x_data:
                self.ax1.clear()
                self.ax1.plot(self.x_data, self.y1_data, label="Pitch", color="blue")
                self.ax1.set_title("X")

                self.ax2.clear()
                self.ax2.plot(self.x_data, self.y2_data, label="Roll", color="green")
                self.ax2.set_title("Y")

                self.ax3.clear()
                self.ax3.plot(self.x_data, self.y3_data, label="Position", color="red")
                self.ax3.set_title("Z")

                # Se gx_data, gy_data, gz_data forem usados, adicione-os aqui
                self.ax4.clear()
                self.ax4.set_title("Giroscópio")
                #self.ax4.legend()

                self.canvas.draw()

    def save_data(self):
        dataID = random.randint(999, 9999)
        file_name = f'dados_gravados_{dataID}.txt'

        if self.x_data:
            folder_path = "C:\\Users\\bruno\\Desktop\\Solution\\TOUSANDOESSES\\painel_inclinometro\\dados"
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)

            file_path = os.path.join(folder_path, file_name)
            with open(file_path, "w") as file:
                file.write("Timestamp, Pitch, Roll, Position\n")
                for t, y1, y2, y3 in zip(self.x_data, self.y1_data, self.y2_data, self.y3_data):
                    file.write(f"{t}, {y1:.2f}, {y2:.2f}, {y3:.2f}\n")

            print(f"Dados salvos em {file_path}")

# Criar aplicação
root = tk.Tk()
app = DynamicPlotApp(root)
root.mainloop()
