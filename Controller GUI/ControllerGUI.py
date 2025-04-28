import sys
import csv
import serial
import time
import threading
import pandas as pd
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QFileDialog
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# GUI VARIABLES ======================================================================================================
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800
data_file = "dataLog_test1.csv" #filepath to .csv file where data will be stored


V_inst = 0
I_inst = 0
P_inst = 0
currentState = 0
currentEnergy = 0


# BLUETOOTH VARIABLES ================================================================================================
port = 'COM8'
num_bytes = 21
baud_rate = 115200
bluetooth_serial = None
serial_lock = threading.Lock()
add_to_log = True
time_step = 0.5
running_time = 0

# ===================================================================================================================
# ===================================================================================================================
class LogViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Power Log Viewer")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.plot_button = QPushButton("Plot")
        self.plot_button.clicked.connect(self.plotPowerLog)
        self.canvas = FigureCanvas(plt.Figure(figsize=(8, 6)))
        
        layout = QVBoxLayout()
        layout.addWidget(self.plot_button)
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
    def plotPowerLog(self):
        print("Plotting Power Log")
        #TODO: implement
        # Open file picker
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(self, "Select a CSV file", "", "CSV Files (*.csv)")

        if file_path:
            # Read CSV
            df = pd.read_csv(file_path)

            # Parse 'Time' column as datetime
            df['Time'] = pd.to_datetime(df['Time'])

            # Set up plotting
            ax = self.canvas.figure.subplots()
            ax.clear()

            # Plot Voltage, Current, and Power on the same graph
            ax.plot(df['Time'], df['Voltage'], label='Voltage (V)', marker='o')
            ax.plot(df['Time'], df['Current'], label='Current (mA)', marker='x')
            ax.plot(df['Time'], df['Power'], label='Power mW)', marker='s')

            ax.set_title("Voltage, Current, Power Over Time")
            ax.set_xlabel("Time")
            ax.set_ylabel("Values")
            ax.legend()
            ax.grid(True)

            self.canvas.draw()
# ===================================================================================================================         
            
# ===================================================================================================================
class RunViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Run Viewer")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
        
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        
        self.start_button.clicked.connect(send_START)
        self.stop_button.clicked.connect(send_STOP)

        layout = QVBoxLayout()
        label = QLabel("This is the run viewer!")
        layout.addWidget(label)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        self.setLayout(layout)
    
    def update_progress_bars():
    # Update the progress bars based on the latest voltage and current
        voltage_progress.setValue(int((V_inst / 15.0) * 100))  # Set voltage as percentage (max 15V)
        current_progress.setValue(int((I_inst / 5.0) * 100))  # Set current as percentage (max 5A)  


# ==================================================================================================================
# ===== MAIN WINDOW ================================================================================================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ESP32 Bluetooth Control")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.begin_button = QPushButton("Begin New Run")
        self.logviewer_window_button = QPushButton("Open Power Log Viewer")

        # Connect buttons to functions
        self.begin_button.clicked.connect(self.begin_run)
        self.logviewer_window_button.clicked.connect(self.open_logviewer_window)

        layout = QVBoxLayout()
        # Add the buttons to the window
        layout.addWidget(self.begin_button)
        layout.addWidget(self.logviewer_window_button)

        self.setLayout(layout)

    def begin_run(self):
        print("Starting a new run.")
        #open the run window
        self.runviewer_window = RunViewer()
        self.runviewer_window.show()

    
    def open_logviewer_window(self):
        self.logviewer_window = LogViewer()
        self.logviewer_window.show()
        
# ==================================================================================================================
# GUI FUNCTIONS ====================================================================================================
def send_START():
    print("Sending START to vehicle.")
    send_message("Start")


def send_STOP():
    print("Sending STOP to vehicle.")
    send_message("Stop")
    
def update_K():
    print("Sending K to vehicle.")
    #TODO: pull desired parameter from GUI
    send_message("K=")

def send_message(command):
    with serial_lock:
        bluetooth_serial.write(command.encode('utf-8'))
        
    # if the connection is lost, break
    if not bluetooth_serial.is_open:
        print("Error: Serial port is not open.")
        return

        
# ==============================================================================
#=========================================================================================
""" Separate the numbers out from the response string"""
def parse_dataLog(response):
    values = response.split(":")

    #print(values)
    
    current = float(values[0])             # current in mA
    voltage = float(values[1])             # voltage in V
    power = current * voltage              # power in mW
    state = float(values[2])              # state enumeration

    

    energy = power * time_step

    global running_time

    print("TIME (s): " + str(running_time))
    print("Current (mA): " + str(current))
    print("Voltage (V): " + str(voltage))
    print("Power (mW): " + str(power))
    print("Energy (J):" + str(energy))

    log = [running_time, voltage, current, power, energy, state]

    if add_to_log:
        add_to_csv(data_file, log)
    
    running_time += time_step

""" Append row of collected data to .csv file"""
def add_to_csv(filepath, row_data):
    """Appends a list of values as a new row into the given CSV file."""
    with open(filepath, mode="a", newline="") as file:  # "a" = append mode
        writer = csv.writer(file)
        writer.writerow(row_data)

""" Reads power data logs from ESP32"""
def handle_Rx():
    print("Starting Rx thread.")
    global bluetooth_serial, serial_lock
    
    while 1:
        
        with serial_lock:
            response = bluetooth_serial.read(num_bytes)
        
        if not response:
            continue  
            
        response_str = str(response)[2:]        #strip out the first two characters "b'"
        response_str = response_str[:-1]        #strip out the last character too "'"
        print('====================')
        
        # parse the response
        parse_dataLog(response_str)


        # if the connection is lost, break
        if not bluetooth_serial.is_open:
            print("Error: Serial port is not open.")
            return
        
        

def start_Rx_thread():
    rx_thread = threading.Thread(target=handle_Rx, daemon=True) #run in the background
    rx_thread.start()
    
    


# Establish Bluetooth Connection =======================================================
try:
    #attempt to establish a serial connection
    #bluetooth_serial = serial.Serial(port, baud_rate)
    bluetooth_serial = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
    print(bluetooth_serial.name)
    print(f"Connected to port {port} at {baud_rate} baud.")

    if __name__ == "__main__":
        
        # If Bluetooth Connection is properly established, run the GUI
        app = QApplication([])
        window = MainWindow()
        window.show()
        sys.exit(app.exec())
        
        
        start_Rx_thread()
        
except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    # Close the serial port
    if 'bluetooth_serial' in locals() and bluetooth_serial.is_open:
        bluetooth_serial.close()
        print("Serial port closed")

      
