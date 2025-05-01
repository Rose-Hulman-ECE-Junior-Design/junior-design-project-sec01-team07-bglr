import sys
import csv
import serial
import time
import threading
import pandas as pd
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QRadioButton, QLabel, QPushButton, QLineEdit, QVBoxLayout, QFileDialog, QProgressBar, QGridLayout
from PyQt6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


# GUI VARIABLES ======================================================================================================
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800

data_file = "temp2.csv" #filepath to .csv file where data will be stored


V_inst = 7.223
I_inst = 302
P_inst = 2181.34
S_inst = "IDLE"
E_inst = 200
countdown = 90.0
timerDone = False
log_num = 0

speed = 0


state_map = {
    "0": "IDLE",
    "1": "DRIVING",
    "2": "CHARGING",
    "3": "COMPLETE"
}


# BLUETOOTH VARIABLES ================================================================================================
port = 'COM3'
num_bytes = 39
baud_rate = 115200
bluetooth_serial = None
serial_lock = threading.Lock()
add_to_log = False
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
        
        (tot_energy_spent, tot_energy_gained) = calculateEnergyUsage(data_file)
        
        self.energy_spent_label = QLabel("Total Energy Spent: " + str(tot_energy_spent) + " J")
        self.energy_gained_label = QLabel("Total Energy Gained: " + str(tot_energy_gained) + " J")
        
        layout = QVBoxLayout()
        layout.addWidget(self.plot_button)
        layout.addWidget(self.canvas)
        layout.addWidget(self.energy_spent_label)
        layout.addWidget(self.energy_gained_label)
        
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
# ===================================================================================================================
class RunViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Run Viewer")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
        
        # Buttons
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        
        all_buttons = [self.start_button, self.stop_button]
        
        for i in all_buttons:
            i.setMinimumSize(80, 200)  # (ancho, largo)
            
        self.start_button.clicked.connect(send_START)
        self.stop_button.clicked.connect(send_STOP)
        
        # Create radio buttons for options
        self.speed1_radio = QRadioButton("Speed 1", self)
        self.speed2_radio = QRadioButton("Speed 2", self)
        self.speed3_radio = QRadioButton("Speed 3", self)
        self.speed4_radio = QRadioButton("Speed 4", self)
        self.speed5_radio = QRadioButton("Speed 5", self)
        self.speed6_radio = QRadioButton("Speed 6", self)
        
        radio_buttons = [self.speed1_radio, self.speed2_radio, self.speed3_radio, self.speed4_radio, self.speed5_radio, self.speed6_radio]
        
        k = 1
        for i in radio_buttons:
            i.toggled.connect(lambda: self.update_speed(k))
            k = k + 1

                
        # Set up the timer to call update_run_graphics every 200ms
        timer = QTimer(self)
        timer.timeout.connect(self.update_run_graphics)  # Connect the update function
        timer.start(200)  # Call every 200ms
        
        # Labels
        self.instructions_label = QLabel("Insert RunViewer Instructions Here.")
        self.instructions_label.resize(50, 30)
        
        self.voltage_label = QLabel("Voltage: " + str(V_inst) + " V", self)
        self.current_label = QLabel("Current: " + str(I_inst) + " mA", self)
        self.power_label = QLabel("Power: " + str(P_inst)[:7] + " mW", self)
        self.energy_label = QLabel("Energy: " + str(E_inst) + " J", self)
        
        self.state_label = QLabel(S_inst)
        self.time_label = QLabel("Time Remaining: " + str(countdown))
        
        all_labels = [self.voltage_label, self.current_label, self.power_label, self.energy_label, self.state_label, self.time_label]
        for i in all_labels:
            i.setStyleSheet("QLabel { color: black; font-size: 20px; }")
            i.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Progress Bars
        self.voltage_progress = QProgressBar(self)      # VOLTAGE
        self.current_progress = QProgressBar(self)      # CURRENT
        self.power_progress = QProgressBar(self)        # POWER
        self.energy_progress = QProgressBar(self)        # ENERGY
        
        p_bars = [self.voltage_progress, self.current_progress, self.power_progress, self.energy_progress]
        for i in p_bars:
            # Set the style sheet to change the chunk color
            i.setRange(0, 100)     # Progress bar from 0 to 100%
            i.setTextVisible(True) # Show percentage
            i.setStyleSheet("""
                    QProgressBar {
                        border: 2px solid grey;
                        border-radius: 5px;
                        background-color: #F0F0F0;
                        text-align: center; /* Light gray background */
                    }
                    QProgressBar::chunk {
                        background-color: #00C853;  /* Bright green fill */
                        margin: 1px;
                        height: 20px;  /* optional: how wide each chunk is */
                    }
                """)
            i.setMinimumWidth(50)
            i.setMinimumHeight(200)
            i.setOrientation(Qt.Orientation.Vertical)
            i.setAlignment(Qt.AlignmentFlag.AlignCenter)
        

        # Adding to Layout - grid format
        layout = QGridLayout()
        layout.addWidget(self.instructions_label, 0, 0, 1, 6)   # add the instruction and timer label
        layout.addWidget(self.time_label, 1, 4, 1, 2)
        
        layout.addWidget(self.voltage_label, 4, 0, 1, 1)        # add the progress labels
        layout.addWidget(self.current_label, 4, 1, 1, 1)
        layout.addWidget(self.power_label, 4, 2, 1, 1)
        layout.addWidget(self.energy_label, 4, 3, 1, 1)
        layout.addWidget(self.state_label, 1, 0, 1, 3)
        
        layout.addWidget(self.voltage_progress, 2, 0, 2, 1)     # add the progress bars
        layout.addWidget(self.current_progress, 2, 1, 2, 1)
        layout.addWidget(self.power_progress, 2, 2, 2, 1)
        layout.addWidget(self.energy_progress, 2, 3, 2, 1)
        
        layout.addWidget(self.start_button, 2, 4, 1, 2)         # add the stop/start buttons
        layout.addWidget(self.stop_button, 3, 4, 1, 2)
        
        layout.addWidget(self.speed1_radio, 5, 0, 1, 1)         # add the speed selector radio buttons
        layout.addWidget(self.speed2_radio, 5, 1, 1, 1)
        layout.addWidget(self.speed3_radio, 5, 2, 1, 1)
        layout.addWidget(self.speed4_radio, 5, 3, 1, 1)
        layout.addWidget(self.speed5_radio, 5, 4, 1, 1)
        layout.addWidget(self.speed6_radio, 5, 5, 1, 1)
        
        self.setLayout(layout)
        
    # updates the RunViewer window graphics every 200 ms
    def update_run_graphics(self):
    # Update the progress bars and labels based on the latest voltage and current
        global countdown
         
        self.voltage_label.setText("Voltage: " + str(V_inst) + " V")
        self.current_label.setText("Current: " + str(I_inst) + " mA")
        self.power_label.setText("Power: " + str(P_inst)[:7] + " mW")
        self.energy_label.setText("Energy: " + str(E_inst) + " J")
        self.state_label.setText(S_inst)
        self.time_label.setText("Time Remaining: " + "{:.2f}".format(countdown))
        
        self.voltage_progress.setValue(int((V_inst / 12) * 100))  # Set voltage as percentage (max 15V)
        self.current_progress.setValue(int((I_inst / 1500) * 100))  # Set current as percentage (max 5A)
        self.power_progress.setValue(int((P_inst / 3000) * 100))
        self.energy_progress.setValue(int((E_inst / 500) * 100))
        
        countdown =  countdown - 0.2  #timer tick
        
        global timerDone
        if countdown <= 0:
            timerDone = True
            
    def update_speed(self, speednum):
        match speednum:
            case 1:
                send_message("S1")
            case 2:
                send_message("S1")    
            case 3:
                send_message("S3")
            case 4:
                send_message("S4")  
            case 5:
                send_message("S5")
            case 6:
                send_message("S6")  
# ==================================================================================================================
# ===== MAIN WINDOW ================================================================================================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ESP32 Bluetooth Control")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.begin_button = QPushButton("Begin Run")
        self.logviewer_window_button = QPushButton("Open Power Log Viewer")
        
        big_buttons = [self.begin_button, self.logviewer_window_button]
        for i in big_buttons:
            i.setMinimumSize(300, 200)
            
        self.browse_button = QPushButton("Browse...")
        self.path_input = QLineEdit()
        
        # Connect buttons to functions
        self.begin_button.clicked.connect(self.begin_run)
        self.logviewer_window_button.clicked.connect(self.open_logviewer_window)
        self.browse_button.clicked.connect(self.select_file)
        
        buttons = [self.begin_button, self.logviewer_window_button, self.browse_button] 
        
        for i in buttons:
            i.setMinimumSize(80, 200)

        layout = QGridLayout()
        # Add the buttons to the window
        layout.addWidget(self.begin_button, 0, 0, 1, 4)
        layout.addWidget(self.logviewer_window_button, 1, 0, 1, 4)
        layout.addWidget(self.browse_button, 2, 0, 1, 1)
        layout.addWidget(self.path_input, 2, 1, 1, 3)

        self.setLayout(layout)

    # Open a new Log Viewer Window to allow user to see a data log
    def open_logviewer_window(self):
        self.logviewer_window = LogViewer()
        self.logviewer_window.show()
    
    # Select a file from the file explorer
    # allows user to determine where they want to store their data log
    def select_file(self):
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Create or Select CSV File",
            filter="CSV Files (*.csv)"
        )
        if file_path:
            if not file_path.lower().endswith(".csv"):
                file_path += ".csv"
            self.path_input.setText(file_path)
    
    # Begins a new run, brings up a new Run Viewer Window
    # Called when user presses "Begin New Run" button
    def begin_run(self):
        print("Starting a new run.")
        
        global add_to_log, data_file
        
        try:
            data_file = self.path_input.text()
        except:
            print("User has not selected a file. Default file will be used.")
        
        initialize_csv(data_file)
        add_to_log = True           # only start writing data to log file when we have started a run
        
        #open the run window
        self.runviewer_window = RunViewer()
        self.runviewer_window.show()
        
# ==================================================================================================================
# GUI FUNCTIONS ====================================================================================================
def send_START():
    print("Sending START to vehicle.")
    send_message("Start")

def send_STOP():
    print("Sending STOP to vehicle.")
    send_message("Stop")
    
def update_speed():
    print("Sending Speed to vehicle.")
    #TODO: pull desired parameter from GUI
    send_message("S1")

def send_message(command):
    global bluetooth_serial, serial_lock
    
    command = command + "\n"        # ADD A NULL TERMINATOR CHARACTER!!!
    
    with serial_lock:
        bluetooth_serial.write(command.encode('utf-8'))
        
    # if the connection is lost, break
    if not bluetooth_serial.is_open:
        print("Error: Serial port is not open.")
        return

        
# ==============================================================================
#=========================================================================================

# 
def calculateEnergyUsage(filepath):
    # TODO: implement
    total_energy_spent = 10
    total_energy_regained = 2
    
    return total_energy_spent, total_energy_regained

def initialize_csv(filepath):
    with open(filepath, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Log Number", "Voltage (V)", "Current (mA)", "Power (mW)", "Energy (J)", "State"])
        
        
""" Separate the numbers out from the response string"""
""" Separate the numbers out from the response string"""
def parse_dataLog(response):
    values = response.split(":")

    global I_inst, V_inst, P_inst, S_inst, E_inst, add_to_log, log_num, state_map
    
    I_inst = float(values[0])             # current in mA
    V_inst = float(values[1])             # voltage in V
    P_inst = I_inst * V_inst              # power in mW
    S_inst = "foo"
    #S_inst = state_map[str(values)]
    #S_inst = state_map.get(values[2].strip(), "UNKNOWN")          # state enumeration
    v_cap = float(values[3])

    # TODO: Scale v_cap back to its normal value

    E_inst = v_cap * v_cap * 1000 / 2  # turn power supply voltage to energy


    print("TIME (s): " + str(log_num *0.5))
    print("Current (mA): " + str(I_inst))
    print("Voltage (V): " + str(V_inst))
    print("Power (mW): " + str(P_inst)[:7])
    print("Energy (J):" + str(E_inst))



    if add_to_log:
        log = [log_num, V_inst, I_inst, P_inst, E_inst, S_inst]
        #     packet num, Voltage, Current, Power, Energy, State
        add_to_csv(data_file, log)
        log_num = log_num + 1
    

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
            #response = bluetooth_serial.read(num_bytes)
            response = bluetooth_serial.readline()
            
        if response:
            print("Received data: ", response)
        else:
            print("No data received")
            continue

            
        response_str = str(response)[2:]        #strip out the first two characters "b'"
        response_str = response_str[:-3]        #strip out the three characters too "\n'"
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
    
    # If Bluetooth Connection is properly established, start Rx thread and run the GUI
    start_Rx_thread()
    
    app = QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
    print("Set up the GUI")
        


        
except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    # Close the serial port
    if 'bluetooth_serial' in locals() and bluetooth_serial.is_open:
        bluetooth_serial.close()
        print("Serial port closed")

      
