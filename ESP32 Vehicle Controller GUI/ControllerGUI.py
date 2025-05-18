import sys
import csv
import serial
import time
import threading
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QRadioButton, QLabel, QPushButton, QDoubleSpinBox, QLineEdit, QVBoxLayout, QFileDialog, QProgressBar, QGridLayout
from PyQt6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

""" 

ControllerGUI.py

ESP32 Vehicle Controller GUI

Features: 

Main Window allows the user to choose between pulling up the Run Viewer 
or the Log Viewer. It also prompts the user to select a file to which 
data will be saved to.

Run Viewer allows the user to control the vehicle during a run. User may:
- start the vehicle by pushing a button
- stop the vehicle by pushing a button
- select vehicle speed by selecting a radio button
- modify controller parameters by inputting 
- view the amount of time remaning in the run
- view the vehicle's current state
- view the vehicle's current power data

Log Viewer allows the user to see the vehicle's power data in plot form.
User may select which .csv data file they want to view. Window displays:
- voltage vs time plot
- current vs time plot
- power vs time plot
- energy vs time plot
- total energy consumed by vehicle
- total energy regained during the recharging period
- average power

Author: CKG

"""


# GUI VARIABLES ======================================================================================================
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800

data_file = "temp.csv" #filepath to .csv file where data will be stored

V_inst = 0
I_inst = 0
P_inst = 0
S_inst = "IDLE"
E_inst = 0
timerDone = False
log_num = 0

speed = 0

state_map = {
    0: "IDLE",
    1: "DRIVING",
    2: "CHARGING",
    3: "COMPLETE"
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

# SYSTEM VARIABLES ==================================================================================================
V_MAX = 12                  # TODO: consult LL and BB about these numbers
I_MAX = 1300
P_MAX = V_MAX * I_MAX
E_MAX = 300

powerSupply_capacitance = 100   #


RECHARGING_PERIOD = 45      # recharging period, seconds
FIRST_DRIVING_PERIOD = 90
SECOND_DRIVING_PERIOD = 45

first_timer_done = False      # first driving period finished
second_timer_done = False     # recharging period finished
third_timer_done = False      # second driving period finished

time_remaining = FIRST_DRIVING_PERIOD
timer_tick = 0.2

# ===================================================================================================================
# ===================================================================================================================
class LogViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Power Log Plot Viewer")
        self.setMinimumSize(WINDOW_WIDTH, WINDOW_HEIGHT)

        # UI elements
        self.plot_button = QPushButton("Plot CSV Data")
        self.plot_button.clicked.connect(self.plotPowerLog)
        self.canvas = FigureCanvas(plt.Figure(figsize=(10, 6)))

        self.energy_spent_label = QLabel("Total Energy Spent: N/A")
        self.energy_gained_label = QLabel("Total Energy Gained (RECHARGE): N/A")
        self.avg_power_label = QLabel("Average Power: N/A")

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.plot_button)
        layout.addWidget(self.canvas)
        layout.addWidget(self.energy_spent_label)
        layout.addWidget(self.energy_gained_label)
        layout.addWidget(self.avg_power_label)
        self.setLayout(layout)
        

    """ Plots the voltage, current, power, and energy on four subplots given a .csv data file."""      
    def plotPowerLog(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select a CSV file", "", "CSV Files (*.csv)")
        if not file_path:
            print("No file selected.")
            return

        try:
            df = pd.read_csv(file_path)
        except Exception as e:
            print(f"Failed to read CSV: {e}")
            return

        # Prepare data
        df['Time (s)'] = df['Log Number'] * 0.5  # or however time is calculated
        recharge_mask = df['State'] == 'RECHARGE'
        recharge_start = df['Time (s)'][recharge_mask].iloc[0] if recharge_mask.any() else None
        recharge_end = df['Time (s)'][recharge_mask].iloc[-1] if recharge_mask.any() else None

        # Clear old plots
        fig = self.canvas.figure
        fig.clear()
        axs = fig.subplots(4, 1, sharex=True)
        fig.tight_layout(pad=3.0)

        # Plot variables
        plot_vars = ['Voltage (V)', 'Current (mA)', 'Power (mW)', 'Energy (J)']
        colors = ['blue', 'green', 'red', 'purple']
        
        # Identify RECHARGE section
        recharge_mask = df['State'] == 'CHARGING'
        if recharge_mask.any():
            recharge_indices = df.index[recharge_mask]
            recharge_start = df['Time (s)'].iloc[recharge_indices[0]]
            recharge_end = df['Time (s)'].iloc[recharge_indices[-1]]
        else:
            recharge_start = recharge_end = None

        for i, (var, color) in enumerate(zip(plot_vars, colors)):
            if var in df.columns:
                axs[i].plot(df['Time (s)'], df[var], label=var, color=color)
                if recharge_start is not None:
                    axs[i].axvspan(recharge_start, recharge_end, color='green', alpha=0.3, label='RECHARGE')
                axs[i].set_ylabel(var)
                axs[i].grid(True)
                axs[i].legend()

        axs[-1].set_xlabel("Time (s)")
        self.canvas.draw()

        # Update stats
        total_energy, recharge_energy, avg_power = calculateEnergyUsage(file_path)
        self.energy_spent_label.setText(f"Total Energy Spent: {total_energy:.2f} J")
        self.energy_gained_label.setText(f"Total Energy Gained (RECHARGE): {recharge_energy:.2f} J")
        self.avg_power_label.setText(f"Average Power: {avg_power:.2f} mW")
        
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
        self.recharge_button = QPushButton("Recharge")
        
        all_buttons = [self.start_button, self.stop_button, self.recharge_button]
        
        for i in all_buttons:
            i.setMinimumSize(80, 200)  # (ancho, largo)
            
        self.start_button.clicked.connect(send_START)
        self.stop_button.clicked.connect(send_STOP)
        self.recharge_button.clicked.connect(self.enter_Recharge)
        
        # Create radio buttons for options
        self.speed1_radio = QRadioButton("Speed 1", self)
        self.speed2_radio = QRadioButton("Speed 2", self)
        self.speed3_radio = QRadioButton("Speed 3", self)
        self.speed4_radio = QRadioButton("Speed 4", self)
        self.speed5_radio = QRadioButton("Speed 5", self)
        self.speed6_radio = QRadioButton("Speed 6", self)
        
        radio_buttons = [self.speed1_radio, self.speed2_radio, self.speed3_radio, self.speed4_radio, self.speed5_radio, self.speed6_radio]

        for idx, button in enumerate(radio_buttons, start=1):
            button.toggled.connect(lambda checked, k=idx: self.update_speed(k))
            i.setFixedSize(200, 100)
        
        # Create KP Controllers
        self.kp1_spin = QDoubleSpinBox(self)
        self.kp1_spin.setRange(0.0, 5.0)       # Set desired range
        self.kp1_spin.setDecimals(3)             # Number of decimal places
        self.kp1_spin.setSingleStep(0.1)         # Increment step
        self.kp1_spin.setPrefix("Kp1: ")

        self.kp2_spin = QDoubleSpinBox(self)
        self.kp2_spin.setRange(0.0, 5.0)
        self.kp2_spin.setDecimals(3)
        self.kp2_spin.setSingleStep(0.1)
        self.kp2_spin.setPrefix("Kp2: ")
        
        self.ki_spin = QDoubleSpinBox(self)
        self.ki_spin.setRange(0.0, 5.0)
        self.ki_spin.setDecimals(3)
        self.ki_spin.setSingleStep(0.1)
        self.ki_spin.setPrefix("Ki: ")

        self.kd_spin = QDoubleSpinBox(self)
        self.kd_spin.setRange(0.0, 5.0)
        self.kd_spin.setDecimals(3)
        self.kd_spin.setSingleStep(0.1)
        self.kd_spin.setPrefix("Kd: ")
        
        self.kp1_spin.valueChanged.connect(self.kp1_changed)
        self.kp2_spin.valueChanged.connect(self.kp2_changed)
        self.ki_spin.valueChanged.connect(self.ki_changed)
        self.kd_spin.valueChanged.connect(self.kd_changed)

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
        self.time_label = QLabel("Time Remaining: " + str(FIRST_DRIVING_PERIOD))
        
        all_labels = [self.voltage_label, self.current_label, self.power_label, self.energy_label, self.state_label, self.time_label]
        for i in all_labels:
            i.setStyleSheet("QLabel { color: black; font-size: 20px; }")
            i.setAlignment(Qt.AlignmentFlag.AlignCenter)
            i.setFixedSize(200, 100)
        
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
        
        layout.addWidget(self.voltage_label, 5, 0, 1, 1)        # add the progress labels
        layout.addWidget(self.current_label, 5, 1, 1, 1)
        layout.addWidget(self.power_label, 5, 2, 1, 1)
        layout.addWidget(self.energy_label, 5, 3, 1, 1)
        layout.addWidget(self.state_label, 1, 0, 1, 3)
        
        layout.addWidget(self.voltage_progress, 2, 0, 3, 1)     # add the progress bars
        layout.addWidget(self.current_progress, 2, 1, 3, 1)
        layout.addWidget(self.power_progress, 2, 2, 3, 1)
        layout.addWidget(self.energy_progress, 2, 3, 3, 1)
        
        layout.addWidget(self.start_button, 2, 4, 1, 2)         # add the stop/start buttons
        layout.addWidget(self.stop_button, 3, 4, 1, 2)
        layout.addWidget(self.recharge_button, 4, 4, 1, 2)
        
        layout.addWidget(self.speed1_radio, 6, 0, 1, 1)         # add the speed selector radio buttons
        layout.addWidget(self.speed2_radio, 6, 1, 1, 1)
        layout.addWidget(self.speed3_radio, 6, 2, 1, 1)
        layout.addWidget(self.speed4_radio, 6, 3, 1, 1)
        layout.addWidget(self.speed5_radio, 6, 4, 1, 1)
        layout.addWidget(self.speed6_radio, 6, 5, 1, 1)
        
        layout.addWidget(self.kp1_spin, 7, 0, 1, 1)
        layout.addWidget(self.kp2_spin, 7, 1, 1, 1)
        layout.addWidget(self.ki_spin, 8, 0, 1, 1)
        layout.addWidget(self.kd_spin, 8, 1, 1, 1)
        
        self.setLayout(layout)
        
    # updates the RunViewer window graphics every 200 ms
    def update_run_graphics(self):
    # Update the progress bars and labels based on the latest voltage and current
        global time_remaining, timerDone, timer_tick, first_timer_done, second_timer_done, third_timer_done, RECHARGING_PERIOD, SECOND_DRIVING_PERIOD
         
        self.voltage_label.setText("Voltage: " + str(V_inst) + " V")
        self.current_label.setText("Current: " + str(I_inst) + " mA")
        self.power_label.setText("Power: " + str(P_inst)[:7] + " mW")
        self.energy_label.setText("Energy: " + str(E_inst) + " J")
        self.state_label.setText(S_inst)
        self.time_label.setText("Time Remaining: " + "{:.2f}".format(time_remaining))
        
        self.voltage_progress.setValue(int(((V_inst - 7) / 1) * 100))  # Set voltage as percentage (max 15V)
        self.current_progress.setValue(int((I_inst / I_MAX) * 100))  # Set current as percentage (max 5A)
        self.power_progress.setValue(int((P_inst / P_MAX) * 100))
        self.energy_progress.setValue(int((E_inst / E_MAX) * 100))
        
        time_remaining =  time_remaining - timer_tick  #timer tick (200ms)
        
        # if time_remaining <= 0:
            
        #     if first_timer_done & second_timer_done:
        #         third_timer_done = True
        #         send_STOP()
        #         print("All timers done. Exiting...")
        #         self.close()
            
        #     if (first_timer_done) & (not second_timer_done) :
        #         second_timer_done = True
        #         # disable the control buttons
        #         self.start_button.setEnabled(True)
        #         self.stop_button.setEnabled(True)
                
        #         send_STOP()             # kick the vehicle out of Recharging mode
        #         time_remaining = SECOND_DRIVING_PERIOD
        #         print("Setting time remaining to SECOND_DRIVING_PERIOD")
            
        #     if (not first_timer_done):
        #         first_timer_done = True
        #         send_message("RECHARGE")        # kick the vehicle into recharging mode
                
        #         # disable the control buttons
        #         self.start_button.setEnabled(False)
        #         self.stop_button.setEnabled(False)
                
        #         time_remaining = RECHARGING_PERIOD
                
        #         print("Setting time remaining to RECHARGING_PERIOD")
                

    def enter_Recharge(self):
        #TODO: restart the timer for 45 seconds
        send_message("RECHARGE")
        
    def kp1_changed(self, value):
        send_message(f"Kp1={value:.3f}")  # You can format the value to 3 decimals if needed

    def kp2_changed(self, value):
        send_message(f"Kp2={value:.3f}")

    def ki_changed(self, value):
        send_message(f"Ki={value:.3f}")
    
    def kd_changed(self, value):
        send_message(f"Kd={value:.3f}")
    # Updates Vehicle speed.
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

# Send START command to vehicle.
def send_START():
    print("Sending START to vehicle.")
    send_message("Start")

# Send STOP command to vehicle.
def send_STOP():
    print("Sending STOP to vehicle.")
    send_message("Stop")

"""
Send a command to the vehicle.
Input - command string
"""
def send_message(command):
    global bluetooth_serial, serial_lock
    print("Sending: " + command)
    
    command = command + "\n"        # ADD A NULL TERMINATOR CHARACTER!!!
    
    with serial_lock:
        bluetooth_serial.write(command.encode('utf-8'))
        
    # if the connection is lost, break
    if not bluetooth_serial.is_open:
        print("Error: Serial port is not open.")
        return

# ===============================================================================================================
# ===============================================================================================================

""""
Extracts data from a .csv file and calculates

- the total energy consumed during the driving periods
- the total energy regained during the recharging period
- the average power used by the vehicle

Inputs - data filepath
Outputs - see parameters above

"""
def calculateEnergyUsage(filepath):
    # TODO: implement correctly
    total_energy_spent = 10
    total_energy_regained = 2
    avg_power = 3
    
    #TODO: compare the results between doing the power integration and 
    # subtracting starting vs ending instantaneous power supply energy
    
    df = pd.read_csv(data_file)           # read the csv file
    
    power = df["Power (mW)"].to_numpy()         # extract the power vector
    time = 0.5 * df["Log Number"].to_numpy()    # extract the time vector
    
    charging_df = df[df['State'] != 'CHARGING']
    power_re = charging_df["Power (mW)"].to_numpy()         # extract the power vector
    time_re = 0.5 * charging_df["Log Number"].to_numpy()    # extract time vector
    
    # Numerical integration using the trapezoidal rule
    energy = np.trapezoid(power, time)  # returns energy in joules
    
    return total_energy_spent, total_energy_regained, avg_power

"""Writes column names to the .csv for easier processing. """
def initialize_csv(filepath):
    with open(filepath, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Log Number", "Voltage (V)", "Current (mA)", "Power (mW)", "Energy (J)", "State"])
        
        
""" 
Extracts individual values from the serial packet received, updates global 
variables, and logs the data into the .csv file.
"""
def parse_dataLog(response):
    global I_inst, V_inst, P_inst, S_inst, E_inst, add_to_log, log_num, state_map
    
    values = response.split(":")          # extract individual values from the serial packet
    
    I_inst = float(values[0])             # current in mA
    V_inst = float(values[1])             # voltage in V
    P_inst = I_inst * V_inst              # power in mW
    S_inst = state_map.get(int(values[2]), "UNKNOWN")    # current state
    v_cap = float(values[3])              # capacitor voltage
    
    
    # TODO: Scale v_cap back to its normal value
    # E = (1/2)CV^2
    E_inst = v_cap * v_cap * powerSupply_capacitance / 2  # turn power supply voltage to energy


    #print("TIME (s): " + str(log_num *0.5))
    #print("Current (mA): " + str(I_inst))
    #print("Voltage (V): " + str(V_inst))
    #print("Power (mW): " + str(P_inst)[:7])
    #print("Energy (J):" + str(E_inst))
    #print("State: " + S_inst)

    if add_to_log:
        log = [log_num * 0.5, V_inst, I_inst, P_inst, E_inst, S_inst]
        #     packet num, Voltage, Current, Power, Energy, State
        add_to_csv(data_file, log)
        log_num = log_num + 1
    

""" 
Append row of collected data to .csv file
Inputs - filepath to desired csv
         row data to be appended
"""
def add_to_csv(filepath, row_data):
    """Appends a list of values as a new row into the given CSV file."""
    with open(filepath, mode="a", newline="") as file:  # "a" = append mode
        writer = csv.writer(file)
        writer.writerow(row_data)

""" Continuously reads power data logs from ESP32"""
def handle_Rx():
    print("Starting Rx thread.")
    global bluetooth_serial, serial_lock
    
    while 1:
        with serial_lock:
            #response = bluetooth_serial.read(num_bytes)
            response = bluetooth_serial.readline()
        
        if not response:
            print("No data received.")
            continue
            
        response_str = str(response)[2:]        #strip out the first two characters "b'"
        response_str = response_str[:-3]        #strip out the three characters too "\n'"
        # print('====================')
        
        # parse the response
        parse_dataLog(response_str)


        # if the connection is lost, break
        if not bluetooth_serial.is_open:
            print("Error: Serial port is not open.")
            return
        
        
# Start the Rx Thread
# this is run before we start the GUI
def start_Rx_thread():
    rx_thread = threading.Thread(target=handle_Rx, daemon=True) #run in the background
    rx_thread.start()
    
    
# ======================================================================================
# ======================== EXECUTION LOOP ==============================================

# Establish Bluetooth Connection =======================================================
try:
    #attempt to establish a serial connection
    #bluetooth_serial = serial.Serial(port, baud_rate)
    bluetooth_serial = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
    print(f"Connected to port {port} at {baud_rate} baud.")
    
    # If Bluetooth Connection is properly established, start Rx thread and run the GUI
    start_Rx_thread()
    
    app = QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
        
except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    # Close the serial port
    if 'bluetooth_serial' in locals() and bluetooth_serial.is_open:
        bluetooth_serial.close()
        print("Serial port closed")

      
