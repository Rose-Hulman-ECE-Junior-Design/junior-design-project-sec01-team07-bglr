import sys
import csv
import pandas as pd
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QLineEdit, QVBoxLayout, QFileDialog, QProgressBar, QGridLayout
from PyQt6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800

data_file = ""    #filepath to .csv file where data will be stored

V_inst = 7.223
I_inst = 302
P_inst = 2181.34
S_inst = "DRIVING"
E_inst = 200
countdown = 90.0
timerDone = False

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
        
        all_buttons = [self.start_button, self.stop_button]
        
        for i in all_buttons:
            i.setMinimumSize(80, 200)  # (ancho, largo)
        
        # Set up the timer to call update_run_graphics every 200ms
        timer = QTimer(self)
        timer.timeout.connect(self.update_run_graphics)  # Connect the update function
        timer.start(200)  # Call every 200ms
        
        self.instructions_label = QLabel("Insert RunViewer Instructions Here.")
        self.instructions_label.resize(50, 30)
        
        # Labels
        self.voltage_label = QLabel("Voltage: " + str(V_inst) + " V", self)
        self.current_label = QLabel("Current: " + str(I_inst) + " mA", self)
        self.power_label = QLabel("Power: " + str(P_inst) + " mW", self)
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
            self.voltage_progress.setRange(0, 100)     # Progress bar from 0 to 100%
            self.voltage_progress.setTextVisible(True) # Show percentage
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
        
        # Buttons
        self.start_button.clicked.connect(send_START)
        self.stop_button.clicked.connect(send_STOP)

        # Adding to Layout - grid format
        layout = QGridLayout()
        layout.addWidget(self.instructions_label, 0, 0, 1, 6)
        layout.addWidget(self.time_label, 1, 4, 1, 2)
        
        layout.addWidget(self.voltage_label, 4, 0, 1, 1)
        layout.addWidget(self.current_label, 4, 1, 1, 1)
        layout.addWidget(self.power_label, 4, 2, 1, 1)
        layout.addWidget(self.energy_label, 4, 3, 1, 1)
        layout.addWidget(self.state_label, 1, 0, 1, 3)
        
        layout.addWidget(self.voltage_progress, 2, 0, 2, 1)
        layout.addWidget(self.current_progress, 2, 1, 2, 1)
        layout.addWidget(self.power_progress, 2, 2, 2, 1)
        layout.addWidget(self.energy_progress, 2, 3, 2, 1)
        
        layout.addWidget(self.start_button, 2, 4, 1, 2)
        layout.addWidget(self.stop_button, 3, 4, 1, 2)
        
        
        
        self.setLayout(layout)
        
    # updates the RunViewer window graphics every 200 ms
    def update_run_graphics(self):
    # Update the progress bars and labels based on the latest voltage and current
        global countdown
         
        self.voltage_label.setText("Voltage: " + str(V_inst) + " V")
        self.current_label.setText("Current: " + str(I_inst) + " mA")
        self.power_label.setText("Power: " + str(P_inst) + " mW")
        self.energy_label.setText("Energy: " + str(E_inst) + " J")
        self.state_label.setText(S_inst)
        self.time_label.setText("Time Remaining: " + "{:.2f}".format(countdown))
        
        self.voltage_progress.setValue(int((V_inst / 12) * 100))  # Set voltage as percentage (max 15V)
        self.current_progress.setValue(int((I_inst / 1500) * 100))  # Set current as percentage (max 5A)
        self.power_progress.setValue(int((P_inst / 3000) * 100))
        self.energy_progress.setValue(int((E_inst / 500) * 100))
        
        countdown =  countdown - 0.2  #timer tick
        
        if countdown <= 0:
            timerDone = True


# ======================================================================================
# ===== MAIN WINDOW ===========
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
        data_file = self.path_input.text()
        
        initialize_csv(data_file)
        add_to_log = True
        
        
        #open the run window
        self.runviewer_window = RunViewer()
        self.runviewer_window.show()
        
# ===============================================================================

def send_START():
    print("Sending START to vehicle.")
    #TODO: implement

def send_STOP():
    print("Sending STOP to vehicle.")
    #TODO: implement
    
def update_K():
    print("Sending K to vehicle.")
    #TODO: implement


def initialize_csv(filepath):
    with open(filepath, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Log Number", "Voltage (V)", "Current (mA)", "Power (mW)", "Energy (J)", "State"])
             
# ==============================================================================

# TODO: establish BT connection
      
app = QApplication([])
window = MainWindow()
window.show()
sys.exit(app.exec())