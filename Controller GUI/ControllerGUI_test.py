import sys
import pandas as pd
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QFileDialog, QProgressBar, QGridLayout
from PyQt6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800

data_file = ""    #filepath to .csv file where data will be stored

V_inst = 7.223
I_inst = 302
P_inst = 2181.34

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
        
        # Set up the timer to call update_progress_bars every 100ms
        timer = QTimer()
        timer.timeout.connect(lambda: self.update_progress_bars(self))  # Connect the update function
        timer.start(100)  # Call every 100ms
        
        # Voltage label and progress bar
        self.voltage_label = QLabel("Voltage (V)" + str(V_inst), self)
        self.current_label = QLabel("Current (mA)" + str(I_inst), self)
        self.power_label = QLabel("Power (mW)" + str(P_inst), self)
        
        self.voltage_progress = QProgressBar(self)
        self.voltage_progress.setRange(0, 12)  # Progress bar from 0 to 100%
        self.voltage_progress.setTextVisible(True)  # Show percentage
        
        self.current_progress = QProgressBar(self)
        self.current_progress.setRange(0, 2000)  # Progress bar from 0 to 100%
        self.current_progress.setTextVisible(True)  # Show percentage
        
        self.power_progress = QProgressBar(self)
        self.power_progress.setRange(0, 2000)  # Progress bar from 0 to 100%
        self.power_progress.setTextVisible(True)  # Show percentage
        
        
        self.start_button.clicked.connect(send_START)
        self.stop_button.clicked.connect(send_STOP)

        layout = QGridLayout()
        label = QLabel("Insert RunViewer Instructions Here.")
        layout.addWidget(label, 0, 0)
        
        layout.addWidget(self.voltage_label)
        layout.addWidget(self.current_label)
        layout.addWidget(self.power_label)
        
        layout.addWidget(self.voltage_progress)
        layout.addWidget(self.current_progress)
        layout.addWidget(self.power_progress)
        
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        self.setLayout(layout)
        
    def update_progress_bars(self):
    # Update the progress bars and labels based on the latest voltage and current
        self.voltage_label.setText("Voltage (V)" + str(V_inst))
        self.current_label.setText("Current (mA)" + str(I_inst))
        self.power_label.setText("Power (mW)" + str(P_inst))
        
        self.voltage_progress.setValue(int((V_inst / 15.0) * 100))  # Set voltage as percentage (max 15V)
        self.current_progress.setValue(int((I_inst / 500) * 100))  # Set current as percentage (max 5A)
        self.power_progress.setValue(int((P_inst / 5000) * 100))


# ======================================================================================
# ===== MAIN WINDOW ===========
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ESP32 Bluetooth Control")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.begin_button = QPushButton("Begin Run")
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


        
# ==============================================================================

# TODO: establish BT connection
      
app = QApplication([])
window = MainWindow()
window.show()
sys.exit(app.exec())