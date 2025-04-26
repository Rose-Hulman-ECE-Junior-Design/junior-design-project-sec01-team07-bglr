import sys
import pandas as pd
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QFileDialog
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800

data_file = ""    #filepath to .csv file where data will be stored

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