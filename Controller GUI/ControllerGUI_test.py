import sys
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800


class LogViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Power Log Viewer")
        self.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        layout = QVBoxLayout()
        label = QLabel("This is a second window!")
        layout.addWidget(label)
        self.setLayout(layout)
        
        
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

    
    def open_logviewer_window(self):
        self.second_window = LogViewer()
        self.second_window.show()
        
        
app = QApplication([])
window = MainWindow()
window.show()
sys.exit(app.exec())