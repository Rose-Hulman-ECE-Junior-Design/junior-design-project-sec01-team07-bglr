# Milestone 4 Submission

For Milestone 4, please review the following files.


1. Controller GUI > ControllerGUI.py `(actual GUI)`
2. Controller GUI > ControllerGui_test.py `(testing GUI features)`
3. Controller GUI > dataLog_test1.csv `(testing data logging from ESP32/INA219)`



## UI User Manual
To use the ESP32 Vehicle Controller Graphic User Interface (GUI), see the following documentation.

#### Section 1 - Pairing Your Laptop to the ESP32

The first task is to pair your laptop to the ESP32 Vehicle.

First, open your Bluetooth & Other Devices settings (Settings -> Devices) From there, select "Add Bluetooth or other device". A window titled "Add a device" should show up. Make sure your ESP32 Vehicle is powered and on, then select "CurentHogs_ESP32_SPP_Device". You should see a message reading "Your device is ready to go!" once the vehicle has been paired.

Next, go back to Bluetooth & Other Devices, scroll down to Related Settings, and click on "More Bluetooth options". In the Bluetooth Settings window, click on the COM Ports tab. Identify which port is has been assigned to CurrentHogs_ESP32_SPP_Device in the Outgoing direction. Be sure to write down this COM Port number down somewhere, you will need it in the next step. This part is important! We need to be able to tell the VC-GUI which COM port to connect to. 

What are COM ports? https://www.serial-over-ethernet.com/serial-to-ethernet-guide/what-is-com-port/

#### Section 2 - Installing Python
The Vehicle Controller GUI (VC-GUI) is an app run by a Python script. Make sure you have Python 3.10.11 and the following packages installed:

pandas
matplotlib
PyQt6
Pyserial

Any version of Python 3 should work. To install a package, open up your Command Terminal and type 'pip install my_package' For example:

pip install pandas

How to install Python:
https://www.geeksforgeeks.org/how-to-install-python-on-windows/

How to install a Python package:
https://packaging.python.org/en/latest/tutorials/installing-packages/

#### Section 3 - Using the GUI
You should now have Python up and running. But before we can run our app, we need to make a small change. Open ControllerGUI.py using your choice of editor (you can legit open this with Notepad). Near the top of the file, look for the Bluetooth Variables section. Change the variable port to whichever COM Port your ESP32 Vehicle is connected to (remember? from Section 1?). For example, if my device had connected on COM Port 9, I would change this line to

port = 'COM9'

Feel free to check out the other parameters and the rest of the code, but do not make any changes (or else). Now we are ready to roll.

Open your file explorer in whichever location your repository is located. In the file explorer, click the 

python3 ControllerGUI.py

You should see something like the following line printed to the Command Terminal if everything connected properly.

Connected to port COM9 at 115200 baud.

The main window will display, and the VC-GUI is ready to be used.

### Arduino installation
-	Go to the [Arduino website](https://www.arduino.cc/en/software/) and select Download Options > Windows 10+ or appropriate operating system
    -	Follow installation instructions
-	Installing libraries and configuring IDE
	- Same instructions as vehicle overview document in Moodle
-	How to use arduino ide (writing code, uploading to esp32, etc)
