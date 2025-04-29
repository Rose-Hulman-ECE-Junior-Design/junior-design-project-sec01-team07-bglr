# Milestone 3 Submission

For Milestone 4, please review the following files.


1. HUSKYLENS_test > HUSKYLENS_test.ino `(added delay for sampling camera data)`
2. skeleton > ESP32_Vehicle.cpp `(contains working P controller for line following and updated datalogging)`
3. skeleton > ESP32_Vehicle.h `(added constants and initialization functions)`
4. skeleton > skeleton.ino `(updated state conditions and Bluetooth)`
5. test_BT_controller.py `(provide data outputs to GUI from BT link)`
6. test_BluetoothLink.py `(establish BT link on startup)`
7. test_Python_threads.py `(run parallel threads for BT transmission/reception)`



## Documentation Plan
### 1. The relationship of the competition to ECE technology
Electrical and Computer Engineering (ECE) is a large and varied field of study that forms the basis of a lot of the technologies that people regularly use. Some of the main subfields of ECE are controls, Signal Processing, Power electronics, Communication, Electronics and Circuits, and Electromagnetic. Through this competition some of the topics that will be discussed are controls, power electronics, communication, and general engineering practices! Controls   will be discussed for controlling the steering and speed of the robot. Communication for the talking between the user and the robot through Bluetooth. Lastly, power electronics will be discussed in the creation of the battery and capacitor array to power the robot. 
### 2. UI User Manual
To use the ESP32 Vehicle Controller Graphic User Interface (GUI), see the following documentation.

#### Section 1 - Pairing Your Laptop to the ESP32

The first task at hand is to pair your laptop to the ESP32 Vehicle.

First, open your Bluetooth & Other Devices settings (Settings -> Devices) From there, select "Add Bluetooth or other device". A window titled "Add a device" should show up. Make sure your ESP32 Vehicle is powered and on, then select "CurentHogs_ESP32_SPP_Device". You should see a message reading "Your device is ready to go!" once the vehicle has been paired.

Next, go back to Bluetooth & Other Devices, scroll down to Related Settings, and click on "More Bluetooth options". In the Bluetooth Settings window, click on the COM Ports tab. Identify which port is has been assigned to CurrentHogs_ESP32_SPP_Device in the Incoming direction. Be sure to write down this COM Port number down somewhere, you will need it in the next step. This part is important! We need to be able to tell the VC-GUI which COM port to connect to. 

What are COM ports? https://www.serial-over-ethernet.com/serial-to-ethernet-guide/what-is-com-port/

#### Section 2 - The Python GUI
The Vehicle Controller GUI (VC-GUI) is an app run by a Python script. Make sure you have Python 3.10.11 and the following packages installed:

pandas
matplotlib
PyQt6
Pyserial

How to install Python:
https://www.geeksforgeeks.org/how-to-install-python-on-windows/

How to install a Python package:
https://packaging.python.org/en/latest/tutorials/installing-packages/

You should now have Python up and running. But before we can run our app, we need to make a small change. Open ControllerGUI.py using your choice of editor (you can legit open this with Notepad). Near the top of the file, look for the Bluetooth Variables section. Change the variable port to whichever COM Port your ESP32 Vehicle is connected to (remember? from Section 1?). For example, if my device had connected on COM Port 9, I would change this line to

port = 'COM9'

Feel free to check out the other parameters and the rest of the code, but do not make any changes (or else). Now we are ready to roll.

Open your file explorer in whichever location your repository is located. In the file explorer, right-click
To run the VC-GUI, enter the following line into the Command Terminal

python3 ControllerGUI.py

You should see the following 

Connected to port COM8 at 115200 baud.



### 3. How to get and use software
-	Go to the [Arduino website](https://www.arduino.cc/en/software/) and select Download Options > Windows 10+ or appropriate operating system
    -	Follow installation instructions
-	Installing libraries and configuring IDE
	- Same instructions as vehicle overview document in Moodle
-	How to use arduino ide (writing code, uploading to esp32, etc)
### 4. API use for writing new programs using your code, etc.
>-	Not sure what this will look like yet
 >    -	How the control system works, changing control variables, etc.
### 5. operation of the robot in relation to (2) and (4)
-	How to charge robot (wired and wireless)
-	Robot should be off when uploading code (flip switch)
-	State diagram?
-	Other relevant information

__The goal is to write this information in a way that effectively communicates what we need to and also maintain a fun and educational tone (easy for HS students/teachers to follow)__
