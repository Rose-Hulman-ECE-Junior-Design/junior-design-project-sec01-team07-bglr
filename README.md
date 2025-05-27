# Welcome to the ESP32!

This is a simple guide to setting up your computer to be able to run your robot code. You will learn how to install the required software, learn how the line-following works, and how to use the GUI that tracks your runs. Let's get started!


## YOUR TASKS

### Calculate the Angle Error

The line-following works by  to calculate the angle error, or the difference between the angle of the arrow and a vertical line on the center of the screen (0 degrees). We also need to determine the difference between the x-position of the tip of the arrow and the center of the screen.

![image](https://github.com/user-attachments/assets/637b97e7-b042-4415-bff2-e19708054542)

To find the angle error, use the following formula:

$\text{error}_1 = \tan^{-1} \left( \frac{\text{target.x} - \text{origin.x}}{\text{target.y} - \text{origin.y}} \right)$

To find the centering/displacement error, use the following formula:

$\text{error}_2 = \tan^{-1} \left( \frac{ \left( \frac{1}{2} \left( \text{origin.x} + \text{target.x} \right) - \text{center.x} \right) }{ \text{camera.height} } \right)$

Although the code will calculate these values automatically, these forumlas provide a mathematical background for how it works. Now let's talk about the user interface!

## UI User Manual
To use the ESP32 Vehicle Controller Graphic User Interface (GUI), see the following documentation.

### Arduino installation
Go to the [Arduino website](https://www.arduino.cc/en/software/) and select `Download Options > Windows 10+` or appropriate operating system and follow installation instructions.

Open your Vehicle_Main.ino in your Arduino Editor. Go to `Sketch > Include Libraries > Manage Libraries`. The Library Manager should pop up. Search for and install the following libraries.
    - QuickPID - this library is used to implement the PID Controller (more on that later)
    - Adafruit INA219 - this library is used to interface with the INA219 sensor on the vehicle, which allows us to obtain our voltage and current readings

Next, go to `Sketch > Include Library > Add .ZIP Library`. Use the .zip files provided on Moodle to install the remaining libraries.
    - HUSKYLENS - this library is used to interface with the HUSKYLENS camera


What are Libraries? Libraries are collections of "pre-written" code that somebody else already wrote that allows you, the programmer, to have a MUCH easier time doing stuff. Libraries let you use a variety of different functions. It might take a bit of reading documentation to find *how* exactly to use those functions.

Why do we use them? Because there is no need to reinvent the wheel :)



#### Section 1 - Pairing Your Laptop to the ESP32

The first task is to pair your laptop to the ESP32 Vehicle.

First, open your Bluetooth & Other Devices settings `(Settings -> Devices)` From there, select "Add Bluetooth or other device". A window titled "Add a device" should show up. Make sure your ESP32 Vehicle is powered and on, then select "CurentHogs_ESP32_SPP_Device". You should see a message reading "Your device is ready to go!" once the vehicle has been paired.

Next, go back to Bluetooth & Other Devices, scroll down to Related Settings, and click on "More Bluetooth options". In the Bluetooth Settings window, click on the COM Ports tab. Identify which port is has been assigned to CurrentHogs_ESP32_SPP_Device in the Outgoing direction. Be sure to write down this COM Port number down somewhere, you will need it in the next step. This part is important! We need to be able to tell the VC-GUI which COM port to connect to. 

What are COM ports? https://www.serial-over-ethernet.com/serial-to-ethernet-guide/what-is-com-port/

#### Section 2 - Installing Python
The Vehicle Controller GUI (VC-GUI) is an app run by a Python script. Make sure you have Python 3.10.11 and the following packages installed:
```
- pandas
- matplotlib
- PyQt6
- Pyserial
```
Any version of Python 3 should work. To install a package, open up your Command Terminal and type 'pip install my_package' For example:

`pip install pandas`

How to install Python:
https://www.geeksforgeeks.org/how-to-install-python-on-windows/

How to install a Python package:
https://packaging.python.org/en/latest/tutorials/installing-packages/

#### Section 3 - Starting the GUI
You should now have Python up and running. But before we can run our app, we need to make a small change. Open ControllerGUI.py using your choice of editor (you can legit open this with Notepad). Near the top of the file, look for the Bluetooth Variables section. Change the variable port to whichever COM Port your ESP32 Vehicle is connected to (remember? from Section 1?). For example, if my device had connected on COM Port 9, I would change this line to

`port = 'COM9'`

Feel free to check out the other parameters and the rest of the code, but do not make any changes (or else). Now we are ready to roll.

Open your file explorer in whichever location your repository is located. In the file explorer, click the 

`python3 ControllerGUI.py`

You should see something like the following line printed to the Command Terminal if everything connected properly.

Connected to port COM9 at 115200 baud.

The main window will display, and the VC-GUI is ready to be used.

#### Section 4 - Using the GUI
Once the VC-GUI is up and running, you should see a starting window that looks like this:

![image](https://github.com/user-attachments/assets/f1722687-c07f-419d-a8d6-e9ba78284726)


To begin a run, first press the Browse button and either create or select a .csv file to save your run's data to. Be sure to name your .csv file something specific and descriptive, like validationTest1_speed3.csv. Otherwise, you might easlily lose track of which file is which.

Once you have selected a file, you may press the Begin a Run button. A window like this will pop up.

![image](https://github.com/user-attachments/assets/1a3f453a-980e-4c48-8842-a26e04154e42)


As soon as the window pop up, 

There are several controls:
  - the Start button sends the vehicle into driving mode
  - the Stop button pauses the vehicle
  - the Recharge button pauses the vehicle and begins the recharging period
  - the Speed radio buttons send the vehicle into driving mode and select the vehicle's speed
  - the PID Controller menu allows you to modify the PID controller constants

There are also several displays:
  - the Voltage, Current, Power, and Energy bars show the instantaneous power data readings. (Note: This is the data that will be stored into the .csv file,   you get to see it live!)
  - the State label shows the current state of the vehicle (whether it is driving, idle, or recharging). This label is there to allow you to identify when something is wrong. (For example, if the vehicle says it is in the Driving state but it is not moving!)
  - the Timer shows you how much time is left in the current period. This feature is there to help you not break the competition rules; the vehicle will automatically stop and enter the recharging state once the initial 90-second driving period is up.

Take some time to play around with these controls and get a good feel as to how the robot drives. Observe what happens to the Current when the vehicle is stopped vs when the vehicle is running around. Observe what happens when you modify Kd, Ki, Kp1, or Kp2. Does the vehicle stay on the line or does it drive off? Does it drive a little funky? Can you get the vehicle to drive as smooth as possible?


Once your time has ended, the RunViewer Window will close. Go back to the Main Window and press the Log Viewer button. The Log Viewer window will show up. Press the Plot button at the top of the window and select your data file. Your data will be plotted onto the area below, like this:

![image](https://github.com/user-attachments/assets/5ea03739-929d-40b5-acb0-5dea9bd4541c)



Now you can see the Voltage, Current, Power, and Energy vs Time throughout your run. Do you notice any patterns?







