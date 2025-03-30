# Milestone 2 Submission

For Milestone 2, please review the following files.

1. INA219_steering_speed_test > INA219_steering_speed_test.ino `(steering servos, controlling speed, reading from INA219)`
2. bluetooth_test > bluetooth_test.ino `(Bluetooth communication)`
3. HUSKYLENS_test > HUSKYLENS_test.ino `(interfacing w/ HUSKYLENS camera)`
4. system_test > system_test.ino `(integrating Bluetooth, INA219, and camera)`
5. speed_control_test > speed_control_test.ino `(experimenting with drive wheel PWM)`
6. steering_angle_control_test > steering_angle_control_test.ino `(experimenting with steering servo PWM)`

*note: 1-4 are the same files as in Milestone 1 but renamed for clarity*

Software outline document placed in the Milestone 2 folder in our Teams channel.

## Documentation Plan
### 1. The relationship of the competition to ECE technology
Electrical and Computer Engineering (ECE) is a large and varied field of study that forms the basis of a lot of the technologies that people regularly use. Some of the main subfields of ECE are controls, Signal Processing, Power electronics, Communication, Electronics and Circuits, and Electromagnetic. Through this competition some of the topics that will be discussed are controls, power electronics, communication, and general engineering practices! Controls   will be discussed for controlling the steering and speed of the robot. Communication for the talking between the user and the robot through Bluetooth. Lastly, power electronics will be discussed in the creation of the battery and capacitor array to power the robot. 
### 2. UI User Manual
-	GUI layout
-	Viewing energy gained/expended by robot
-	How to get updated voltage/power/energy plot
-	Reading capacitor charge levels
    -	Low battery warning
-	Starting/stopping robot during drive periods
-	Controlling robot speed
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
