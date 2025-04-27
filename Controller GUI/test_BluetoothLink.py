
import serial
import time


print("Hello CK!")


port = 'COM12'

baud_rate = 115200 
num_bytes = 48;


try:
    #attempt to establish a seiral connection
    #bluetooth_serial = serial.Serial(port, baud_rate)
    bluetooth_serial = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
    print(bluetooth_serial.name)
    print(f"Connected to port {port} at {baud_rate} baud.")


    while 1:
        #command = input("What would you like to send? ")
        #print("Sending: " + command)
        
        #bluetooth_serial.write(command)

        response = bluetooth_serial.read(num_bytes)
        print(response)
        print('======')


except serial.SerialException as e:
    print(f"Error: {e}")


finally:
    # Close the serial port
    if 'bluetooth_serial' in locals() and bluetooth_serial.is_open:
        bluetooth_serial.close()
        print("Serial port closed")
        
