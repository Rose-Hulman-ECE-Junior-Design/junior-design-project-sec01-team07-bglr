import threading
import time
#import pandas
import serial
import time

# Skeleton User Controller framework
# generally, dispatch one thread to handle data reception
# and one thread to handle sending user commands.

print("Hello CK!")


port = 'COM11'

baud_rate = 9600
num_bytes = 48

bluetooth_serial = 0



#=========================================================================================
""" Separate the numbers out from the response string"""
def parse_dataLog(response):
    current = 0             # current in mA
    voltage = 0             # voltage in V
    power = 0               # power in mA
    state = 1               # state enumeration
    print(response)

#=========================================================================================
""" Reads power data logs from ESP32"""
def handle_Rx(bluetooth_serial):
    print("Starting Rx thread.")
    while 1:
    
        response = bluetooth_serial.read(num_bytes)
        print(response)
        print('======')
        
        # parse the response
    
        #TODO: if connection is ever lost, break
    
    
#=========================================================================================    
""" Sends user commands to the ESP32"""
def handle_Tx(bluetooth_serial):
    print("Starting Tx thread.")
    while 1:
        command = input("What would you like to send? ")
        print("Sending:" + command)
        
        # serial.write wants stuff in bytes
        # for future reference, can do b"Stop" or b"Start"
        bluetooth_serial.write(command.encode('utf-8'))
        
        #TODO: if connection is ever lost, break
    

#=========================================================================================

    
try:
    #attempt to establish a serial connection
    bluetooth_serial = serial.Serial(port, baud_rate)
    print(bluetooth_serial.name)
    print(f"Connected to port {port} at {baud_rate} baud.")

    if __name__ == "__main__":
        threads = []
        
        #dispatch threads
        rx_thread = threading.Thread(target=handle_Rx, args=(bluetooth_serial,) )
        threads.append(rx_thread)
        
            
        tx_thread = threading.Thread(target=handle_Tx, args=(bluetooth_serial,))
        threads.append(tx_thread)

        rx_thread.start()
        tx_thread.start()
            

        for t in threads:
            t.join()

        print("All threads finished")

except serial.SerialException as e:
    print(f"Error: {e}")


finally:
    # Close the serial port
    if 'bluetooth_serial' in locals() and bluetooth_serial.is_open:
        bluetooth_serial.close()
        print("Serial port closed")
        
        
        
    

        
