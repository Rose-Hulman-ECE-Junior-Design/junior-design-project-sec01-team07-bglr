import threading
import time
import csv
import serial
import time

# Skeleton User Controller framework
# generally, dispatch one thread to handle data reception
# and one thread to handle sending user commands.

print("Hello CK!")


port = 'COM11'
data_file = "dataLog_test1.csv"

num_bytes = 21

bluetooth_serial = 0

time_step = 0.5

running_time = 0

bluetooth_serial = serial.Serial(port)
serial_lock = threading.Lock()


#=========================================================================================
""" Separate the numbers out from the response string"""
def parse_dataLog(response):
    values = response.split(":")

    #print(values)
    
    current = float(values[0])             # current in mA
    voltage = float(values[1])             # voltage in V
    power = current * voltage              # power in mW
    #state = float(values[2])               # state enumeration

    

    energy = power * time_step

    global running_time

    print("TIME (s): " + str(running_time))
    print("Current (mA): " + str(current))
    print("Voltage (V): " + str(voltage))
    print("Power (mW): " + str(power))
    print("Energy (J):" + str(energy))

    log = [running_time, voltage, current, power, energy]
    
    add_to_csv(data_file, log)
    
    running_time += time_step


def add_to_csv(filepath, row_data):
    """Appends a list of values as a new row into the given CSV file."""
    with open(filepath, mode="a", newline="") as file:  # "a" = append mode
        writer = csv.writer(file)
        writer.writerow(row_data)
        
        
#=========================================================================================
""" Reads power data logs from ESP32"""
def handle_Rx():
    print("Starting Rx thread.")
    while 1:
        with serial_lock:
            response = bluetooth_serial.read(num_bytes)
        response_str = str(response)[2:]        #strip out the first two characters "b'"
        
        #print(response_str)
        print('====================')
        
        # parse the response
        parse_dataLog(response_str)
    
        #TODO: if connection is ever lost, break
    
    
#=========================================================================================    
""" Sends user commands to the ESP32"""
def handle_Tx():
    print("Starting Tx thread.")
    while 1:
        command = input("What would you like to send? ")
        print("Sending:" + command)
        
        # serial.write wants stuff in bytes
        # for future reference, can do b"Stop" or b"Start"
        with serial_lock:
            bluetooth_serial.write(command.encode('utf-8'))
        
        #TODO: if connection is ever lost, break
    

#=========================================================================================

    
try:
    #attempt to establish a serial connection
    #bluetooth_serial = serial.Serial(port, baud_rate)
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
        #tx_thread.start()
            

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
        
        
        
    

        
