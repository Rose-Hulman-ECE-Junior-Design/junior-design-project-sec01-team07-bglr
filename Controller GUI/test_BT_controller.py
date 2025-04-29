import threading
import time
import csv
import serial
import time

# Skeleton User Controller framework
# generally, dispatch one thread to handle data reception
# and one thread to handle sending user commands.

print("Hello CK!")


port = 'COM8'
data_file = "temp.csv"

num_bytes = 32
baud_rate = 115200
bluetooth_serial = None
serial_lock = threading.Lock()
add_to_log = True


time_step = 0.5
running_time = 0




#=========================================================================================
""" Separate the numbers out from the response string"""
def parse_dataLog(response):
    values = response.split(":")

    #print(values)
    
    current = float(values[0])             # current in mA
    voltage = float(values[1])             # voltage in V
    power = current * voltage              # power in mW
    state = float(values[2])              # state enumeration
    v_cap = float(values[3])
    log_num = int(values[4])

    # TODO: Scale v_cap back to its normal value

    energy = v_cap * v_cap * 1000 / 2  # turn power supply voltage to energy


    print("TIME (s): " + str(log_num *0.5))
    print("Current (mA): " + str(current))
    print("Voltage (V): " + str(voltage))
    print("Power (mW): " + str(power))
    print("Energy (J):" + str(energy))

    log = [log_num, voltage, current, power, energy, state]
    #      packet num, Voltage, Current, Power, Energy, State

    if add_to_log:
        add_to_csv(data_file, log)
    


def add_to_csv(filepath, row_data):
    """Appends a list of values as a new row into the given CSV file."""
    with open(filepath, mode="a", newline="") as file:  # "a" = append mode
        writer = csv.writer(file)
        writer.writerow(row_data)
        
        
#=========================================================================================
""" Reads power data logs from ESP32"""
def handle_Rx():
    print("Starting Rx thread.")
    global bluetooth_serial, serial_lock
    
    while 1:
        
        print("Reading data...")
        with serial_lock:
            response = bluetooth_serial.read(num_bytes)
            # response = bluetooth_serial.readline()
        
        if not response:
            print("No data received")
            continue
                 
        response_str = str(response)[2:]        #strip out the first two characters "b'"
        response_str = response_str[:-1]        #strip out the last character too "'"
        #print('====================')
        
        # parse the response
        parse_dataLog(response_str)


        # if the connection is lost, break
        if not bluetooth_serial.is_open:
            print("Error: Serial port is not open.")
            return
    
    
#=========================================================================================    
""" Sends user commands to the ESP32"""
def handle_Tx():
    print("Starting Tx thread.")
    global bluetooth_serial, serial_lock
    while 1:
        command = input("What would you like to send? ")
        print("Sending:" + command)
        
        # serial.write wants stuff in bytes
        # for future reference, can do b"Stop" or b"Start"
        with serial_lock:
            bluetooth_serial.write(command.encode('utf-8'))
        
        # if the connection is lost, break
        if not bluetooth_serial.is_open:
            print("Error: Serial port is not open.")
            return
    

#=========================================================================================

    
try:
    #attempt to establish a serial connection
    #bluetooth_serial = serial.Serial(port, baud_rate)
    bluetooth_serial = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
    print(bluetooth_serial.name)
    print(f"Connected to port {port} at {baud_rate} baud.")

    if __name__ == "__main__":
        threads = []
        
        #dispatch threads
        rx_thread = threading.Thread(target=handle_Rx )
        threads.append(rx_thread)
        
            
        tx_thread = threading.Thread(target=handle_Tx)
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
        
        
        
    

        
