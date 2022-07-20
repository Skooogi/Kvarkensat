#
#
# 'uplinker.py' reads data for debugging from RTT session initiated by AIS-SDR prototype
#   1st version (Topi):
#   - Read data; either 'a'(all) or a certain number of points 'x'(int)
#   - Save data to a .csv file
#   - Plot data
#

import pylink           # for JLink and RTT session
import numpy as np
import matplotlib.pyplot as plt

import lil_endian       # simple func. flips and parses the read data bytes

serial_num = 801003397  # either JLink or device specific (not sure, but works)
tgt_device = 'STM32H743ZI'
bytes_per_int = 2       # Bytes per in one integer of data. int_16 = 2. uint_32 = 4.

# Initiate JLink session and print session info.
jlink = pylink.JLink()
jlink.open(serial_no=serial_num)
print(jlink.product_name)
print(f"S/N {jlink.serial_number}")
jlink.connect(tgt_device)
con = jlink.target_connected()
print(f"Target connected: {con}")
if not con:                         # if target device is not found, stop execution
    print("Target not found!")
    exit()

# Print device info. Could be seen unnecessary
print(f"Core name: {jlink.core_name()}")
print(f"Core ID: {jlink.core_id()}")
print(f"JLink index: {jlink.index}")
print(f"JTAG IRlen: {jlink.ir_len()}")

# Ask user to start RTT manually, since device has to be running first
print(f"\nMAKE SURE DEVICE HAS INITIATED RTT (i.e. is running)")
user = input("Start RTT? y or n: ")
if user != 'y':     # User stops execution if they don't initiate RTT
    print("RTT not initiated. Stopping execution.")
    exit()
jlink.rtt_start()
print(f"RTT start")
print(f"RTT satus: {jlink.rtt_get_status()}")       # intended typo
print(f"RTT buff descrips: {jlink.rtt_get_buf_descriptor(1,True)}")
print(f"RTT buff descrips: {jlink.rtt_get_buf_descriptor(1,False)}")
# Loop for reading the data from RTT up-buffer '1'
while True:
    # Ask user what to do in loop
    user = input("READ RTT buffer readings? a(all) or x(int) or q(quit): ")
    data = []

    # User input 'q' closes RTT session and stops execution of script
    if user == 'q' or user == 'quit' or user == 'n':
        jlink.rtt_stop()
        print("Stopping execution.")
        exit()

    # User input 'a' reads all data from RTT buffer
    elif user == 'a':
        readings_per_loop = 10
        total_data = 0
        # Reads the RTT buffer in batches of 10 readings per loop until buffer is empty
        while True:
            rtt_bytes = jlink.rtt_read(1, readings_per_loop * bytes_per_int)              # Read data in bytes from RTT
            # print(f"RTT read bytes: {rtt_bytes} \n",
            #       f"where there are {bytes_per_smpl} bytes per actual reading")
            new_data = lil_endian.bytes2ints(rtt_bytes, bytes_per_int, do_prints=False)  # Parse the bytes to data
            data_per_loop = len(new_data)
            # Check if buffer is empty (new_data is empty)
            if data_per_loop == 0:
                break   # break data reading loop
            # Append this loops readings to the total data list
            for i in range(0, data_per_loop):
                data.append(new_data[i])
        # Print out what was read from RTT up-buffer
        print(f"Data length: {len(data)}")
        print(f"All data parsed: {data}")

    # Any other user input is read as an integer for how many readings user wants
    else:
        print(f"Reading {int(user)} data readings. ")
        rtt_bytes = jlink.rtt_read(1, int(user) * bytes_per_int)
        # print(f"RTT read bytes: {rtt_bytes} \n",
        #       f"where there are {bytes_per_smpl} bytes per actual reading")
        data = lil_endian.bytes2ints(rtt_bytes, bytes_per_int, False)
        print(f"Data actually: {data}")

    # Ask whether user wants to save data to .csv file
    user = input("SAVE data? y or n: ")
    if user == 'y':
        lil_endian.csv_writer(data, filename="dataFromRTT")

    # Ask whether user wants to plot the read data
    user = input("PLOT data? y or n: ")
    if user == 'y':
        samples = np.arange(1, len(data)+1)     # 'samples' gives indexes to read data
        plt.figure()
        plt.plot(samples, data, '.k')
        plt.xlabel("sample")
        plt.ylabel("RTT Data [unit]")
        plt.title(f"Num. of integers read from RTT: {len(data)}")
        plt.show()

    # Restart loop for reading data!

