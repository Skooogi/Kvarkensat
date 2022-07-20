#
#
# 'JLinker.py' pushes and pulls measured test signal data over RTT to AIS-SDR prototype.
# for debugging digital signal processing chain
#
#

import pylink           # for JLink and RTT session
import numpy as np
import time
import matplotlib.pyplot as plt


import lil_endian       # simple func. flips and parses the read data bytes

#
# VARIABLES TO MODIFY; Especially 'samples', 'bytes_per_smpl', although they are only DEFAULTS
#                      that will be used if values are not read from target device in initialization.
#
samples = 100                       # How many samples are processed per loop. DEFAULT
sleeptime = 10                      # How long a loop sleeps after sending data.
bytes_per_int = 2                   # as per int16_t now, should be float though. DEFAULT
serial_num = 801003397              # either JLink or device specific (not sure, but works)
tgt_device = 'STM32H743ZI'

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


# Ask user to start RTT manually, since device has to be running first
print(f"\nMAKE SURE DEVICE HAS INITIATED RTT (i.e. is running)")
user = input("Start RTT and sending data? y or n: ")
if user != 'y':     # User stops execution if they don't initiate RTT
    print("RTT not initiated. Stopping execution.")
    exit()
jlink.rtt_start()
print(f"RTT start")
print(f"RTT satus: {jlink.rtt_get_status()}")       # check RTT status, note intended typo

i_data = lil_endian.txt_reader("hI.txt")            # Saved signal from .txt file
q_data = lil_endian.txt_reader("hQ.txt")            # Saved signal from .txt file
i_bytes = lil_endian.ints2bytes(i_data, bytes_per_int, False)
q_bytes = lil_endian.ints2bytes(q_data, bytes_per_int, False)
total_data = len(i_data)                            # Total data length to keep cycling data in loop

# Read the 'specs' for RTT communication.
# Specs being number of samples to send at a time and how many bytes/sample.
specs_bytes = jlink.rtt_read(3, 6)
if len(specs_bytes) >= 6:
    specs = lil_endian.bytes2ints(specs_bytes, bytes_per_int, False)
    samples = specs[0]
    bytes_per_int = specs[1]
    sleeptime = specs[2]
    print(f"RECEIVED sample len: {samples} & bytes/int: {bytes_per_int} & sleeptime: {sleeptime}ms")
else:
    print(f"Using DEFAULT sample len: {samples} & bytes/int: {bytes_per_int} & sleeptime: {sleeptime}ms")

# Current implementation is a loop that sends test signal to RTT,
# sleeps for while (as DSP happens on µC), and then tries to read
# what the µC is sending back.
# Could maybe be done with threads instead.
while True:
    # Ask user what to do in loop
    user = input(f"SEND {samples} data smpls to RTT buffer? a(all) or x(int) or q(quit): ")

    # User input 'q' closes RTT session and stops execution of script
    if user == 'q' or user == 'quit' or user == 'n':
        jlink.rtt_stop()
        print("Stopping execution.\n")
        exit()

    # User input 'a' sends the maximum specified sample length
    if user == 'a' or user == "all":
        send_sams = samples

    # Other user inputs are interpreted as the number of samples t
    else:
        if int(user) >= samples:
            print("Over max samples RTT is configured to handle")
            send_sams = samples
        else:
            send_sams = int(user)

    #
    # SEND DATA:
    # 1.calculate last idx; 2.convert data to bytes; 3.write data to RTT buffer
    #
    last_idx = send_sams*bytes_per_int - 1 + 1  # One sacrificial byte must be sent to be missed by RTT read.
    print(f"Sending {send_sams} samples, {last_idx+1} bytes (one byte as sacrifice)")
    jlink.rtt_write(1, i_bytes[0:last_idx])     # write data (as bytes) to RTT down-buffer '1'
    jlink.rtt_write(2, q_bytes[0:last_idx])     # write data (as bytes) to RTT down-buffer '2'

    print(f"Sent data. Sleeping..")
    time.sleep(sleeptime/1000)
    print(f"Slept for {sleeptime/1000} seconds. New loop..")

    #
    # READ DATA:
    # 1.read rtt buffers; 2.convert bytes to data samples
    #
    read_bytes_i = jlink.rtt_read(1, send_sams * bytes_per_int)             # Read I data from RTT buffer '1'
    read_bytes_q = jlink.rtt_read(2, sens_sams * bytes_per_int)             # Read Q data from RTT buffer '2'
    read_i = lil_endian.bytes2ints(read_bytes_i, bytes_per_int, False)  # Bytes to integers
    read_q = lil_endian.bytes2ints(read_bytes_q, bytes_per_int, False)  # Bytes to integers

    #
    # PLOTTING
    # I data first
    sample_idxs = np.arange(1, len(read_i) + 1)  # 'samples' gives indexes to read data
    plt.figure()
    plt.plot(sample_idxs, read_i, '.b')
    plt.xlabel("sample")
    plt.ylabel("RTT Data [unit]")
    plt.title(f"I data from RTT, {len(read_i)} samples")
    # Q data next
    sample_idxs = np.arange(1, len(read_q) + 1)  # 'samples' gives indexes to read data
    plt.figure()
    plt.plot(sample_idxs, read_q, '.r')
    plt.xlabel("sample")
    plt.ylabel("RTT Data [unit]")
    plt.title(f"Q data from RTT, {len(read_q)} samples")

    print("Show plots..")
    plt.show()
    # After closing plots, go for a new user initiated loop
