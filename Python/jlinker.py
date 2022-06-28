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


serial_num = 801003397  # either JLink or device specific (not sure, but works)
tgt_device = 'STM32H743ZI'
samples = 5000          # Default value for number of samples
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


# Ask user to start RTT manually, since device has to be running first
print(f"\nMAKE SURE DEVICE HAS INITIATED RTT (i.e. is running)")
user = input("Start RTT and sending data? y or n: ")
if user != 'y':     # User stops execution if they don't initiate RTT
    print("RTT not initiated. Stopping execution.")
    exit()
jlink.rtt_start()
print(f"RTT start")
print(f"RTT satus: {jlink.rtt_get_status()}")       # intended typo

print(f"RTT up-buff 'Spx' desc: {jlink.rtt_get_buf_descriptor(1, True)}")
print(f"RTT down-buff I desc: {jlink.rtt_get_buf_descriptor(1, False)}")
print(f"RTT down-buff Q desc: {jlink.rtt_get_buf_descriptor(2, False)}")

#
# VARIABLES TO MODIFY, especially 'samples', 'bytes_per_int' might screw everything
#
read_bytes_spx = jlink.rtt_read(1, 2 * 2)               # Read sample specs from RTT buffer '3'
spx = lil_endian.byte_parser(read_bytes_spx, 2, True)   # Bytes to integers

if len(spx) == 2:
    samples = spx[0]                                    # How many samples are processed per loop.
    bytes_per_int = spx[1]                              # as per int16_t now, should be float though
    print(f"Samples now: {samples} and bytes_per_int {bytes_per_int}")

sleeptime = 10                                      # How long a loop sleeps after sending data.

i_data = lil_endian.txt_reader("hI.txt")            # Saved signal from .txt file
q_data = lil_endian.txt_reader("hQ.txt")            # Saved signal from .txt file

print(f"I data read from txt: {i_data[0:9]}..{i_data[-3:-1]}")
print(f"Q data read from txt: {q_data[0:9]}..{q_data[-3:-1]}")
# Current implementation is a loop that sends test signal to RTT,
# sleeps for while (as DSP happens on µC), and then tries to read
# what the µC is sending back.
# Should maybe be done with threads instead.
while True:
    user = input(f"SEND {samples} data smpls to RTT buffer? a(all) or x(int) or n(exit): ")
    # User input 'n' closes RTT session and stops execution of script
    if user == 'n':
        jlink.rtt_stop()
        print("Stopping execution.")
        exit()
    if user == 'a' or user == "all":
        send_sams = samples
    else:
        send_sams = int(user)

    last_idx = send_sams*bytes_per_int - 1
    i_bytes = lil_endian.bytes_from_data(i_data, bytes_per_int, False)
    q_bytes = lil_endian.bytes_from_data(q_data, bytes_per_int, False)
    # print(f"I data as bytes: {i_bytes[0:last_idx]}")
    # print(f"Q data as bytes: {q_bytes[0:last_idx]}")
    print(f"Sending {send_sams} samples, {last_idx+1} bytes")
    jlink.rtt_write(1, i_bytes[0:last_idx])  # write data (as bytes) to RTT down-buffer '1'
    jlink.rtt_write(2, q_bytes[0:last_idx])  # write data (as bytes) to RTT down-buffer '2'

    print(f"Sent data. Sleeping..")
    time.sleep(sleeptime)
    print(f"Slept for {sleeptime} seconds. New loop..")

    time.sleep(0.5)             # sleep atleast for half a second

    read_bytes_i = jlink.rtt_read(1, samples * bytes_per_int)               # Read I data from RTT buffer '1'
    read_bytes_q = jlink.rtt_read(2, samples * bytes_per_int)               # Read Q data from RTT buffer '2'
    read_i = lil_endian.byte_parser(read_bytes_i, bytes_per_int, False)     # Bytes to integers
    read_q = lil_endian.byte_parser(read_bytes_q, bytes_per_int, False)     # Bytes to integers

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

    plt.show()

