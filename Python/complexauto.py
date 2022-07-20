#
#
# 'JLinker.py' pushes and pulls measured test signal data over RTT to AIS-SDR prototype.
# for debugging digital signal processing chain
#
#

import pylink           # for JLink and RTT session
import numpy as np
import cmath
import time
import matplotlib.pyplot as plt


import lil_endian       # simple func. flips and parses the read data bytes

#
# VARIABLES TO MODIFY; Especially 'samples', 'bytes_per_smpl', although they are only DEFAULTS
#                      that will be used if values are not read from target device in initialization.
#
samples = 800                       # How many samples are processed per loop. DEFAULT
sleeptime = 3000                    # How long a loop sleeps after sending data. [ms] DEFAULT
bytes_per_smpl = 8                  # as per int16_t now, should be float though. DEFAULT
bytes_per_sent = 2                  # as per int16_t, that are sent to STM32 microcontroller
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
i_bytes = lil_endian.ints2bytes(i_data, bytes_per_sent, False)
q_bytes = lil_endian.ints2bytes(q_data, bytes_per_sent, False)
len_total = len(i_data)                            # Total data length to keep cycling data in loop

# Read the 'specs' for RTT communication.
# Specs being number of samples to send at a time and how many bytes/sample.
specs_bytes = jlink.rtt_read(3, 6)
if len(specs_bytes) >= 6:
    specs = lil_endian.bytes2ints(specs_bytes, bytes_per_sent, False)
    samples = specs[0]
    bytes_per_smpl = specs[1]
    sleeptime = specs[2]
    print(f"RECEIVED sample/loop len: {samples} & bytes/smpl: {bytes_per_smpl} & sleeptime: {sleeptime}ms")
else:
    print(f"Using DEFAULT sample/loop len: {samples} & bytes/smpl: {bytes_per_smpl} & sleeptime: {sleeptime}ms")

# Current implementation is a loop that sends samples of signal to RTT,
# sleeps for while (as DSP happens on µC), and then tries to read
# what the µC is sending back. The data read back is saved in a cumulative
# buffer, which should eventually contain the complete test signal.
# Could maybe be done with threads instead.
while True:
    # Ask user what to do in loop
    user = input(f"SEND {len_total} total data smpls to RTT buffer? a(all) or x(int) or q(quit): ")

    # User input 'q' closes RTT session and stops execution of script
    if user == 'q' or user == 'quit' or user == 'n':
        jlink.rtt_stop()
        print("Stopping execution.\n")
        exit()

    # User input 'a' sends the maximum specified sample length
    if user == 'a' or user == "all":
        send_sams = len_total

    # Other user inputs are interpreted as the number of samples tto send
    else:
        if int(user) > len_total:
            print("Over max number of samples test signal has")
            send_sams = len_total
        else:
            send_sams = int(user)

    # Calculate number of loops to send data
    leftover_samples = send_sams % samples
    if leftover_samples == 0:
        loops = int(send_sams / samples)
    else:
        loops = int(np.floor(send_sams / samples))
    read_i = []
    read_q = []
    # LOOP for sending test signal in 'samples' length pieces
    for k in range(0, loops):
        #
        # SEND DATA:
        # 1.calculate last idx; 2.convert data to bytes; 3.write data to RTT buffer
        #
        last_idx = (k+1) * samples * bytes_per_sent - 1 + 1  # One sacrificial byte must be sent to be missed by RTT read

        print(f"Sending {samples} samples, sent {k*samples}/{send_sams}, loop {k+1}/{loops}")
        jlink.rtt_write(1, i_bytes[k * samples * bytes_per_sent:last_idx])  # write data (as bytes) to RTT down-buffer '1'
        jlink.rtt_write(2, q_bytes[k * samples * bytes_per_sent:last_idx])  # write data (as bytes) to RTT down-buffer '2'

        print(f"Sent data. Sleeping..")
        time.sleep(sleeptime / 1000)
        print(f"Slept for {sleeptime / 1000} seconds. New loop..")

        #
        # READ DATA:
        # 1.read rtt buffers; 2.convert bytes to data samples
        #
        read_bytes_i = jlink.rtt_read(1, samples * bytes_per_smpl)  # Read I data from RTT buffer '1'
        read_bytes_q = jlink.rtt_read(2, samples * bytes_per_smpl)  # Read Q data from RTT buffer '2'
        read_i_loop = lil_endian.bytes2ints(read_bytes_i, bytes_per_smpl, False)  # Bytes to integers
        read_q_loop = lil_endian.bytes2ints(read_bytes_q, bytes_per_smpl, False)  # Bytes to integers

        #
        # APPEND DATA to cumulative buffers
        #
        for j in range(0, len(read_i_loop)):
            read_i.append(read_i_loop[j])
        for j in range(0, len(read_q_loop)):
            read_q.append(read_q_loop[j])

    # SEND possible LEFTOVER data
    if leftover_samples > 0:
        print(f"Sending {leftover_samples} leftover samples, sent {loops*samples}/{send_sams}")
        # write data (as bytes) to RTT down-buffer '1' & '2'
        jlink.rtt_write(1, i_bytes[loops * samples * bytes_per_sent:len_total * bytes_per_sent])
        jlink.rtt_write(2, q_bytes[loops * samples * bytes_per_sent:len_total * bytes_per_sent])
        # Sleep a synchronized time with STM
        print(f"Sent data. Sleeping..")
        time.sleep(sleeptime / 1000)
        print(f"Slept for {sleeptime / 1000} seconds. New loop..")
        # Read data from RTT
        read_bytes_iq = jlink.rtt_read(1, leftover_samples * bytes_per_smpl)
        read_iq_loop = lil_endian.bytes2floats(read_bytes_iq, bytes_per_smpl, True)
        # Append data to cumulative buffers
        for j in range(0, len(read_i_loop)):
            read_i.append(read_i_loop[j])
        for j in range(0, len(read_q_loop)):
            read_q.append(read_q_loop[j])

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

