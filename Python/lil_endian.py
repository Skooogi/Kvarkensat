#
#
# 'lil_endian.py' functions for use with script 'JLinker.py'.
#   Need to install numpy (you should have it) and pandas (for nice .csv saves)
#
import numpy as np
import pandas


#
# Function and example for parsing bytes read by JLink RTT (-Topi)
#   - rtt_bytes     =  bytes read by jlink.rtt_read(..) in separate script
#   - bytes_per_smpl =  how many bytes per datatype read from RTT, i.e. int_16 => 2
#   - do_prints     =  Boolean input for whether the function should print its input and output
#
def bytes2ints(rtt_bytes: list[int], bytes_per_int: int, do_prints: bool) -> list[int]:
    if do_prints:
        print(f"'rtt_data' now: {rtt_bytes}")
    # create a new list to be overridden with data
    data = list(np.zeros(int(len(rtt_bytes) / bytes_per_int)))

    for i in range(0, len(data)):
        index = i * bytes_per_int
        data[i] = int.from_bytes(rtt_bytes[index:index + bytes_per_int], byteorder='little', signed=False)
        # print(rtt_bytes[index:index + bytes_per_smpl])

    if do_prints:
        print(f"'data' now: {data}")

    return data

# print(f"Example byte parsing:")
# rtt_bytes_exm = [11, 0, 22, 0, 144, 1, 32, 78, 255, 127]
# bytes_per_int_exm = 2
# byte_parser(rtt_bytes_exm, bytes_per_int_exm, True)


#
# Function for deconstructing i-q -data into bytes read by JLink RTT (-Topi)
#
def ints2bytes(data: list[int], bytes_per_int: int, do_prints: bool) -> list[int]:
    if do_prints:
        print(f"'data' now: {data}")
    # create a new list to be overridden with data
    rtt_bytes = list(np.zeros(bytes_per_int * int(len(data))))

    for i in range(0, len(data)):
        bytes_data_point = int.to_bytes(int(data[i]), bytes_per_int, byteorder='little', signed=False)
        for j in range(0, bytes_per_int):
            rtt_bytes[2*i+j] = bytes_data_point[j]
        # print(rtt_bytes[index:index + bytes_per_smpl])

    if do_prints:
        print(f"'rtt_bytes' now: {rtt_bytes}")

    return rtt_bytes


def bytes2cfloats(rtt_bytes: list[int], bytes_per_cfloat: int, do_prints: bool) -> list[float]:
    if do_prints:
        print(f"'rtt_data' now: {rtt_bytes}")
    # create a new list to be overridden with data
    real_data = list(np.zeros(int(len(rtt_bytes) / bytes_per_float)))
    imag_data = list(np.zeros(int(len(rtt_bytes) / bytes_per_float)))

    for i in range(0, len(data)):
        index = i * bytes_per_cfloat/2
        str = hex(int.from_bytes(rtt_bytes[index:index + bytes_per_cfloat/2], byteorder='little', signed=False))
        real_data[i] = (float.fromhex(str))
        # print(rtt_bytes[index:index + bytes_per_smpl])
        index = i * bytes_per_cfloat / 2 + bytes_per_cfloat/2
        str = hex(int.from_bytes(rtt_bytes[index:index + bytes_per_cfloat/2], byteorder='little', signed=False))
        imag_data[i] = (float.fromhex(str))
    if do_prints:
        print(f"'real_data' now: \n{real_data}\n'imag_data' now: \n{imag_data}\n")

    return [real_data, imag_data]


#
# Writing the data to a separate .csv-file
#
def csv_writer(data, filename: str):
    # Save measurement data as comma separated values .csv-files
    data_ls = []
    samples = np.arange(1, len(data)+1)
    for k in range(0, len(data)):
        data_row = [samples[k], data[k]]
        data_ls.append(data_row)

    data_frame = pandas.DataFrame(data_ls, columns=['sampleNum', 'data'])
    data_frame.to_csv(f'{filename}.csv')

    return


#
# Read the data from a separate .csv-file.
# Returns list of sample numbers 'samples' and list of data integers 'data'
#
def csv_reader(filename: str):
    # open file and read each line into array
    file1 = open(filename, "r")
    lines_full = file1.readlines()
    lines = lines_full[1:]              # drop the header lines
    n_of_lines = len(lines)             # now lines contains only measurement data
    samples = list(np.zeros(n_of_lines))  # empty list for samples
    data = samples.copy()                 # ..and data

    # Loop for reading sample number and data value from each line
    for k in range(0, n_of_lines):
        index_separated = lines[k].partition(',')
        s_separated = index_separated[2].partition(',')
        s = int(s_separated[0])
        data_separated = s_separated[2].partition(',')
        d = int(data_separated[0])
        samples[k] = s
        data[k] = d

    return samples, data


#
#   Reads a saved test signal from .txt files (file only contains data points)
#
def txt_reader(filename: str):
    # open file and read each line into array
    file1 = open(filename, "r")
    lines = file1.readlines()
    # Number of data points
    n_data = len(lines)
    # Take only measurement rtt_bytes
    data_lines = lines[1:n_data+1]
    # Prepare a zeros array for rtt_bytes
    data = np.zeros(n_data)

    # Read actual rtt_bytes from the data_lines into 'rtt_bytes' array
    for i in range(0, n_data):
        data[i] = int(lines[i])

    return data

