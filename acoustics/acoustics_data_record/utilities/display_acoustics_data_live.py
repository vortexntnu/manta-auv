#!/usr/bin/env python3

# Libraries for file manipulation
import glob
import os

import matplotlib.pyplot as plt

# Libraries for handling data structures
import pandas as pd

# Libraries for animation
from matplotlib import animation, gridspec

# Variables for setting upp data structures correctly
HYDROPHONE_DATA_SIZE = (2**10) * 3  # 1 hydrophone buffer is 2^10 long, Each hydrophone data has 3 buffers full of this data
DSP_DATA_SIZE = 2**10  # DSP (Digital Signal Processing) has 2^10 long data
TDOA_DATA_SIZE = 5  # TDOA (Time Difference Of Arrival) has 5 hydrophones it has times for
POSITION_DATA_SIZE = 3  # position only has X, Y, Z basically 3 elements

# Important variables for later processing of data
SAMPLE_RATE = 430_000  # 430 kHz
MAX_FREQUENCY_TO_SHOW = 60_000  # 60 kHz
FPS = 1

# Make a good plot layout ==================================================
fig = plt.figure()
# Create an outer GridSpec for the two columns
outer_gs = gridspec.GridSpec(1, 2, figure=fig, width_ratios=[1, 1])
# Create an inner GridSpec for the first column
gs_hydrophone = gridspec.GridSpecFromSubplotSpec(5, 1, subplot_spec=outer_gs[0], hspace=0.1)
# Create an inner GridSpec for the second column, with height ratios for the 70%/30% split
gs_dsp = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=outer_gs[1], height_ratios=[7, 3], hspace=0.3)

hydrophoneAxis = [None] * 5

# Add subplots in the first column for hydrophone data
for i in range(5):
    hydrophoneAxis[i] = fig.add_subplot(gs_hydrophone[i, 0], sharex=hydrophoneAxis[0] if i else None)
    hydrophoneAxis[i].label_outer()
fig.text(0.25, 0.965, "Hydrophone Data", ha="center")

# Add subplots in the second column
FFTAxis = fig.add_subplot(gs_dsp[0])
filterAxis = fig.add_subplot(gs_dsp[1])

# Plot type so that the size is dynamic
plt.tight_layout()

# Select nice color pallet for graphs
colorSoftPurple = (168 / 255, 140 / 255, 220 / 255)
colorSoftBlue = (135 / 255, 206 / 255, 250 / 255)
colorSoftGreen = (122 / 255, 200 / 255, 122 / 255)

# .CSV Setup ==================================================
# Get Directory of the .csv files
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACOUSTICS_CSV_FILE_DIR = PACKAGE_DIR + "/acoustics_data"

# List of all the acoustic files
acousticsCSVFiles = csv_files = glob.glob(ACOUSTICS_CSV_FILE_DIR + "/acoustics_data_" + "*.csv")

# Get the latest csv file name for acoustics data
acousticsCSVFile = max(acousticsCSVFiles, key=os.path.getctime)


def convert_pandas_object_to_int_array(pandas_object: pd.Series) -> list:
    """
    Convert a pandas object containing a string representation of an integer array to a list of integers.

    Args:
        pandasObject (pandas.Series): A pandas Series object containing a string representation of an integer array.

    Returns:
        list: A list of integers extracted from the pandas object.
    """
    pandas_string = pandas_object.iloc[0].strip("array('i', ").rstrip(")")
    pandas_int_array = [int(x.strip()) for x in pandas_string.strip("[]").split(",")]

    return pandas_int_array


def convert_pandas_object_to_float_array(pandas_object: pd.Series) -> list:
    """
    Convert a pandas object containing a string representation of a float array to a list of floats.

    Args:
        pandasObject (pandas.Series): A pandas Series object containing a string representation of a float array.

    Returns:
        list: A list of floats extracted from the pandas object.
    """
    pandas_string = pandas_object.iloc[0].strip("array('f', ").rstrip(")")
    pandas_float_array = [float(x.strip()) for x in pandas_string.strip("[]").split(",")]

    return pandas_float_array


def get_acoustics_data() -> list:
    """
    Retrieves and processes the latest acoustics data from a CSV file.

    This function reads the latest acoustics data from a specified CSV file and processes it to extract various
    data points including hydrophone data, unfiltered data, filtered data, FFT data, peaks data, TDOA data, and
    position data. The processed data is then returned in a structured format.

    Returns:
        list: A list containing the following processed data:
            - hydrophone1 (list of int): Data from Hydrophone 1.
            - hydrophone2 (list of int): Data from Hydrophone 2.
            - hydrophone3 (list of int): Data from Hydrophone 3.
            - hydrophone4 (list of int): Data from Hydrophone 4.
            - hydrophone5 (list of int): Data from Hydrophone 5.
            - unfilteredData (list of int): Unfiltered data, same as the first 1024 values of Hydrophone 1.
            - filteredData (list of int): Filtered response data.
            - FFTAmplitudeData (list of int): Amplitude data from FFT.
            - FFTFrequencyData (list of float): Frequency data corresponding to FFT amplitudes.
            - peaksAmplitudeData (list of int): Amplitude data of peaks.
            - peaksFrequencyData (list of int): Frequency data of peaks.
            - tdoaData (list of float): Time Difference of Arrival (TDOA) data.
            - positonData (list of float): Position data.

    Raises:
        Exception: If there is an error reading the acoustics data or processing the DSP data.
    """
    # Variables that will be filled with latest acoustics data ----------
    hydrophone1 = [0] * HYDROPHONE_DATA_SIZE
    hydrophone2 = [0] * HYDROPHONE_DATA_SIZE
    hydrophone3 = [0] * HYDROPHONE_DATA_SIZE
    hydrophone4 = [0] * HYDROPHONE_DATA_SIZE
    hydrophone5 = [0] * HYDROPHONE_DATA_SIZE

    unfiltered_data = [0] * DSP_DATA_SIZE
    filtered_data = [0] * DSP_DATA_SIZE
    fft_data = [0] * DSP_DATA_SIZE
    peaks_data = [0] * DSP_DATA_SIZE
    fft_amplitude_data = [0] * DSP_DATA_SIZE
    fft_frequency_data = [0] * DSP_DATA_SIZE
    peaks_amplitude_data = [0] * DSP_DATA_SIZE
    peaks_frequency_data = [0] * DSP_DATA_SIZE

    tdoa_data = [0.0] * TDOA_DATA_SIZE
    positon_data = [0.0] * POSITION_DATA_SIZE

    # Read latest acoustics data ----------
    acoustics_data_frame = pd.read_csv(acousticsCSVFile)
    latest_acoustics_data = acoustics_data_frame.tail(1)

    try:
        # Get latest hydrophone data
        hydrophone1 = convert_pandas_object_to_int_array(latest_acoustics_data["Hydrophone1"])
        hydrophone2 = convert_pandas_object_to_int_array(latest_acoustics_data["Hydrophone2"])
        hydrophone3 = convert_pandas_object_to_int_array(latest_acoustics_data["Hydrophone3"])
        hydrophone4 = convert_pandas_object_to_int_array(latest_acoustics_data["Hydrophone4"])
        hydrophone5 = convert_pandas_object_to_int_array(latest_acoustics_data["Hydrophone5"])

        # Unfiltered data is special as it is the same as Hydrohone 1 first 1024 values
        # This is because Acoustics PCB uses Hydrophone 1 to perform DSP
        # Hydrohones have a ring buffer the size of 3 buffers each containing 1024 values (2^10)
        # We always use the first ring buffer of Hydrophone 1 to perform DSP
        # That is why unfiltered data is the same as Hydrphne 1 first buffer
        unfiltered_data = hydrophone1[0:1024]

        # Get DSP data
        filtered_data = convert_pandas_object_to_int_array(
            latest_acoustics_data["FilterResponse"]
        )  # Also known as Filter response to the raw unfiltered data
        fft_data = convert_pandas_object_to_int_array(latest_acoustics_data["FFT"])
        peaks_data = convert_pandas_object_to_int_array(latest_acoustics_data["Peaks"])

        # Get multilateration data
        tdoa_data = convert_pandas_object_to_float_array(latest_acoustics_data["TDOA"])
        positon_data = convert_pandas_object_to_float_array(latest_acoustics_data["Position"])
    except Exception as e:
        print(f"ERROR: Couldn't read acoustics data. Exception: {e}")

    # Post process DSP data to desired scale and amount ----------
    # 1. Convert FFTData to its corresponding frequency amount
    # 2. Cut out big FFT frequencies out as they are not relevant
    # 3. Cut out big peak frequencies as they are not relevant
    sample_length = len(fft_data)
    max_frequency_index = int(MAX_FREQUENCY_TO_SHOW * sample_length / SAMPLE_RATE)

    fft_amplitude_data = fft_data[0:max_frequency_index]
    fft_frequency_data = [(i * (SAMPLE_RATE / sample_length)) for i in range(sample_length)]
    fft_frequency_data = fft_frequency_data[0:max_frequency_index]

    # Peaks data is special as each peak data value is a array of [Amplitude, Frequency, Phase] of the peak
    # We want to get amplitude and frequency, dont really care about the phase
    try:
        temp_amplitude = []
        temp_frequency = []
        for peak_index in range(1, len(peaks_data), 3):
            if peaks_data[peak_index] < MAX_FREQUENCY_TO_SHOW:
                temp_amplitude += [peaks_data[peak_index - 1]]
                temp_frequency += [peaks_data[peak_index]]

        peaks_amplitude_data = temp_amplitude
        peaks_frequency_data = temp_frequency
    except Exception as e:
        print(f"ERROR processing DSP data. Exception: {e}")

    # return processed data ----------
    return [
        hydrophone1,
        hydrophone2,
        hydrophone3,
        hydrophone4,
        hydrophone5,
        unfiltered_data,
        filtered_data,
        fft_amplitude_data,
        fft_frequency_data,
        peaks_amplitude_data,
        peaks_frequency_data,
        tdoa_data,
        positon_data,
    ]


def display_live_data() -> None:
    """
    Display live acoustics data by plotting hydrophone data, filter response, and FFT data.

    Retrieves the latest acoustics data and separates it into hydrophone data, unfiltered data,
    filtered data, FFT amplitude and frequency data, and peak amplitude and frequency data.
    Plots the hydrophone data, filter response, and FFT data using predefined axes and colors.
    Also prints out unused multilateration data (TDOA and position data).

    Acoustics data structure:
        - acousticsData[0-4]: Hydrophone data for hydrophones 1 to 5
        - acousticsData[5]: Unfiltered data
        - acousticsData[6]: Filtered data
        - acousticsData[7]: FFT amplitude data
        - acousticsData[8]: FFT frequency data
        - acousticsData[9]: Peaks amplitude data
        - acousticsData[10]: Peaks frequency data
        - acousticsData[11]: TDOA data (currently not in use)
        - acousticsData[12]: Position data (currently not in use)

    Note:
        This function assumes that `getAcousticsData`, `hydrophoneAxis`, `filterAxis`, `FFTAxis`,
        `colorSoftBlue`, `colorSoftGreen`, and `colorSoftPurple` are defined elsewhere in the code.
    """
    # Get latest acoustics data
    acoustics_data = get_acoustics_data()

    # Set the latest acoustics data in appropriate variables
    hydrophone_data = [
        acoustics_data[0],  # Hydrophone 1
        acoustics_data[1],  # Hydrophone 2
        acoustics_data[2],  # Hydrophone 3
        acoustics_data[3],  # Hydrophone 4
        acoustics_data[4],  # Hydrophone 5
    ]

    unfiltered_data = acoustics_data[5]

    filter_data = acoustics_data[6]
    fft_amplitude_data = acoustics_data[7]
    fft_frequency_data = acoustics_data[8]
    peaks_amplitude_data = acoustics_data[9]
    peaks_frequency_data = acoustics_data[10]

    tdoa_data = acoustics_data[11]  # Currently not in use
    position_data = acoustics_data[12]  # Currently not in use

    # Plot hydrophone data
    for hydrophone_index in range(5):
        x_hydrophone = list(range(len(hydrophone_data[hydrophone_index][::])))
        hydrophoneAxis[hydrophone_index].clear()
        hydrophoneAxis[hydrophone_index].plot(
            x_hydrophone,
            hydrophone_data[hydrophone_index],
            label=f"Hydrophone {hydrophone_index + 1}",
            color=colorSoftBlue,
            alpha=1,
        )
        hydrophoneAxis[hydrophone_index].legend(loc="upper right", fontsize="xx-small")

    # Plot Filter response
    x_raw = list(range(len(unfiltered_data)))
    x_filter = list(range(len(filter_data)))
    filterAxis.clear()
    filterAxis.set_title("Filter response")
    filterAxis.plot(x_raw, unfiltered_data, label="Raw", color=colorSoftBlue, alpha=0.5)
    filterAxis.plot(x_filter, filter_data, label="Filter", color=colorSoftGreen, alpha=0.7)
    filterAxis.legend(loc="upper right", fontsize="xx-small")

    # Plot FFT data
    FFTAxis.clear()
    FFTAxis.set_title("FFT")
    FFTAxis.set_xlabel("Frequency [Hz]")
    FFTAxis.set_ylabel("Amplitude")
    FFTAxis.bar(
        fft_frequency_data,
        fft_amplitude_data,
        label="FFT",
        color=colorSoftPurple,
        alpha=1,
        width=500,
    )
    FFTAxis.scatter(
        peaks_frequency_data,
        peaks_amplitude_data,
        label="Peaks",
        color="red",
        alpha=0.7,
        s=30,
        linewidths=1.4,
        marker="x",
    )
    FFTAxis.legend(loc="upper right", fontsize="xx-small")

    # Print out the unused Multilateration data
    print(f"TDOA Data: {tdoa_data}     |     Position Data: {position_data}")


# Plotting live data
ani = animation.FuncAnimation(fig, display_live_data, interval=1000 / FPS)
plt.show()
