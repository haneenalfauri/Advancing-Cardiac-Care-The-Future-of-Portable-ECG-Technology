import matplotlib.pyplot as plt
import numpy as np 
import pandas as pd
import csv
from scipy import signal
from pykalman import KalmanFilter
from scipy.optimize import minimize
import math
import tensorflow as tf
from sklearn.model_selection import train_test_split
import neurokit2 as nk

# -----------------------------------------------------------------------------------------------------#
# Extract Data from CSV file into python list and flatten the data

y1 = []
y2 = []

IS_Lead_I = True

Lead_I_Truth = '0208_lead_truth.csv'
Lead_I_Board = '0208_lead_ecg_board.csv'

Lead_II_Truth = '0215 lead2 truth exp1.csv'
Lead_II_Board = '0215 lead2 snap ecg board exp1.csv'

Lead_II_Truth_2 = '0215 lead2 truth exp2.csv'
Lead_II_Board_2 = '0215 lead2 snap ecg board exp2.csv'

# Read from CSV file, filter empty rows
with open(Lead_I_Truth, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        if row[0] != '':
            y1.append(row)

with open(Lead_I_Board, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        if row[0] != '':
            y2.append(row)

# Flatten the data into a single list
ekgData_T = []
ekgData_B = []
raw_ekgData_T = []
raw_ekgData_B = []

# If electrodes were flipped during measurement, set to -1
Flipped = -1

# Lead Truth
for sublist in y1:
    for item in sublist:
        ekgData_T.append(float(item) * Flipped)
        raw_ekgData_T.append(float(item) * Flipped)

Len_ekgData_T = len(ekgData_T)

# Lead Board
for sublist in y2:
    for item in sublist:
        ekgData_B.append(float(item))
        raw_ekgData_B.append(float(item))

Len_ekgData_B = len(ekgData_B)

# Sampling rate
sr = 1000.0
# Time interval
ti = 1.0 / sr

x1 = np.arange(0, Len_ekgData_T / 1000, ti)
x2 = np.arange(0, Len_ekgData_B / 1000, ti)

R1 = 40000
R2 = 60000

# L1, L2 used to zoom in on sec L1 till second L2
L1 = 47
L2 = 59

# -----------------------------------------------------------------------------------------------------#
# Check for errors before proceeding
# Generate an error in case data/time of x and y axis do not align
if len(x1) != Len_ekgData_T or len(x2) != Len_ekgData_B:
    print('Error: x and y dimensions do not match')

def calculate_rmse(signal1, signal2):
    """
    Calculates the RMSE between two signals
    """
    squared_error = 0.0
    for i in range(len(signal1)):
        squared_error += (signal1[i] - signal2[i]) ** 2
    mean_squared_error = squared_error / len(signal1)
    rmse = math.sqrt(mean_squared_error)
    return rmse

print()
RMSE0 = calculate_rmse(ekgData_B, ekgData_T)
print(f"RMSE Original (RAW) : {RMSE0}")
print()

# ------------------------------------------------------------------------------------------------------#

# DC filtering: Offset removal using a 2nd order high-pass Butterworth filter
b, a = signal.butter(2, 0.5, 'highpass', fs=1000)

fig, axs = plt.subplots(2, 1)

axs[0].plot(x1[R1:R2], ekgData_T[R1:R2], label='Before DC Offset Truth', linewidth=0.7)
axs[0].plot(x2[R1:R2], ekgData_B[R1:R2], label='Before DC Offset Board', linewidth=0.7)
axs[0].set_xlim(L1, L2)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
axs[0].grid(True)
axs[0].legend()

# Apply DC Offset removal
ekgData_T[R1:R2] = signal.filtfilt(b, a, ekgData_T[R1:R2])
ekgData_B[R1:R2] = signal.filtfilt(b, a, ekgData_B[R1:R2])

axs[1].plot(x1[R1:R2], ekgData_T[R1:R2], label='After DC Offset Truth', linewidth=0.7)
axs[1].plot(x2[R1:R2], ekgData_B[R1:R2], label='After DC Offset Board', linewidth=0.7)
axs[1].set_xlim(L1, L2)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Amplitude')
axs[1].grid(True)
axs[1].legend()

fig.tight_layout()
plt.show()

# -----------------------------------------------------------------------------------------------------#
# Time alignment function

def time_align(Truth_sig, Board_sig):
    """
    Aligns two signals in time using cross-correlation.
    """
    Truth_sig = np.array(Truth_sig)
    Board_sig = np.array(Board_sig)
    
    cross_corr = np.correlate(Truth_sig, Board_sig, mode='full')
    max_corr_index = np.argmax(cross_corr)
    time_shift = (max_corr_index - len(Truth_sig) + 1) / 1000.0
    Board_sig_aligned = np.roll(Board_sig, max_corr_index)
    
    return list(Board_sig_aligned), time_shift

A, B = time_align(ekgData_T[R1:R2], ekgData_B[R1:R2])
print("Time shift is", B)

fig, axs = plt.subplots(2, 1)

axs[0].plot(x1[R1:R2], ekgData_T[R1:R2], label='Before Time Alignment Truth', linewidth=0.7)
axs[0].plot(x2[R1:R2], ekgData_B[R1:R2], label='Before Time Alignment Board', linewidth=0.7)
axs[0].set_xlim(50, 52)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
axs[0].grid(True)
axs[0].legend()

ekgData_B[R1:R2] = A

axs[1].plot(x1[R1:R2], ekgData_T[R1:R2], label='After Time Alignment Truth', linewidth=0.7)
axs[1].plot(x2[R1:R2], ekgData_B[R1:R2], label='After Time Alignment Board', linewidth=0.7)
axs[1].set_xlim(50, 52)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Amplitude')
axs[1].grid(True)
axs[1].legend()

fig.tight_layout()
plt.show()

print()
RMSE1 = calculate_rmse(ekgData_B, ekgData_T)
print(f"RMSE After Time Alignment & DC offset : {RMSE1}")
percentage_improvement = 100 * (RMSE0 - RMSE1) / RMSE0
print(f"Improvement Percentage: {percentage_improvement} % ")

# -----------------------------------------------------------------------------------------------------#
# IIR filter (filter out 50 Hz and 60Hz noise)
def iir_bandstop(input_list, filter_order, window):
    """
    Applies a bandstop IIR filter to the input list.
    """
    sos = signal.iirfilter(filter_order, window, rs=60, btype='bandstop', analog=False, ftype='cheby2', fs=1000, output='sos')
    return signal.sosfilt(sos, input_list)

ekgFiltered1_T = iir_bandstop(ekgData_T[R1:R2], 17, [49, 51])
ekgFiltered1_B = iir_bandstop(ekgData_B[R1:R2], 17, [49, 51])
ekgFiltered2_T = iir_bandstop(ekgFiltered1_T, 17, [59, 61])
ekgFiltered2_B = iir_bandstop(ekgFiltered1_B, 17, [59, 61])

# -----------------------------------------------------------------------------------------------------#
# Apply a 3-pole lowpass filter at 0.1x Nyquist frequency

b, a = signal.butter(3, 0.1)
ekgFiltered3_T = signal.filtfilt(b, a, ekgFiltered2_T)
ekgFiltered3_B = signal.filtfilt(b, a, ekgFiltered2_B)

# -----------------------------------------------------------------------------------------------------#
# Linear Kalman Filter for noise removal

def kalman_filter(input_list):
    """
    Applies a Kalman filter to the input list.
    """
    kf = KalmanFilter(transition_matrices=[1],
                      observation_matrices=[1],
                      initial_state_mean=input_list[0],
                      initial_state_covariance=0.5,
                      observation_covariance=0.5,
                      transition_covariance=0.02)
    state_means, _ = kf.filter(input_list)
    return np.squeeze(state_means).tolist()

ekgFiltered4_T = kalman_filter(ekgFiltered3_T)
ekgFiltered4_B = kalman_filter(ekgFiltered3_B)

# -----------------------------------------------------------------------------------------------------#
# Check point: Plot the results

fig, axs = plt.subplots(2, 1)

axs[0].plot(x1[R1:R2], ekgData_T[R1:R2], label='Before Denoising Truth', linewidth=0.7)
axs[0].plot(x2[R1:R2], ekgData_B[R1:R2], label='Before Denoising Board', linewidth=0.7)
axs[0].set_xlim(50, 52)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
axs[0].grid(True)
axs[0].legend()

axs[1].plot(x1[R1:R2], ekgFiltered4_T, label='After Denoising Truth', linewidth=0.7)
axs[1].plot(x2[R1:R2], ekgFiltered4_B, label='After Denoising Board', linewidth=0.7)
axs[1].set_xlim(50, 52)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Amplitude')
axs[1].grid(True)
axs[1].legend()

fig.tight_layout()
plt.show()

# -----------------------------------------------------------------------------------------------------#
# Function definitions for P, R, T peak detection and correction

def find_onsets(ecg_delineate, function):
    """
    Finds the onset indices in the ECG signal.
    """
    return [i for i, x in enumerate(ecg_delineate[0][function]) if x == 1]

def find_offsets(ecg_delineate, function):
    """
    Finds the offset indices in the ECG signal.
    """
    return [i for i, x in enumerate(ecg_delineate[0][function]) if x == 1]

def find_peak_indices(ecg_processed, function):
    """
    Finds the peak indices in the ECG signal.
    """
    return [i for i, x in enumerate(ecg_processed[0][function]) if x == 1]

def check_fix(onset, offset):
    """
    Ensures onset and offset lists are of equal length.
    """
    if len(onset) == len(offset) - 1:
        return onset, offset[1:]    
    elif len(onset) == len(offset) + 1:
        return onset[:len(onset) - 1], offset
    elif len(onset) != len(offset):
        print("Mismatch in onset and offset lengths")
        print(f"Onset size: {len(onset)}")
        print(f"Offset size: {len(offset)}")
    return onset, offset

def plot_graph(signal_T, label_T, signal_B, label_B, x1, x2, L1, L2, ekgFiltered4_T, ekgFiltered4_B):
    """
    Plots comparison graphs for the given signals.
    """
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(x1, ekgFiltered4_T, label='Truth before', linewidth=0.7)
    axs[0].plot(x2, ekgFiltered4_B, label='Board before', linewidth=0.7)
    axs[0].set_xlim(L1, L2)
    axs[0].set_xlabel('Time')
    axs[0].set_ylabel('Amplitude')
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(x1, signal_T, label=label_T, linewidth=0.7)
    axs[1].plot(x2, signal_B, label=label_B, linewidth=0.7)
    axs[1].set_xlim(L1, L2)
    axs[1].set_xlabel('Time')
    axs[1].set_ylabel('Amplitude')
    axs[1].grid(True)
    axs[1].legend()

    fig.tight_layout()
    plt.show()

def find_samples(indexes, signal):
    """
    Finds sample amplitudes for the given indexes.
    """
    return [signal[i] for i in indexes]

sampling_rate = 1000

Sig_T = [ekgFiltered4_T]
Sig_B = [ekgFiltered4_B]

# Detect waves indices for Board
for ecg in Sig_B:
    ecg_processed = nk.ecg_process(ecg, sampling_rate=sampling_rate, method='neurokit')
R_indices_B = find_peak_indices(ecg_processed, "ECG_R_Peaks")
P_indices_B = find_peak_indices(ecg_processed, "ECG_P_Peaks")
T_indices_B = find_peak_indices(ecg_processed, "ECG_T_Peaks")

for ecg in Sig_B:
    ecg_delineate = nk.ecg_delineate(ecg, rpeaks=None, sampling_rate=1000, method='cwt', show_type='all')

# R WAVE
R_onset_B = find_onsets(ecg_delineate, "ECG_R_Onsets")
R_offset_B = find_offsets(ecg_delineate, "ECG_R_Offsets")

# T WAVE
T_onset_B = find_onsets(ecg_delineate, "ECG_T_Onsets")
T_offset_B = find_offsets(ecg_delineate, "ECG_T_Offsets")

# P WAVE
P_onset_B = find_onsets(ecg_delineate, "ECG_P_Onsets")
P_offset_B = find_offsets(ecg_delineate, "ECG_P_Offsets")

# Detect waves indices for Truth
for ecg in Sig_T:
    ecg_processed = nk.ecg_process(ecg, sampling_rate=sampling_rate, method='neurokit')
R_indices_T = find_peak_indices(ecg_processed, "ECG_R_Peaks")
P_indices_T = find_peak_indices(ecg_processed, "ECG_P_Peaks")
T_indices_T = find_peak_indices(ecg_processed, "ECG_T_Peaks")

for ecg in Sig_T:
    ecg_delineate = nk.ecg_delineate(ecg, rpeaks=None, sampling_rate=1000, method='cwt', show_type='all')

# R WAVE
R_onset_T = find_onsets(ecg_delineate, "ECG_R_Onsets")
R_offset_T = find_offsets(ecg_delineate, "ECG_R_Offsets")

# T WAVE
T_onset_T = find_onsets(ecg_delineate, "ECG_T_Onsets")
T_offset_T = find_offsets(ecg_delineate, "ECG_T_Offsets")

# P WAVE
P_onset_T = find_onsets(ecg_delineate, "ECG_P_Onsets")
P_offset_T = find_offsets(ecg_delineate, "ECG_P_Offsets")

# -----------------------------------------------------------------------------------------------------#
# Adjust and scale wave amplitudes

def scale_wave_amplitudes(wave1, wave2, onset_idx_list1, offset_idx_list1, onset_idx_list2, offset_idx_list2):
    """
    Scales wave amplitudes between two signals based on onset and offset indices.
    """
    R = min(len(onset_idx_list1), len(offset_idx_list1), len(onset_idx_list2), len(offset_idx_list2))
    for i in range(R):
        onset_idx1, offset_idx1 = onset_idx_list1[i], offset_idx_list1[i]
        onset_idx2, offset_idx2 = onset_idx_list2[i], offset_idx_list2[i]
        
        subset1 = wave1[onset_idx1:offset_idx1+1]
        subset2 = wave2[onset_idx2:offset_idx2+1]

        max1, min1 = max(subset1), min(subset1)
        max2, min2 = max(subset2), min(subset2)
        
        if max1 == min1:
            scaling_factor = 1
        else:
            scaling_factor = (max2 - min2) / (max1 - min1)

        for i in range(onset_idx1, offset_idx1+1):
            wave1[i] *= scaling_factor

    return wave1

X = ekgFiltered4_B.copy()
Y = ekgFiltered4_T.copy()

# Clean the ECG signal
cleaned_signal = nk.ecg_clean(ekgFiltered4_T, sampling_rate=1000)

# Check the quality of the ECG signal
quality = nk.ecg_quality(ekgFiltered4_T, sampling_rate=1000, method="zhao2018", approach="fuzzy")

print()
print(f"ECG signal quality: {quality}")

scaled_wave1_P = scale_wave_amplitudes(X, Y, P_onset_B, P_offset_B, P_onset_T, P_offset_T)
scaled_wave1_T = scale_wave_amplitudes(scaled_wave1_P, Y, T_onset_B, T_offset_B, T_onset_T, T_offset_T)
scaled_wave1_R = scale_wave_amplitudes(scaled_wave1_T, Y, R_onset_B, R_offset_B, R_onset_T, R_offset_T)

Final_Output = scaled_wave1_R

def dc_fix(wave1, wave2, onset_idx_list1, offset_idx_list1, onset_idx_list2, offset_idx_list2):
    """
    Applies DC offset correction to the waves.
    """
    R = min(len(onset_idx_list1), len(offset_idx_list1), len(onset_idx_list2), len(offset_idx_list2))
    for i in range(R):
        onset_idx1, offset_idx1 = onset_idx_list1[i], offset_idx_list1[i]
        onset_idx2, offset_idx2 = onset_idx_list2[i], offset_idx_list2[i]
        
        subset1 = wave1[onset_idx1:offset_idx1+1]
        subset2 = wave2[onset_idx2:offset_idx2+1]

        if len(subset1) > 0:
            avg1 = sum(subset1) / len(subset1)
        else:
            avg1 = 0
            
        if len(subset2) > 0:
            avg2 = sum(subset2) / len(subset2)
        else:
            avg2 = 0

        if avg1 != 0:
            scaling_factor = avg2 / avg1
        else:
            scaling_factor = 0
    
        for i in range(onset_idx1, offset_idx1+1):
            wave1[i] *= scaling_factor
        
    return wave1

if IS_Lead_I:
    Final_Output = dc_fix(scaled_wave1_R, Y, T_offset_B, P_onset_B, T_offset_T, P_onset_T)

def helper(ekgFiltered4_B, indices):
    temp_sig = ekgFiltered4_B.copy()
    for j in indices:
        temp_sig[j] = ekgFiltered4_B[j] * 0.5
    return temp_sig

L1 = 50
L2 = 51.25

plot_graph(helper(ekgFiltered4_T, P_indices_T), 'Truth P_Peaks', helper(ekgFiltered4_B, P_indices_B), 'Board P_Peaks', x1[R1:R2], x2[R1:R2], L1, L2, ekgFiltered4_T, ekgFiltered4_B)
plot_graph(helper(ekgFiltered4_T, R_indices_T), 'Truth R_Peaks', helper(ekgFiltered4_B, R_indices_B), 'Board R_Peaks', x1[R1:R2], x2[R1:R2], L1, L2, ekgFiltered4_T, ekgFiltered4_B)
plot_graph(helper(ekgFiltered4_T, T_indices_T), 'Truth T_Peaks', helper(ekgFiltered4_B, T_indices_B), 'Board T_Peaks', x1[R1:R2], x2[R1:R2], L1, L2, ekgFiltered4_T, ekgFiltered4_B)

L1 = 50
L2 = 56

plot_graph(ekgFiltered4_T, 'Truth signal', Final_Output, 'Board_After_Mapping_PRT_Waves', x1[R1:R2], x2[R1:R2], L1, L2, ekgFiltered4_T, ekgFiltered4_B)

def average_difference(numbers):
    """
    Calculates the average difference between consecutive elements in a list.
    """
    diff_sum = sum(abs(numbers[i] - numbers[i+1]) for i in range(len(numbers) - 1))
    return diff_sum // (len(numbers) - 1)

RR_range = average_difference(R_indices_T)
RR_Time_interval = RR_range / sampling_rate
print('RR_Time_interval', RR_Time_interval, 'Sec')
print("Heart Rate = ", 60 / RR_Time_interval, 'PPM')

print()
RMSE2 = calculate_rmse(ekgFiltered4_B, ekgFiltered4_T)
print(f"RMSE After Denoising and filtering : {RMSE2}")
percentage_improvement = 100 * (RMSE1 - RMSE2) / RMSE1
print(f"Improvement Percentage: {percentage_improvement} % ")
percentage_improvement = 100 * (RMSE0 - RMSE2) / RMSE0
print(f"Cumulative Improvement Percentage: {percentage_improvement} % ")

print()
RMSE3 = calculate_rmse(Final_Output, ekgFiltered4_T)
print(f"RMSE After I/O Mapping: {RMSE3}")
percentage_improvement = 100 * (RMSE2 - RMSE3) / RMSE2
print(f"Improvement Percentage: {percentage_improvement} % ")
percentage_improvement = 100 * (RMSE0 - RMSE3) / RMSE0
print(f"Cumulative Improvement Percentage: {percentage_improvement} % ")

# -----------------------------------------------------------------------------------------------------#
# Moving average filter to smooth out the signal

def moving_average(input_list, window_size):
    """
    Applies a moving average filter to the input list.
    """
    output_list = []
    for i in range(len(input_list)):
        if i < window_size:
            output_list.append(input_list[i])
        else:
            window = input_list[i-window_size:i]
            average = sum(window) / window_size
            output_list.append(average)
    return output_list

ekgFiltered5_T = moving_average(ekgFiltered4_T, 30)
ekgFiltered5_B = moving_average(ekgFiltered4_B, 40)

# -----------------------------------------------------------------------------------------------------#
# Median filter for smoothing the signal while retaining sharp edges

def median_filter(input_list, window_size):
    """
    Applies a median filter to the input list.
    """
    output_list = []
    for i in range(len(input_list)):
        if i < window_size:
            output_list.append(input_list[i])
        else:
            window = input_list[i-window_size:i]
            window.sort()
            median = window[int(window_size/2)]
            output_list.append(median)
    return output_list

ekgFiltered4_T = median_filter(ekgFiltered3_T, 1)
ekgFiltered4_B = median_filter(ekgFiltered3_B, 1)

# -----------------------------------------------------------------------------------------------------#
# Check point: Plot the results

fig, axs = plt.subplots(2, 1)

axs[0].plot(x1[R1:R2], ekgFiltered3_T, label='Before Kalman Truth', linewidth=0.7)
axs[0].plot(x2[R1:R2], ekgFiltered3_B, label='Before Kalman Board', linewidth=0.7)
axs[0].set_xlim(L1, L2)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
axs[0].grid(True)
axs[0].legend()

axs[1].plot(x1[R1:R2], ekgFiltered4_T, label='After Kalman Truth', linewidth=0.7)
axs[1].plot(x2[R1:R2], ekgFiltered4_B, label='After Kalman Board', linewidth=0.7)
axs[1].set_xlim(L1, L2)
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Amplitude')
axs[1].grid(True)
axs[1].legend()

fig.tight_layout()
plt.show()

# -----------------------------------------------------------------------------------------------------#
# Plot and zoom into the x-axis Raw data and scaled/offset data for both board and truth

fig, axes = plt.subplots(2, 4, figsize=(15, 10))

axes[0, 0].plot(x1[R1:R2], raw_ekgData_T[R1:R2], color='black', linewidth=0.4)
axes[0, 0].set_xlim(L1, L2)
axes[0, 0].set_title("Raw Truth Data")

axes[1, 0].plot(x2[R1:R2], raw_ekgData_B[R1:R2], color='black', linewidth=0.4)
axes[1, 0].set_xlim(L1, L2)
axes[1, 0].set_title("Raw Board Data")

axes[0, 1].plot(x1[R1:R2], ekgData_T[R1:R2], color='blue', linewidth=0.3)
axes[0, 1].set_xlim(L1, L2)
axes[0, 1].set_title("Lead Truth (Offsetted/Scaled)")

axes[1, 1].plot(x2[R1:R2], ekgData_B[R1:R2], color='blue', linewidth=0.3)
axes[1, 1].set_xlim(L1, L2)
axes[1, 1].set_title("Lead Board (Offsetted/Scaled)")

axes[0, 2].plot(x1[R1:R2], ekgFiltered3_T, color='red', linewidth=0.3)
axes[0, 2].set_xlim(L1, L2)
axes[0, 2].set_title("Lead Truth (+/BPF/LPF)")

axes[1, 2].plot(x1[R1:R2], ekgFiltered3_B, color='red', linewidth=0.3)
axes[1, 2].set_xlim(L1, L2)
axes[1, 2].set_title("Lead Board (+/BPF/LPF)")

axes[0, 3].plot(x1[R1:R2], ekgFiltered4_T, color='deeppink', linewidth=0.3)
axes[0, 3].set_xlim(L1, L2)
axes[0, 3].set_title("Lead Truth (+/Median Filter)")

axes[1, 3].plot(x1[R1:R2], ekgFiltered4_B, color='deeppink', linewidth=0.3)
axes[1, 3].set_xlim(L1, L2)
axes[1, 3].set_title("Lead Board (+/Median Filter)")

plt.tight_layout()
plt.show()

ecg_signal = ekgFiltered3_B

# Define the bandpass filter parameters
nyquist_freq = 500
low_cutoff = 5
high_cutoff = 15
order = 2

# Apply the bandpass filter to extract the carrier signal
b, a = signal.butter(order, [low_cutoff/nyquist_freq, high_cutoff/nyquist_freq], btype='band')
carrier_signal = signal.filtfilt(b, a, ecg_signal)

# Rectify the carrier signal to get the absolute value
rectified_signal = np.abs(carrier_signal)

# Define the low-pass filter parameters
cutoff_freq = 5
order = 2

# Apply the low-pass filter to extract the envelope signal
b, a = signal.butter(order, cutoff_freq/nyquist_freq, btype='low')
envelope_signal = signal.filtfilt(b, a, rectified_signal)

# Magnify the envelope signal
magnification_factor = 3
magnified_envelope_signal = envelope_signal * magnification_factor

# Combine the magnified envelope signal with the original signal
combined_signal = ecg_signal + magnified_envelope_signal

# Plot the original signal and combined signal for comparison
t = np.arange(len(ecg_signal)) / 1000
plt.plot(t, ecg_signal, label='Original ECG signal')
#plt.plot(t, magnified_envelope_signal, label='Magnified envelope signal')
#plt.plot(t, envelope_signal, label='Envelope signal')
plt.plot(t, combined_signal, label='Combined signal')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.legend()
plt.show()

# Process the ECG signal using NeuroKit
ecg_signals, info = nk.ecg_process(ecg_signal, sampling_rate=1000)

# Create a dataframe from the processed signals
df = pd.DataFrame(ecg_signals)

# Add the information to the dataframe
df["Rpeaks"] = info["ECG_R_Peaks"]
df["Heart_Rate"] = info["Heart_Rate"]

# Detect the R-peaks in the ECG signal using the Pan-Tompkins algorithm
rpeaks = nk.ecg_peaks(df, method="pantompkins1985", correct_artifacts=True)

# Detect the P-waves by searching for the first peak before each R-peak
p_peaks = []
for rpeak in rpeaks["ECG_R_Peaks"]:
    # Search for the first peak before the R-peak in a window of 150ms
    window_start = int(rpeak - 0.15 * 1000)
    window_end = rpeak
    window_signal = ecg_processed[window_start:window_end]
    p_peak = nk.epochs_findpeaks(window_signal, method="peak", show=False)["Peaks"][0]
    
    # Magnify the P-wave peak amplitude by a factor of 3
    p_peak_amplitude = window_signal[p_peak] * 3
    
    # Add the magnified P-wave amplitude to the list of P-peaks
    p_peaks.append(p_peak_amplitude)

# Plot the detected P-peaks along with the original ECG signal
fig, axs = plt.subplots(2, 1)

axs[0].plot(x1[R1:R2], p_peaks, label='P Peaks', linewidth=0.7)
axs[0].plot(x2[R1:R2], ekgFiltered3_B, label='Original', linewidth=0.7)
axs[0].set_xlim(L1, L2)
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Amplitude')
axs[0].grid(True)
axs[0].legend()

fig.tight_layout()
plt.show()

print()
print('End')
