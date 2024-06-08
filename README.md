# Advancing-Cardiac-Care-The-Future-of-Portable-ECG-Technology

This repository contains the source code and documentation for an ECG signal processing and RF communication protocol. The project involves a central controller and receiver, with RF settings configured for optimal data transmission to synchronize multiple EKG portable boards. Additionally, it includes Python scripts for denoising and preprocessing EKG signals.

Project Overview
This project focuses on developing a robust communication protocol for transmitting EKG data using RF technology. The communication protocol is implemented in C, while the EKG signal denoising and preprocessing are handled using Python.

**Repository Structure**

Controller.c: The C code for the central controller that orchestrates RF settings and manages the data transmission cycle using a round-robin method.

Portable Boards.c: The C code for the receiver, which stays in RX mode, buffers EKG readings to prevent data loss, and waits for an acknowledgment signal from the central controller.

RF_settings.h: The header file containing RF configuration settings essential for compatibility between the controller and receiver.

Report_Results.pdf: A comprehensive report detailing the full communication protocol, including the transmission cycle, RF settings, and data handling methods.

**Summary of Communication Protocol**
The communication protocol developed in this project ensures reliable transmission of EKG data using RF technology. The central controller uses a round-robin method to give each device an equal opportunity to transmit data, while the receiver stays in RX mode to buffer incoming EKG readings and wait for acknowledgment signals. This approach minimizes data collisions and ensures a smooth flow of information.

**Python Scripts for EKG Signal Processing**
The Python scripts provided in this repository are designed to denoise and preprocess EKG signals. These scripts implement advanced signal processing techniques to enhance the quality of EKG data before transmission.
