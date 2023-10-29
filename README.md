Battery Testing Equipment Project
Project Overview
This repository contains the code for a 3KW battery testing equipment comprising three main boards: Module Board, Mainframe Board, and Panel Board. The equipment is designed to facilitate comprehensive battery testing with high precision and flexibility.

Features
Modular Design: The equipment is composed of six Module Units, each capable of testing batteries in the voltage range of 0.5V to 150V and currents up to 60A. Modules can operate in Constant Current (CC) or Constant Resistance (CR) modes.

High-Frequency Profiling: The equipment supports profiling up to 1KHz, enabling detailed battery analysis.

Data Sampling: Data from each module is sampled at a rate of 1ms and stored in the hard storage memory unit.

Isolation and Parallel Operation: Modules are completely isolated from each other and can be operated in series or parallel to test batteries with higher profile data requirements.

User Interface: The equipment includes a Panel Board for user interaction and to set and monitor processes on the module.

Data Communication: The Mainframe Board gathers data from the module boards and sends it to a PC. It also receives written processes from users and applies them to the modules.

Project Structure
ModuleBoard/: Code for the Module Board.
MainframeBoard/: Code for the Mainframe Board.
PanelBoard/: Code for the Panel Board.
Getting Started
Provide instructions here on how to set up and run the code. Include any dependencies or hardware requirements.

Usage
The Battery Testing Equipment project comes with a user-friendly software interface for controlling the modules, monitoring data, and storing test results in CSV format on a PC. The software is an integral part of the project and allows users to perform the following tasks:

Configure Test Profiles: Users can create and configure test profiles, specifying parameters such as voltage, current, mode (CC or CR), and profiling frequency. This software enables users to design custom battery test plans to meet their specific needs.

Real-Time Monitoring: The software provides real-time monitoring of the battery testing process. Users can view voltage, current, and other relevant data as the test progresses. This feature helps in analyzing battery performance during testing.

Data Logging: All data collected during battery testing is automatically logged by the software. It stores the data in CSV format, making it easy to analyze and visualize the results. Users can define the location where the CSV files are saved for easy access.

Control Module Boards: The software allows users to control the individual Module Boards, setting them to operate in series or parallel configurations. Users can start, pause, or stop tests as needed and adjust parameters during testing.

User-Friendly Interface: The software features an intuitive and user-friendly interface with clear visualization of data, making it accessible for both experienced engineers and those new to battery testing.

Export Test Reports: Users can generate test reports summarizing the results and save them in PDF format. These reports can be shared with colleagues or stakeholders for further analysis or documentation.


Contact
Mahdiyar Raees Almohaddesin 
mahdiyar.raeesi@gmail.com
www.linkedin.com/in/mahdiyar-raees-almohaddesin-526b32159
