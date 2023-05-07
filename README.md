# STM32-Based-Quadcopter-Flight-Controller-Software
This is a quadcopter flight controller software developed by me for undergraduate graduation project. The project is written in C language and STM32CubeIDE was used. 

Flight Software Has The Following Features:
1) Angle Mode Flight
2) GPS & Altitude Hold Flight (In Order To Compansate Position and Altitude Drift For Image Processing)
3) Go To a Single GPS Point Function (Not Working So Properly Right Now)
4) Telemetry Module Sending the Status Data of Quad (My Teammate Will Share the Source Codes of The Ground Station Software Soon)
5) Fail-safe (In Case of Connection Loss Between MCU and Transmitter, Motors Stop)
6) Independent Watchdog Timer (So You Can Add Code and Check If It Will Exceed Normal Runtime of The Loop)
7) Calibration Processes for Magnetometer, Gyroscope&Accelerometer.
8) Battery Voltage Observation 
9) Driver for MPU6050, MS5611,U-Blox M8N GPS, HMC5883L Compass

Note: Detailed documentation, circuit schematics and a guidance about how you can deploy the code to your microcontroller will be added soon.
