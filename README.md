# Washing Machine Control System

### Overview
A microcontroller-based system that simulates a real washing machine.  
Users can control **time**, **temperature**, and **spin speed (RPM)** using potentiometers, while sensors provide safety and monitoring features.

### Features
- Adjustable cycle time, temperature, and RPM
- Door lock and overload protection  
- RGB LED load indicator  
- 7-segment time display with countdown  
- Buzzer alerts for start and finish  
- Button debouncing and sensor filtering  

### Tech
- **Language:** C / C++  
- **Platform:** STM32 (Mbed OS)  
- **Components:** LDR, FSR, temperature sensor, RGB LED, buzzer  

### How to Run
1. Open the project in **Mbed Studio**.  
2. Connect the Breadboard & STM32 board. 
3. Flash 'main.cpp' to your STM32 board.  
4. Open the Serial Monitor and run the program. 
