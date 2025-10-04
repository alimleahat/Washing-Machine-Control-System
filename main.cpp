#include "mbed.h"
#include <cstdio>
#include <cmath>

// Possible System States
enum SystemState {
    OFF,
    IDLE,
    RUNNING
};

// Constants
const float FREQUENCY = 100.0f;           // RGB LED PWM frequency (Hz)
const float FSR_THRESHOLD = 0.1f;         // Force sensor serial output threshold (10%)
const float LDR_THRESHOLD = 15.0f;        // Light sensor serial output threshold (15%)
const float TEMP_THRESHOLD = 5.0f;        // Temperature change threshold (5°C)
const float DOOR_OPEN_THRESHOLD = 40.0f;  // Door open threshold (light > 40%)
const float TEMP_SENSOR_CALIBRATION = 0.5f;// Temperature calibration 
const float FILTER_ALPHA = 0.3f;          // Low-pass filter coefficient
const int DEBOUNCE_COUNT = 3;             // Button debounce count
const int NUM_SAMPLES = 5;                // Sensor averaging samples

// Load level thresholds
const float LOAD_LIGHT = 0.2f;            // Light load
const float LOAD_MEDIUM = 0.4f;           // Medium load
const float LOAD_HEAVY = 0.6f;            // Heavy load
const float LOAD_OVERLOAD = 0.7f;         // Overload condition

// Inputs
AnalogIn potRPM(PA_7);      // RPM Pot
AnalogIn potTemp(PA_6);     // Temp Pot
AnalogIn potTime(PA_5);     // Time Pot
AnalogIn fsrSensor(PA_1);   // FSR
AnalogIn tempSensor(PC_3);  // Temp sensor
AnalogIn ldrSensor(PC_2);   // LDR

DigitalIn powerButton(PC_10);        // Power button
DigitalIn startPauseButton(PC_11);   // Run Cycle button

// Outputs
BusOut segDis(PA_11, PA_12, PB_1, PB_14, PB_15, PB_12, PB_11);  // 7 segment display
PwmOut buzzer(PA_15);       // Buzzer
PwmOut rgbRed(PB_3);        // RGB LED (Red)
PwmOut rgbGreen(PB_4);      // RGB LED (Green)
PwmOut rgbBlue(PB_5);       // RGB LED (Blue)
DigitalOut redLED(PC_0);    // Door Open Led (Red LED)

// 7 Segment Digits
const int hexDis[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

// Global Variables
SystemState systemState = OFF;
bool doorOpenWarningActive = false;
bool overloadWarningActive = false;

// Button debouncing (to make sure system starts in off state)
int lastPowerState = 0;
int lastStartState = 0;

// Previous readings for to detect significant change
float prevFsr = -1.0f;
float prevLdr = -1.0f; 
float prevTempActual = -1.0f;
int prevRpm = -1;
int prevTemp = -1;
int prevTime = -1;

// Door state debouncing
int doorOpenCount = 0;
int doorClosedCount = 0;

// Functions
void setRGB(float r, float g, float b);
void playBeep(float freq, int duration_ms);
void setLoadLevelColor(float load);
void runCycleCountdown(int minutes);
bool hasSignificantChange(float newVal, float prevVal, float threshold);
void readAndProcessSensors();
void handleButtons();
void updateDisplay(int value);
bool isDoorOpen(float ldr);
bool isOverloaded(float load);
void powerOn();
void powerOff();
float readAveragedSensor(AnalogIn& sensor, float scale = 1.0f);

// Set RGB LED colours
void setRGB(float r, float g, float b) {
    rgbRed.write(r);
    rgbGreen.write(g);
    rgbBlue.write(b);
}

// Play a beep sound
void playBeep(float freq, int duration_ms) {
    buzzer.period(1.0f / freq);
    buzzer.write(0.5f);
    wait_us(duration_ms * 1000);
    buzzer.write(0.0f);
}

// Set RGB based on load level
void setLoadLevelColor(float load) {
    if (load < LOAD_LIGHT) {
        setRGB(0.0f, 1.0f, 0.0f);  // Green for light load
    } else if (load < LOAD_MEDIUM) {
        setRGB(0.5f, 1.0f, 0.0f);  // Yellowish for normal load
    } else if (load < LOAD_HEAVY) {
        setRGB(1.0f, 1.0f, 0.0f);  // Yellow for medium load
    } else if (load < LOAD_OVERLOAD) {
        setRGB(1.0f, 0.5f, 0.0f);  // Orange for heavy load
    } else {
        setRGB(1.0f, 0.0f, 0.0f);  // Red for overload
    }
}

// Read sensor with averaging to reduce noise
float readAveragedSensor(AnalogIn& sensor, float scale) {
    float sum = 0.0f;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += sensor.read() * scale;
        wait_us(5000); // 5ms delay between samples
    }
    return sum / NUM_SAMPLES;
}

// Running the wash cycle
void runCycleCountdown(int minutes) {
    int steps = minutes / 10;
    
    for (int i = steps; i > 0; --i) {
        updateDisplay(i);
        printf("⏳ Cycle countdown: %d0 minutes remaining\n", i);
    }
    
    updateDisplay(0);
    printf("✅ 🧼 Cycle complete!\n");
    
    // Completion beep
    for (int i = 0; i < 3; ++i) {
        playBeep(1000, 200);
        wait_us(200000);
    }
}

// Check for significant change in sensor readings
bool hasSignificantChange(float newVal, float prevVal, float threshold) {
    return (prevVal < 0 || fabs(newVal - prevVal) >= threshold);
}

// Check if door is open based on ldr
bool isDoorOpen() {
    return (ldrSensor > DOOR_OPEN_THRESHOLD);
}

// Check if washer is overloaded
bool isOverloaded(float load) {
    return (load > LOAD_OVERLOAD);
}

// Update 7 segment display
void updateDisplay(int value) {
    if (systemState == OFF) {
        segDis = 0x00;  // Display off when system off
    } else {
        segDis = hexDis[value % 10];
    }
}

// Power on the system
void powerOn() {
    systemState = IDLE;
    playBeep(600, 100);
    printf("🟢 [System On]\n");
    
    // Initialize PWM
    rgbRed.period(1.0f/FREQUENCY);
    rgbGreen.period(1.0f/FREQUENCY); 
    rgbBlue.period(1.0f/FREQUENCY);
}

// System power off
void powerOff() {
    systemState = OFF;
    playBeep(600, 100);
    printf("🔴 [System Off]\n");
    
    // Reset outputs
    setRGB(0, 0, 0);
    redLED = 0;
    updateDisplay(0);
    
    // Reset variables
    doorOpenWarningActive = false;
    overloadWarningActive = false;
}

// Sensor Processing
void readAndProcessSensors() {
    // Read potentiometers and scale values
    int rpm = (int)(potRPM.read() * 7.0f + 2.0f) * 100;  // 2-9 × 100
    int temp = (int)(potTemp.read() * 4.0f + 2.0f) * 10; // 2-6 × 10
    
    // Calculate time (N × 10 minutes)
    float potVal = potTime.read();
    int time = static_cast<int>(roundf(potVal * 8.0f)) + 1;
    time = time > 9 ? 9 : time;
    time *= 10;

    // Read sensors using averaging function
    float fsr = readAveragedSensor(fsrSensor);
    float ldr = readAveragedSensor(ldrSensor, 100.0f);
    float tempActual = readAveragedSensor(tempSensor, 330.0f) * TEMP_SENSOR_CALIBRATION;
    
    
    // Update display with current time setting in IDLE mode
    if (systemState == IDLE) {
        updateDisplay(time / 10);
    }
    
    // Check for significant changes in pots 
    if (hasSignificantChange(rpm, prevRpm, 50) || 
        hasSignificantChange(temp, prevTemp, 5) || 
        hasSignificantChange(time, prevTime, 5)) {
        
        printf("⚙️ RPM: %d | 🌡️ Temp Set: %d°C | ⏱️ Time: %d min\n", rpm, temp, time);
        prevRpm = rpm;
        prevTemp = temp;
        prevTime = time;
    }

    // Check for significant changes in sensor readings
    if (hasSignificantChange(fsr, prevFsr, FSR_THRESHOLD) || 
        hasSignificantChange(tempActual, prevTempActual, TEMP_THRESHOLD) || 
        hasSignificantChange(ldr, prevLdr, LDR_THRESHOLD)) {
        
        int displayTemp = round(tempActual);
        bool doorIsOpen = isDoorOpen();
        const char* doorStatus = doorIsOpen ? "Door Open" : "Door Closed";
        
        printf("📦 Load: %.2f | 🌡️ Temp: %d°C | 🚪 %s\n", fsr, displayTemp, doorStatus);
        prevFsr = fsr;
        prevTempActual = tempActual;
        prevLdr = ldr;
    }

    // Door State
    bool currentDoorReading = isDoorOpen();
    
    if (currentDoorReading) {
        doorOpenCount++;
        doorClosedCount = 0;
    } else {
        doorClosedCount++;
        doorOpenCount = 0;
    }
    
    // State changes only after consistent readings
    bool doorCurrentlyOpen = doorOpenWarningActive ? 
                           (doorClosedCount < DEBOUNCE_COUNT) : 
                           (doorOpenCount >= DEBOUNCE_COUNT);
                           
    if (doorCurrentlyOpen != doorOpenWarningActive) {
        doorOpenWarningActive = doorCurrentlyOpen;
        if (doorCurrentlyOpen && systemState == IDLE) {
            playBeep(700, 200);
        }
    }
    
    // Update door status LED when in IDLE state only 
    if (systemState == IDLE) {
        redLED = doorOpenWarningActive ? 1 : 0;
    }
    
    // Update RGB based on load level
    setLoadLevelColor(fsr);
    
    // Handle overload warning state changes
    bool currentlyOverloaded = isOverloaded(fsr);
    if (currentlyOverloaded != overloadWarningActive) {
        overloadWarningActive = currentlyOverloaded;
        if (currentlyOverloaded) {
            printf("❗⚠️ WARNING: 🧺 Washer overloaded!\n");
            playBeep(500, 100);
        } else {
            printf("✅ Load level acceptable\n");
        }
    }
}

// button presses
void handleButtons() {
    // Handle Power Button
    if (powerButton.read() == 0 && lastPowerState == 1) {
        if (systemState == OFF) {
            powerOn();
        } else {
            powerOff();
        }
        lastPowerState = 0;
    }
    
    if (powerButton.read() == 1) {
        lastPowerState = 1;
    }

    // Handle Start Button (only when system is on)
    if (startPauseButton.read() == 0 && lastStartState == 1 && systemState != OFF) {
        lastStartState = 0;
        
        // Only handle if in idle state
        if (systemState == IDLE) {
            // Check for door open and overload only at start
            if (doorOpenWarningActive) {
                printf("❌ Cannot start: Door is open! Close door first.\n");
                playBeep(300, 500);
            } 
            else if (overloadWarningActive) {
                printf("❌ Cannot start: Washer overloaded! Reduce load.\n");
                playBeep(300, 500);
            }
            else {
                // Get current settings
                int rpm = (int)(potRPM.read() * 7.0f + 2.0f) * 100;
                int temp = (int)(potTemp.read() * 4.0f + 2.0f) * 10;
                int time = (static_cast<int>(roundf(potTime.read() * 8.0f)) + 1) * 10;
                time = time > 90 ? 90 : time;
                
                printf("▶️ Starting wash cycle: %d RPM, %d°C, %d minutes\n", rpm, temp, time);
                systemState = RUNNING;
                playBeep(700, 100);
                
                // Run cycle
                runCycleCountdown(time);
                
                // Return to idle after completion
                systemState = IDLE;
                printf("⏹️ Cycle ended\n");
            }
        } else if (systemState == RUNNING) {
            printf("⚠️ Cycle already in progress\n");
            playBeep(500, 100);
        }
    }
    
    if (startPauseButton.read() == 1) {
        lastStartState = 1;
    }
}

int main() {
    // Initialize components
    buzzer.write(0.0f);
    setRGB(0.0f, 0.0f, 0.0f);
    redLED = 0;
    updateDisplay(0);
    
    printf("🔄 System starting in OFF state\n");
    
    while (true) {
        // Handle buttons even in off mode
        handleButtons();
        
        switch (systemState) {
            case OFF:
                // Wait for power button in OFF state
                wait_us(500000);
                break;
                
            case IDLE:
            case RUNNING:
                // Process sensors and update displays
                readAndProcessSensors();
                wait_us(100000);  // 100ms
                break;
        }
    }
}