/*
 * ═══════════════════════════════════════════════════════════════════
 *  ESP32-WROOM THERMAL PID CONTROL SYSTEM - PROTEUS SIMULATION
 * ═══════════════════════════════════════════════════════════════════
 * 
 * Hardware Configuration (ESP32-WROOM 22-pin):
 * - BME280 Sensor (I2C):
 *   Pin 6 (VDDIO)  → 3.3V (ESP32 Pin 2)
 *   Pin 3 (SDI)    → GPIO 21 / SDA (ESP32 Pin 13)
 *   Pin 4 (SCK)    → GPIO 22 / SCL (ESP32 Pin 19)
 *   Pin 2 (CSB)    → GND (ESP32 Pin 1) - I2C MODE
 *   Pin 5 (SDO)    → GND (ESP32 Pin 1) - Address 0x76
 *
 * - Heater (12V Lamp): GPIO 25 (ESP32 Pin 12) → MOSFET
 * - Fan (12V Motor):   GPIO 26 (ESP32 Pin 13) → MOSFET
 * - Humidifier (LED):  GPIO 27 (ESP32 Pin 14) → MOSFET
 *
 * PID Control:
 * - Temperature Target: 40°C
 * - Humidity Target: 60%
 * - Sampling Rate: 10Hz (100ms)
 * ═══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ═══════════════════════════════════════════════════════════════════
//                     PIN DEFINITIONS (ESP32-WROOM)
// ═══════════════════════════════════════════════════════════════════
// ESP32-WROOM Pin Mapping (from your 22-pin module)
#define PIN_HEATER      25    // PWM output to Heater MOSFET (ESP32 Pin 12 - D25)
#define PIN_FAN         26    // PWM output to Fan MOSFET (ESP32 Pin 13 - D26)
#define PIN_HUMIDIFIER  27    // PWM output to Humidifier MOSFET (ESP32 Pin 14 - D27)
#define PIN_SDA         21    // I2C Data line (ESP32 Pin 13 - D13)
#define PIN_SCL         22    // I2C Clock line (ESP32 Pin 19 - D14)

// ═══════════════════════════════════════════════════════════════════
//                        PWM CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
#define PWM_FREQ        5000  // 5kHz PWM frequency
#define PWM_RESOLUTION  8     // 8-bit resolution (0-255)
#define PWM_CH_HEATER   0     // PWM Channel 0 for Heater
#define PWM_CH_FAN      1     // PWM Channel 1 for Fan
#define PWM_CH_HUMID    2     // PWM Channel 2 for Humidifier

// ═══════════════════════════════════════════════════════════════════
//                     CONTROL SETPOINTS (TARGETS)
// ═══════════════════════════════════════════════════════════════════
volatile float Setpoint_Temp = 40.0;      // Target Temperature: 40°C
volatile float Setpoint_Humidity = 60.0;  // Target Humidity: 60%

// ═══════════════════════════════════════════════════════════════════
//                      SENSOR READINGS (INPUTS)
// ═══════════════════════════════════════════════════════════════════
volatile float Input_Temp = 0.0;          // Current Temperature
volatile float Input_Humidity = 0.0;      // Current Humidity
volatile float Input_Pressure = 0.0;      // Current Pressure (optional)

// ═══════════════════════════════════════════════════════════════════
//                      ACTUATOR OUTPUTS (PWM)
// ═══════════════════════════════════════════════════════════════════
volatile float Output_Heater = 0.0;       // Heater PWM (0-255)
volatile float Output_Fan = 0.0;          // Fan PWM (0-255)
volatile float Output_Humidifier = 0.0;   // Humidifier PWM (0-255)

// ═══════════════════════════════════════════════════════════════════
//                        SAFETY LIMITS
// ═══════════════════════════════════════════════════════════════════
#define TEMP_MAX_SAFE   60.0   // Emergency shutoff at 60°C
#define TEMP_MIN_SAFE   5.0    // Minimum safe temperature
#define HUMID_MAX_SAFE  95.0   // Maximum safe humidity
#define HUMID_MIN_SAFE  10.0   // Minimum safe humidity

// ═══════════════════════════════════════════════════════════════════
//                    PID CONTROLLER CLASS
// ═══════════════════════════════════════════════════════════════════
class SimplePID {
private:
    float Kp, Ki, Kd;           // PID Gains
    float integral;             // Accumulated integral
    float prevError;            // Previous error (for derivative)
    float outputMin, outputMax; // Output limits
    float Ts;                   // Sampling time (seconds)

public:
    // Constructor
    SimplePID(float p, float i, float d, float minVal, float maxVal, float samplingTime) {
        Kp = p; 
        Ki = i; 
        Kd = d;
        outputMin = minVal; 
        outputMax = maxVal;
        Ts = samplingTime;
        integral = 0;
        prevError = 0;
    }

    // Compute PID output
    float compute(float setpoint, float measured) {
        // Calculate error
        float error = setpoint - measured;

        // 1. PROPORTIONAL TERM
        float P = Kp * error;

        // 2. INTEGRAL TERM (with anti-windup)
        integral += Ki * error * Ts;
        
        // Anti-windup: Prevent integral from growing too large
        if (integral > outputMax) integral = outputMax;
        if (integral < outputMin) integral = outputMin;

        // 3. DERIVATIVE TERM
        float D = Kd * (error - prevError) / Ts;

        // Calculate total output
        float output = P + integral + D;

        // Clamp output to limits
        if (output > outputMax) output = outputMax;
        if (output < outputMin) output = outputMin;

        // Store error for next iteration
        prevError = error;

        return output;
    }

    // Reset PID state
    void reset() {
        integral = 0;
        prevError = 0;
    }

    // Update PID gains on-the-fly
    void setTunings(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }
};

// ═══════════════════════════════════════════════════════════════════
//                      GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════
Adafruit_BME280 bme;                // BME280 sensor object
TaskHandle_t TaskTempHandle;        // Temperature control task
TaskHandle_t TaskHumidHandle;       // Humidity control task

// ═══════════════════════════════════════════════════════════════════
//              TASK 1: TEMPERATURE CONTROL LOOP
// ═══════════════════════════════════════════════════════════════════
void Task_TemperatureControl(void *pvParameters) {
    // Initialize PID controller for temperature
    // Parameters: Kp, Ki, Kd, Min, Max, Sampling Time
    // TUNE THESE VALUES FOR YOUR SYSTEM!
    SimplePID tempPID(20.0, 0.5, 5.0, 0, 255, 0.1);
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms period
    xLastWakeTime = xTaskGetTickCount();

    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  TEMPERATURE CONTROL TASK STARTED    ║");
    Serial.println("╚══════════════════════════════════════╝");

    for (;;) {
        // ─────────────────────────────────────────────────
        // STEP 1: READ SENSOR
        // ─────────────────────────────────────────────────
        Input_Temp = bme.readTemperature();

        // Check for sensor error
        if (isnan(Input_Temp)) {
            Serial.println("[ERROR] BME280 temperature reading failed!");
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ─────────────────────────────────────────────────
        // STEP 2: SAFETY CHECKS
        // ─────────────────────────────────────────────────
        
        // OVERHEAT PROTECTION
        if (Input_Temp > TEMP_MAX_SAFE) {
            ledcWrite(PWM_CH_HEATER, 0);      // FORCE HEATER OFF
            ledcWrite(PWM_CH_FAN, 255);       // FORCE FAN FULL SPEED
            Output_Heater = 0;
            Output_Fan = 255;
            
            Serial.println("╔════════════════════════════════════════╗");
            Serial.println("║  ⚠️  EMERGENCY: OVERTEMPERATURE!      ║");
            Serial.println("║  Heater: OFF | Fan: MAXIMUM           ║");
            Serial.println("╚════════════════════════════════════════╝");
            
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // UNDERTEMP PROTECTION
        if (Input_Temp < TEMP_MIN_SAFE) {
            ledcWrite(PWM_CH_HEATER, 255);    // FORCE HEATER FULL
            ledcWrite(PWM_CH_FAN, 0);         // TURN OFF FAN
            Output_Heater = 255;
            Output_Fan = 0;
            
            Serial.println("[SAFETY] Undertemperature! Heater: MAX");
            
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ─────────────────────────────────────────────────
        // STEP 3: COMPUTE PID CONTROL OUTPUT
        // ─────────────────────────────────────────────────
        float pidOutput = tempPID.compute(Setpoint_Temp, Input_Temp);

        // ─────────────────────────────────────────────────
        // STEP 4: ACTUATE (Split heating/cooling)
        // ─────────────────────────────────────────────────
        
        if (pidOutput > 0) {
            // NEED TO HEAT (temperature below setpoint)
            Output_Heater = pidOutput;
            Output_Fan = 0;
            ledcWrite(PWM_CH_HEATER, (int)Output_Heater);
            ledcWrite(PWM_CH_FAN, 0);
        } 
        else {
            // NEED TO COOL (temperature above setpoint)
            Output_Heater = 0;
            Output_Fan = abs(pidOutput);
            ledcWrite(PWM_CH_HEATER, 0);
            ledcWrite(PWM_CH_FAN, (int)Output_Fan);
        }

        // ─────────────────────────────────────────────────
        // STEP 5: WAIT FOR NEXT CYCLE (Precise timing)
        // ─────────────────────────────────────────────────
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ═══════════════════════════════════════════════════════════════════
//              TASK 2: HUMIDITY CONTROL LOOP
// ═══════════════════════════════════════════════════════════════════
void Task_HumidityControl(void *pvParameters) {
    // Initialize PID controller for humidity
    SimplePID humidPID(15.0, 0.3, 2.0, 0, 255, 0.1);
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms period
    xLastWakeTime = xTaskGetTickCount();

    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  HUMIDITY CONTROL TASK STARTED       ║");
    Serial.println("╚══════════════════════════════════════╝");

    for (;;) {
        // ─────────────────────────────────────────────────
        // STEP 1: READ SENSOR
        // ─────────────────────────────────────────────────
        Input_Humidity = bme.readHumidity();

        // Check for sensor error
        if (isnan(Input_Humidity)) {
            Serial.println("[ERROR] BME280 humidity reading failed!");
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ─────────────────────────────────────────────────
        // STEP 2: SAFETY CHECK
        // ─────────────────────────────────────────────────
        if (Input_Humidity > HUMID_MAX_SAFE) {
            ledcWrite(PWM_CH_HUMID, 0);
            Output_Humidifier = 0;
            Serial.println("[SAFETY] Humidity too high! Humidifier OFF");
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ─────────────────────────────────────────────────
        // STEP 3: COMPUTE PID
        // ─────────────────────────────────────────────────
        Output_Humidifier = humidPID.compute(Setpoint_Humidity, Input_Humidity);

        // ─────────────────────────────────────────────────
        // STEP 4: ACTUATE (Only positive output)
        // ─────────────────────────────────────────────────
        // Note: We can only ADD humidity, not remove it easily
        if (Output_Humidifier > 0) {
            ledcWrite(PWM_CH_HUMID, (int)Output_Humidifier);
        } else {
            ledcWrite(PWM_CH_HUMID, 0);
        }

        // ─────────────────────────────────────────────────
        // STEP 5: WAIT FOR NEXT CYCLE
        // ─────────────────────────────────────────────────
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ═══════════════════════════════════════════════════════════════════
//                         SETUP FUNCTION
// ═══════════════════════════════════════════════════════════════════
void setup() {
    // Initialize Serial Communication
    Serial.begin(115200);
    delay(1000);  // Wait for serial to stabilize
    
    Serial.println("\n\n");
    Serial.println("╔═══════════════════════════════════════════════════╗");
    Serial.println("║                                                   ║");
    Serial.println("║      ESP32-WROOM THERMAL PID CONTROL SYSTEM       ║");
    Serial.println("║      Temperature & Humidity Regulation            ║");
    Serial.println("║                                                   ║");
    Serial.println("║      Status: INITIALIZING...                      ║");
    Serial.println("║                                                   ║");
    Serial.println("╚═══════════════════════════════════════════════════╝");
    Serial.println();

    // ─────────────────────────────────────────────────
    // Initialize I2C Communication
    // ─────────────────────────────────────────────────
    Serial.println("[INIT] Starting I2C bus...");
    Wire.begin(PIN_SDA, PIN_SCL);
    Serial.println("[OK] I2C bus initialized (SDA=GPIO21, SCL=GPIO22)");
    Serial.println("[INFO] BME280 Pins: SDI→GPIO21, SCK→GPIO22, CSB→GND, SDO→GND");

    // ─────────────────────────────────────────────────
    // Initialize BME280 Sensor
    // ─────────────────────────────────────────────────
    Serial.println("[INIT] Scanning for BME280...");
    
    if (!bme.begin(0x76)) {  // Try primary address
        Serial.println("[RETRY] Address 0x76 failed, trying 0x77...");
        if (!bme.begin(0x77)) {  // Try alternate address
            Serial.println("╔═══════════════════════════════════════╗");
            Serial.println("║  ❌ ERROR: BME280 NOT FOUND!         ║");
            Serial.println("║  Troubleshooting:                     ║");
            Serial.println("║  1. Check I2C wiring:                 ║");
            Serial.println("║     Pin 3 (SDI) → GPIO 21             ║");
            Serial.println("║     Pin 4 (SCK) → GPIO 22             ║");
            Serial.println("║  2. Check power:                      ║");
            Serial.println("║     Pin 6 (VDDIO) → 3.3V              ║");
            Serial.println("║  3. Check GND connections:            ║");
            Serial.println("║     Pin 2 (CSB) → GND                 ║");
            Serial.println("║     Pin 5 (SDO) → GND                 ║");
            Serial.println("║  4. Check pull-up resistors           ║");
            Serial.println("╚═══════════════════════════════════════╝");
            while (1) {
                delay(1000);  // Halt execution
            }
        }
    }
    Serial.println("[OK] BME280 sensor detected at 0x76");

    // Configure BME280 for fast response
    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,       // Continuous measurement
        Adafruit_BME280::SAMPLING_X2,       // Temperature oversampling
        Adafruit_BME280::SAMPLING_X1,       // Pressure (not used much)
        Adafruit_BME280::SAMPLING_X2,       // Humidity oversampling
        Adafruit_BME280::FILTER_OFF,        // No filter for fast response
        Adafruit_BME280::STANDBY_MS_0_5     // 0.5ms standby time
    );

    // ─────────────────────────────────────────────────
    // Setup PWM Channels (ESP32 v3.3.3+ uses ledcAttach)
    // ─────────────────────────────────────────────────
    Serial.println("[INIT] Setting up PWM channels...");
    
    // Modern ESP32 (v3.3.3+) PWM setup
    ledcAttach(PIN_HEATER, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_FAN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_HUMIDIFIER, PWM_FREQ, PWM_RESOLUTION);

    // Initialize all outputs to OFF (safe state)
    ledcWrite(PWM_CH_HEATER, 0);
    ledcWrite(PWM_CH_FAN, 0);
    ledcWrite(PWM_CH_HUMID, 0);
    
    Serial.println("[OK] PWM channels attached (ESP32 v3.3.3+):");
    Serial.println("     - Heater (GPIO 25) at 5kHz");
    Serial.println("     - Fan (GPIO 26) at 5kHz");
    Serial.println("     - Humidifier (GPIO 27) at 5kHz");

    // ─────────────────────────────────────────────────
    // Create FreeRTOS Control Tasks
    // ─────────────────────────────────────────────────
    Serial.println("[INIT] Creating FreeRTOS tasks...");
    
    xTaskCreatePinnedToCore(
        Task_TemperatureControl,  // Task function
        "TempPID",                // Task name
        4096,                     // Stack size (bytes)
        NULL,                     // Parameters
        2,                        // Priority (higher = more important)
        &TaskTempHandle,          // Task handle
        1                         // Run on Core 1
    );

    xTaskCreatePinnedToCore(
        Task_HumidityControl,
        "HumidPID",
        4096,
        NULL,
        1,                        // Lower priority than temperature
        &TaskHumidHandle,
        1                         // Run on Core 1
    );

    Serial.println("[OK] FreeRTOS tasks created and running");
    Serial.println();
    Serial.println("╔═══════════════════════════════════════════════════╗");
    Serial.println("║            ✓ SYSTEM READY ✓                      ║");
    Serial.println("╚═══════════════════════════════════════════════════╝");
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
//                      MAIN MONITORING LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
    static unsigned long lastPrint = 0;
    
    // Print status every 500ms
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.printf("⏱️  Time: %lu sec\n", millis() / 1000);
        Serial.println();
        
        // ─── TEMPERATURE STATUS ───
        Serial.println("🌡️  TEMPERATURE CONTROL:");
        Serial.printf("   Current:  %.2f °C\n", Input_Temp);
        Serial.printf("   Target:   %.2f °C\n", Setpoint_Temp);
        Serial.printf("   Error:    %+.2f °C\n", Setpoint_Temp - Input_Temp);
        Serial.printf("   Heater:   %3d/255 PWM", (int)Output_Heater);
        if (Output_Heater > 200) Serial.print(" 🔥 HIGH");
        else if (Output_Heater > 100) Serial.print(" 🔥 MEDIUM");
        else if (Output_Heater > 0) Serial.print(" 🔥 LOW");
        Serial.println();
        
        Serial.printf("   Fan:      %3d/255 PWM", (int)Output_Fan);
        if (Output_Fan > 200) Serial.print(" ❄️  HIGH");
        else if (Output_Fan > 100) Serial.print(" ❄️  MEDIUM");
        else if (Output_Fan > 0) Serial.print(" ❄️  LOW");
        Serial.println();
        Serial.println();
        
        // ─── HUMIDITY STATUS ───
        Serial.println("💧 HUMIDITY CONTROL:");
        Serial.printf("   Current:  %.1f %%\n", Input_Humidity);
        Serial.printf("   Target:   %.1f %%\n", Setpoint_Humidity);
        Serial.printf("   Error:    %+.1f %%\n", Setpoint_Humidity - Input_Humidity);
        Serial.printf("   Humid:    %3d/255 PWM", (int)Output_Humidifier);
        if (Output_Humidifier > 200) Serial.print(" 💨 HIGH");
        else if (Output_Humidifier > 100) Serial.print(" 💨 MEDIUM");
        else if (Output_Humidifier > 0) Serial.print(" 💨 LOW");
        Serial.println();
        Serial.println();
        
        // ─── SYSTEM STATUS ───
        Serial.print("📊 Status: ");
        float tempError = abs(Setpoint_Temp - Input_Temp);
        float humidError = abs(Setpoint_Humidity - Input_Humidity);
        
        if (tempError < 1.0 && humidError < 5.0) {
            Serial.println("✅ STABLE - Within Target Range");
        } else if (tempError < 5.0 && humidError < 10.0) {
            Serial.println("⚠️  CONVERGING - Approaching Target");
        } else {
            Serial.println("🔄 REGULATING - Actively Controlling");
        }
        
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    }
    
    // Small delay to prevent Serial flooding
    delay(50);
}