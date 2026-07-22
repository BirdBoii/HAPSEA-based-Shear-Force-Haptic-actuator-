#include <Arduino.h>

// put function declarations here:
#include <Wire.h>
#include <INA226.h>

// ============================================================
//  Hardware / sensor configuration
// ============================================================
INA226 ina(0x40);

#define LPWM_PIN 26
#define RPWM_PIN 27
#define PWMSignal_Pin 23

//============================================================
// Defination of refrence signal
//============================================================

volatile float globalReference = 0.0f;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

const float globalCurrentLimit = 1; //in Amperes
const int timePeriodSine = 600;


const float    V_SOURCE       = 10.0f;
const uint32_t PWM_FREQ       = 20000;
const uint8_t  PWM_RESOLUTION = 8;
const uint16_t PWM_MAX        = 255;

const float    SAMPLE_RATE_HZ   = 400.0f;
const uint32_t SENSOR_PERIOD_US = (uint32_t)(1000000.0f / SAMPLE_RATE_HZ + 0.5f);

const uint32_t CONTROL_PERIOD_US = SENSOR_PERIOD_US;  // 100 Hz, Ts = 10 ms

// Remove/Comment out this line if you want to use the rolling filter instead of the Kalman filter
#define usingKalman

//Kalman Filter Parameters
volatile float kalmanGain = 0.0;
volatile float estimateValueKalman = 1.0;
volatile float kalmanErrorEstimate = 0.1;
volatile float kalmanErrorMeasured = 0.0f;
volatile float prevEstimateKalman = 0.0;

// Kalman Filter Tuning Parameters
const float kalman_Q = 0.05f; // Process noise (increase to make filter more responsive)
const float kalman_R = 0.3f;  // Measurement noise (increase to make filter smoother/slower)


//#define PROFILE_WITH_GPIO
//#define PROFILE_PIN 25

// ============================================================
//  Shared buffer - written by sensorTask, read by controlISR.
//  20-sample rolling window at 400 Hz = 50 ms of history.
//  rollingAvg is updated incrementally (O(1)) on every sensor tick
//  so controlISR only needs to read a single float.
// ============================================================
#define BUFFER_SIZE 20   // not a power of two - use modulo increment below

volatile float   shuntBuffer[BUFFER_SIZE] = {};
volatile uint8_t bufferIndex = 0;
volatile float   rollingAvg  = 0.0f;
static portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;



// ============================================================
//  Timers + task handle
// ============================================================

hw_timer_t  *sensorTimer  = NULL;
hw_timer_t  *controlTimer = NULL;
TaskHandle_t sensorTaskHandle = NULL;

TaskHandle_t pwmSignalTaskHandle = NULL;

// ============================================================
//  PID state
// ============================================================
float Kp = 100.0f;
float Ki = 0.0f;
float Kd = 0.0f;

float integral   = 0.0f;
float error_prev = 0.0f;

const float Ts = CONTROL_PERIOD_US * 1e-6f;   // 0.01 s

struct LogEntry {
    uint32_t time_us;
    float reference;
    float measurement;
    float duty;
    float error;
};

const uint32_t LOG_SIZE = 5000;

volatile LogEntry logBuffer[LOG_SIZE];
volatile uint32_t logIndex = 0;
volatile bool loggingFinished = false;

volatile float deltaV = 0.0f;

const float REF_AMPLITUDE = 0.02f;
const float REF_FREQ      = 0.01f;

volatile uint32_t controlExecCycles_last = 0;
volatile uint32_t controlExecCycles_max  = 0;
volatile uint32_t controlOverrunCount    = 0;

// ============================================================
//  INTERRUPT 1: sensor trigger
// ============================================================

//This is where the sensing function is being called from. Priority will be checked and then the function will be called

void IRAM_ATTR sensorISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ============================================================
//  Sensor task is defined below
// ============================================================

void sensorTask(void *pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!ina.isConversionReady()) {
            continue;
        }

        float shunt = ina.getShuntVoltage_mV() / 100.0f;  // Amps, R = 0.1 Ω

        
        float oldVal = shuntBuffer[bufferIndex];

        #ifndef usingKalman 
        
        //These will complete without exiting the loop, prevents other parts of the code from reading incomplete values 
        portENTER_CRITICAL(&bufferMux);
            shuntBuffer[bufferIndex] = shunt;
            bufferIndex = (bufferIndex + 1 >= BUFFER_SIZE) ? 0 : bufferIndex + 1;
            // Incremental O(1) update: add new sample, subtract evicted sample
            rollingAvg += (shunt - oldVal) * (1.0f / BUFFER_SIZE);
        portEXIT_CRITICAL(&bufferMux);

        #endif

        #ifdef usingKalman
        portENTER_CRITICAL(&bufferMux);
        
        kalmanErrorEstimate = kalmanErrorEstimate + kalman_Q;
        kalmanGain = kalmanErrorEstimate / (kalmanErrorEstimate + kalman_R);
        
        // Update the estimate with the new measurement
        estimateValueKalman = estimateValueKalman + kalmanGain * (shunt - estimateValueKalman);
        
        // Update the error covariance for the next loop
        kalmanErrorEstimate = (1.0f - kalmanGain) * kalmanErrorEstimate;
        
        portEXIT_CRITICAL(&bufferMux);
        #endif
    }
}


// ============================================================
//  INTERRUPT 2: controller (100 Hz)
// ============================================================

TaskHandle_t controlTaskHandle = NULL;

// The interrupt just wakes up the task
void IRAM_ATTR controlISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(controlTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// The task does the heavy math safely
void controlTask(void *pvParameters)
{
    for (;;)
    {
        // Wait for the hardware timer to wake us up
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t cyclesStart = ESP.getCycleCount();
#ifdef PROFILE_WITH_GPIO
        digitalWrite(PROFILE_PIN, HIGH);
#endif

        float measurement;
        #ifndef usingKalman
        portENTER_CRITICAL(&bufferMux); // Note: Swapped _ISR for standard macro
            measurement = rollingAvg;
        portEXIT_CRITICAL(&bufferMux);
        #endif

        #ifdef usingKalman
        portENTER_CRITICAL(&bufferMux);
            measurement = estimateValueKalman;
        portEXIT_CRITICAL(&bufferMux);
        #endif

        float t = micros() * 1e-6f;
        float reference;
        
        if (t < 2.0f) {
            reference = 0.0f;
        } else {
            portENTER_CRITICAL(&spinlock); 
            reference = globalReference;
            portEXIT_CRITICAL(&spinlock);
        }

        float error = reference - measurement;
        integral += error * Ts;
        float derivative = (error - error_prev) / Ts;

        float Vdesired = Kp * error + Ki * integral + Kd * derivative;
        Vdesired += deltaV;
        error_prev = error;

        if (Vdesired > V_SOURCE) {
            Vdesired = V_SOURCE;
            if (error > 0.0f) integral -= error * Ts;
        }
        if (Vdesired < -V_SOURCE) {
            Vdesired = -V_SOURCE;
            if (error < 0.0f) integral -= error * Ts;
        }

        float duty = fabsf(Vdesired) / V_SOURCE;
        if (duty > 1.0f) duty = 1.0f;

        uint16_t pwmCounts = (uint16_t)(duty * PWM_MAX);

        // SAFELY writing to PWM from a Task
        if (Vdesired >= 0.0f) {
            ledcWrite(RPWM_PIN, pwmCounts);
            ledcWrite(LPWM_PIN, 0);
        } else {
            ledcWrite(RPWM_PIN, 0);
            ledcWrite(LPWM_PIN, pwmCounts);
        }

        if (!loggingFinished) {
            if (logIndex < LOG_SIZE) {
                logBuffer[logIndex].time_us     = micros();
                logBuffer[logIndex].reference   = reference;
                logBuffer[logIndex].measurement = measurement;
                logBuffer[logIndex].duty        = duty;
                logBuffer[logIndex].error       = error;
                logIndex += 1;
            } else {
                loggingFinished = true; 
            }
        }

#ifdef PROFILE_WITH_GPIO
        digitalWrite(PROFILE_PIN, LOW);
#endif
        uint32_t cycles = ESP.getCycleCount() - cyclesStart;
        controlExecCycles_last = cycles;
        if (cycles > controlExecCycles_max) {
            controlExecCycles_max = cycles;
        }
        if (cycles > (uint32_t)CONTROL_PERIOD_US * 240) {
            controlOverrunCount += 1;
        }
    }
}

void pwmSignal(void *parameters)
{
    float temp = globalCurrentLimit;
    while(true){
        //for sin make sure that the below line is uncommented
        //temp = sinf(millis()/1000.0f * timePeriodSine) * globalCurrentLimit;
        temp = -temp;
        vTaskDelay(pdMS_TO_TICKS(200));
        portENTER_CRITICAL(&spinlock);
        globalReference = temp;
        portEXIT_CRITICAL(&spinlock);
        vTaskDelay(pdMS_TO_TICKS(1));

    }
}

// ============================================================
//  Setup
// ============================================================
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    ina.begin();
    delay(100);

#ifdef PROFILE_WITH_GPIO
    pinMode(PROFILE_PIN, OUTPUT);
    digitalWrite(PROFILE_PIN, LOW);
#endif
    ledcAttach(PWMSignal_Pin, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  //Create a task
    xTaskCreatePinnedToCore(
        sensorTask, "sensorTask", 4096, NULL,
        configMAX_PRIORITIES - 1,
        &sensorTaskHandle,
        1
    );

    //creating a task to test the filters-
    xTaskCreate(
        pwmSignal, "PWMSignal",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        &pwmSignalTaskHandle
    );

    // ... existing sensorTask and pwmSignal setup ...

  // Create the control task right before attaching the timers
    xTaskCreatePinnedToCore(
        controlTask, "controlTask", 8192, NULL,
        configMAX_PRIORITIES - 1, // High priority to run instantly when timer fires
        &controlTaskHandle,
        1 // Pinned to core 1
    );

    sensorTimer = timerBegin(1000000);
  // ... rest of timer setup ...

    sensorTimer = timerBegin(1000000);
    timerAttachInterrupt(sensorTimer, &sensorISR);
    timerAlarm(sensorTimer, SENSOR_PERIOD_US, true, 0);

    controlTimer = timerBegin(1000000);
    timerAttachInterrupt(controlTimer, &controlISR);
    timerAlarm(controlTimer, CONTROL_PERIOD_US, true, 0);
    ledcWrite(PWMSignal_Pin, 200);
}

// ============================================================
//  Loop
// ============================================================
void loop()
{
    static bool dumped = false;
    Serial.println("Hello");
    if (loggingFinished && !dumped)
    {
        dumped = true;
        
        for (uint32_t i = 0; i < logIndex; i++)
        {
            Serial.print(">time_us:");
            Serial.println(logBuffer[i].time_us);

            Serial.print(">refrence:");
            Serial.println(logBuffer[i].reference, 5);

            Serial.print(">measurement:");
            Serial.println(logBuffer[i].measurement, 5);
            
            Serial.print(",");
            Serial.print(logBuffer[i].duty, 5);
            Serial.print(",");
            Serial.println(logBuffer[i].error, 5);
        }
    }


    vTaskDelay(pdMS_TO_TICKS(1000));
}