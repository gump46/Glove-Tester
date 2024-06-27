#include <Arduino.h>

#include <HX711.h>
#include <Adafruit_MPRLS.h>
#include <ESP_FlexyStepper.h>

#define DEBUG 1 // Global debug flag, enables (1) or disables (0) debug serial output
#include "hmi.h"
#include "utility.h"

// IO Pin Assignments =============================================================================

// Debug serial
#define PIN_SERIAL_TX0 (uint8_t) 9       // Debug serial TX pin
#define PIN_SERIAL_RX0 (uint8_t) 10      // Debug serial RX pin

// Nextion touchscreen (initalized in NexHardware.cpp - DO NOT CALL Serial2.begin() in setup())
#define PIN_SERIAL_TX2 (uint8_t) 17      // Touchscreen serial TX pin (not setup in main.cpp)
#define PIN_SERIAL_RX2 (uint8_t) 16      // Touchscreen serial RX pin (not setup in main.cpp)

// Stepper motor controller
#define PIN_MOTOR_ENABLE (uint8_t) 0     // Stepper motor controller enable pin
#define PIN_MOTOR_STEP (uint8_t) 13      // Stepper motor controller run pin
#define PIN_MOTOR_DIRECTION (uint8_t) 12 // Stepper motor controller direction pin
#define PIN_MOTOR_LIMIT (uint8_t) 4      // Stepper motor limit switch pin

// Water pump controller
#define PIN_WATER_PUMP (uint8_t) 25      // Water pump h-bridge input pin

// Vaccum pump controller
#define PIN_VACUUM_PUMP (uint8_t) 26     // Vacuum pump h-bridge input pin

// Valve relays
#define PIN_RELAY_VACUUM_A (uint8_t) 15  // Vacuum valve a relay control pin (vacuum = low, exhaust = high)
#define PIN_RELAY_VACUUM_B (uint8_t) 2   // Vacuum valve b relay control pin (vacuum = low, exhaust = high)

// Load cell reader
#define PIN_LC_DATA (uint8_t) 27         // Load cell data pin
#define PIN_LC_CLOCK (uint8_t) 14        // Load cell clock pin

// Pressure sensor (I2C bus 1)
#define PIN_I2C_SDA (uint8_t) 21         // Pressure sensor I2C data pin
#define PIN_I2C_SCL (uint8_t) 22         // Pressure sensor I2C clock pin

// Hardware Constants =============================================================================

// Debug serial
#define SERIAL0_BAUD 115200                // Baud rate used by serial 1

// Nextion touchscreen (initalized in NexHardware.cpp - DO NOT CALL Serial2.begin() in setup())
#define SERIAL2_BAUD 115200                // Baud rate used by serial 2

// Stepper motor
#define MOTOR_ACCELERATION (uint16_t) 1000 // Stepper motor acceleration in steps/second^2
#define MOTOR_DECELERATION (uint16_t) 1000 // Stepper motor deceleration in steps/second^2
#define MOTOR_LIMIT_DEBOUNCE (uint8_t) 50  // Stepper motor limit switch debounce time in ms
#define STEPS_PER_MM (uint16_t) 225        // Number of steps required to move stepper 1 mm

// Water pump
#define WATER_PUMP_PWM_CH (uint8_t) 0      // Water pump PWM channel
#define WATER_PUMP_PWM_FREQ (float) 1000   // Water pump PWM frequency in Hz
#define WATER_PUMP_PWM_RES (uint8_t) 8     // Water pump PWM resolution in bits
#define WATER_PUMP_DUTY (uint8_t) 255      // Water pump motor duty cycle 0-255 (255 = 100%)
#define WATER_PUMP_SPRAY (uint16_t) 2000   // Water pump spray duration in milliseconds

// Vacuum pump
#define VACUUM_PUMP_PWM_CH (uint8_t) 1     // Vacuum pump PWM channel
#define VACUUM_PUMP_PWM_FREQ (float) 1000  // Vacuum pump PWM frequency in Hz
#define VACUUM_PUMP_PWM_RES (uint8_t) 8    // Vacuum pump PWM resolution in bits
#define VACUUM_PUMP_DUTY (uint8_t) 255     // Vacuum pump duty cycle 0-255 (255 = 100%)
#define MINIMUM_PRESSURE (uint32_t) 400    // Minimum (target) pressure in hPa for glove depressurization
#define MAXIMUM_PRESSURE (uint32_t) 800    // Maximum pressure in hPa before glove needs to be re-depressurized
#define MAXIMUM_PRESSURE_D (uint32_t) 3    // Pressure delta in Pa/ms above which glove has probably broken
#define PRESSURE_TIMEOUT (uint16_t) 30000  // Timeout in ms before giving up trying to depressurize glove

// Load cell reader
#define LOADCELL_GAIN (uint8_t) 64         // Load cell gain (valid values = 32, 64, 128)

// Pressure sensor
#define PRESSURE_I2C_ADDR (uint8_t) 0x18   // Pessure sensor I2C address
#define PRESSURE_I2C_BUS &Wire             // Pressure sensor I2C bus (&Wire or &Wire1)

// Software Constants =============================================================================

#define TEST_DEFAULT (uint16_t) 50     // Default value the test field is initalized to
#define TEST_STEP (uint8_t) 1          // How much the + and - buttons increase and decrease the test value
#define TEST_MIN (uint8_t) 1           // Minimum value the test field can have
#define TEST_MAX (uint16_t) 65535      // Maximum value the test field can have
#define SPEED_DEFAULT (uint16_t) 35    // Default value the speed field is initalized to
#define SPEED_STEP (uint8_t) 1         // How much the + and - buttons increase and decrease the speed value
#define SPEED_MIN (uint16_t) 5         // Minimum value the speed field can have in 10^-4m/s
#define SPEED_MAX (uint16_t) 100       // Maximum value the speed field can have 10^-4m/s
#define FORCE_DEFAULT (uint32_t) 15000 // Default value the force field is initalized to in mN
#define FORCE_STEP (uint16_t) 100      // How much the + and - buttons increase and decrease the force value
#define FORCE_MIN (uint16_t) 1000      // Minimum value the force field can have in mN
#define FORCE_MAX (uint32_t) 20000     // Maximum value the force field can have in mN
#define SPRAY_DEFAULT (int32_t) -1     // Default value the spray field is initalized to (-1 = disabled)
#define SPRAY_STEP (uint8_t) 1         // How much the + and - buttons increase and decrease the spray value
#define SPRAY_MIN (int8_t) -1          // Minimum value the spray field can have (-1 = disabled)
#define SPRAY_MAX (uint16_t) 65535     // Maximum value the spray field can have

// Global Object Definitions ======================================================================

// Stepper motor
ESP_FlexyStepper StepperCtl;

// Load cell
HX711 LoadCell;

// Pressure Sensor
Adafruit_MPRLS Pressure;

// Global Variables ===============================================================================

// Stepper motor limit switch debounce
volatile uint64_t limitDebounce = 0;           // Time in ms the current debounce timer started (from millis())
volatile bool limitDebounceInProgress = false; // Indicates if a debounce is currently in progress
volatile int state = digitalRead(PIN_MOTOR_LIMIT);
volatile int lastState = digitalRead(PIN_MOTOR_LIMIT);

// Enum defining state machine states
enum class MachineState
{
    Idle,
    TestInit,
    TestPreVacuum,
    TestVacuum,
    TestPostVacuum,
    TestPreSpray,
    TestSpray,
    TestPostSpray,
    TestPreTouch,
    TestStepperAdvance,
    TestStepperRetract,
    TestPostTouch,
    TestPost,
    TestAbort
};

MachineState currentState = MachineState::Idle;  // Current state machine state

//Loadcell reading variable
long loadcell_reading = 0;

// Pressure monitor variables
uint64_t lastPressureTime = 0;                // Last pressure value read by pressure sensor
float lastPressureValue = 0;                  // Time in milliseconds since boot last pressure reading was taken
float pressureDelta = 0;                      // Delta of last pressure reading

// General state machine variables
bool pressureDeltaExceeded = false;           // Flag set if pressure delta is exceeded and glove is likely broken
uint32_t testIndex = 1;                       // Current test being run
uint32_t totalTests = TEST_DEFAULT;           // Number of tests to run (GUI-set)

// TestVacuum variables
uint64_t testVacuumStartTime = 0;             // Time TestVacuum state was entered in ms
bool depressurizeFailed = false;              // Flag set if glove depressurization times out

// TestSpray variables
bool sprayEnabled = false;                    // Flag set if glove spray is enabled (GUI-set)
int32_t testsBetweenSprays = SPRAY_DEFAULT;   // Number of tests between each spray (GUI-set)
uint64_t currentSprayInterval = 0;            // Number ot tests run since last spray
uint64_t testSprayStartTime = 0;              // Time TestSpray state was entered in ms

// TestStepper variables
uint32_t stepperVelocity = SPEED_DEFAULT;     // Max velocity of stepper motor in 10^-4m/s (GUI-set)
uint32_t loadCellThreshold = FORCE_DEFAULT;   // Target force to exert with stepper motor in TODO: determine units

// Test elapsed time variables
long lastTime;
long elapsedTime;
bool updateTime = false;

// MIGHT DELETE, TEST VARIABLES
uint8_t data[7]; // holds output data 
uint8_t cmd[3] = {0xAA, 0x00, 0x00}; // command to be sent 
double press_counts = 0; // digital pressure reading [counts] 
double outputmax = 15099494; // output at maximum pressure [counts] 
double outputmin = 1677722; // output at minimum pressure [counts] 
double pmax = 25; // maximum value of pressure range [bar, psi, kPa, etc.] 
double pmin = 0; // minimum value of pressure range [bar, psi, kPa, etc.] 
double pressure = 0; // pressure reading [bar, psi, kPa, etc.]

// Function Prototypes ============================================================================

/**
 * Update the values of the TextFields HomeTest, HomeSpeed, HomeForce, and HomeSpray.
 *
 * \param testVal Value to print to HomeTest
 * \param speedVal Value to print to HomeSpeed
 * \param forceVal Value to print to HomeForce suffixed with "mN"
 * \param sprayVal Value to print to HomeSpray
 */
void update_home(uint16_t testVal, uint16_t speedVal, uint32_t forceVal, int32_t sprayVal);

/**
 * Converts time in ms to the format "m:ss.ss" and prints to the specified TextField. Example:
 * "54:23.45".
 *
 * \param text TextField to print formatted time to
 * \param time Time in ms to print
 */
void print_time(TextField text, long time);

/**
 * Prints the specified touch counts in the format "thisTouch of totalTouch" to the specified
 * TextField. Example: "4 of 50".
 *
 * \param text TextField to print formatted counts to
 * \param thisTouch Current touch being executed
 * \param totalTouch Total number of touches being executed in this test
 */
void print_touches(TextField touch, int thisTouch, int totalTouch);

/**
 * Interrupt callback function for the stepper motor limit switch. Implements debouncing of the
 * switch and notifies the StepperCtl service that the limit switch has been enabled/disabled.
 */
void interrupt_stepper();

/**
 * State machine helper function. Checks if the current pressure delta exceeds the threshold
 * specified in the above defines, and if so, sets the pressureDeltaExceeded flag and the
 * state to TestPost.
 */
void check_pressure_delta();

/**
 * Button push event handler. Should be called for every button push event.
 *
 * \param event Button push event type
 */
void button_push_handler(ButtonPushEvent event);

/**
 * Button release event handler. Should be called for every button release event.
 *
 * \param event Button release event type
 */
void button_release_handler(ButtonReleaseEvent event);

// Setup ==========================================================================================

void setup()
{
    // Set sprayEnabled to true if SPRAY_DEFAULT isn't -1
    if(SPRAY_DEFAULT != -1)
    {
        sprayEnabled = true;
    }

    // Debug serial output init
    if(DEBUG)
    {
        Serial.begin(SERIAL0_BAUD, SERIAL_8N1);
        delay(500); // Short delay to give serial console on laptop time to init
    }
    dprint(Serial, "Serial 0 started at baud %d (TX=%d RX=%d)", SERIAL0_BAUD, PIN_SERIAL_TX0, PIN_SERIAL_RX0);

    // GPIO init
    dprint(Serial, "Beginning GPIO init");
    pinMode(PIN_MOTOR_ENABLE, OUTPUT);
    dprint(Serial, "GPIO pin %d set to output", PIN_MOTOR_ENABLE);
    pinMode(PIN_WATER_PUMP, OUTPUT);
    dprint(Serial, "GPIO pin %d set to output", PIN_WATER_PUMP);
    pinMode(PIN_VACUUM_PUMP, OUTPUT);
    dprint(Serial, "GPIO pin %d set to output", PIN_VACUUM_PUMP);
    pinMode(PIN_RELAY_VACUUM_A, OUTPUT);
    dprint(Serial, "GPIO pin %d set to output", PIN_RELAY_VACUUM_A);
    pinMode(PIN_RELAY_VACUUM_B, OUTPUT);
    dprint(Serial, "GPIO pin %d set to output", PIN_RELAY_VACUUM_B);
    pinMode(PIN_MOTOR_LIMIT, INPUT);
    attachInterrupt(PIN_MOTOR_LIMIT, interrupt_stepper, CHANGE);
    dprint(Serial, "GPIO pin %d set to input", PIN_MOTOR_LIMIT);

    // PWM init
    if(ledcSetup(WATER_PUMP_PWM_CH, WATER_PUMP_PWM_FREQ, WATER_PUMP_PWM_RES))
    {
        dprint(Serial, "GPIO pin %d linked to PWM %d at %f Hz with %i bit resolution", PIN_WATER_PUMP, WATER_PUMP_PWM_CH, WATER_PUMP_PWM_FREQ, WATER_PUMP_PWM_RES);
    }
    else
    {
        dprint(Serial, "Error: GPIO pin %d failed to link to PWM %d at %f Hz with %i bit resolution", PIN_WATER_PUMP, WATER_PUMP_PWM_CH, WATER_PUMP_PWM_FREQ, WATER_PUMP_PWM_RES);
    }
    ledcAttachPin(PIN_WATER_PUMP, WATER_PUMP_PWM_CH);
    if(ledcSetup(VACUUM_PUMP_PWM_CH, VACUUM_PUMP_PWM_FREQ, VACUUM_PUMP_PWM_RES))
    {
        dprint(Serial, "GPIO pin %d linked to PWM %d at %f Hz with %i bit resolution", PIN_VACUUM_PUMP, VACUUM_PUMP_PWM_CH, VACUUM_PUMP_PWM_FREQ, VACUUM_PUMP_PWM_RES);
    }
    else
    {
        dprint(Serial, "Error: GPIO pin %d failed to link to PWM %d at %f Hz with %i bit resolution", PIN_VACUUM_PUMP, VACUUM_PUMP_PWM_CH, VACUUM_PUMP_PWM_FREQ, VACUUM_PUMP_PWM_RES);
    }
    ledcAttachPin(PIN_VACUUM_PUMP, VACUUM_PUMP_PWM_CH);

    // I2C init
    dprint(Serial, "Beginning I2C init");
    if(Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) // not technically necessary but more verbose, Wire will also init in Pressure.begin()
    {
        dprint(Serial, "I2C0 initalized (SDA=%d SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);
    }
    else
    {
        dprint(Serial, "Error: I2C0 failed to initialize! (SDA=%d SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);
    }

    // Pressure sensor init and initial pressure value reading
    dprint(Serial, "Beginning MPRLS pressure sensor init");
    if(Pressure.begin(PRESSURE_I2C_ADDR, PRESSURE_I2C_BUS))
    {
        dprint(Serial, "Pressure sensor initalized (ADDR=0x%X)", PRESSURE_I2C_ADDR);
        float pressureTemp = Pressure.readPressure();
        if(pressureTemp != NAN)
        {
            lastPressureTime = millis();
            lastPressureValue = pressureTemp;
            dprint(Serial, "Pressure sensor init reading: %f hPa", pressureTemp);
        }
        else
        {
            dprint(Serial, "Error: Pressure sensor failed to read value!");
        }
    }
    else
    {
        dprint(Serial, "Error: Pressure sensor failed to initialize! (ADDR=0x%X)", PRESSURE_I2C_ADDR);
    }

    // Stepper motor controller init
    dprint(Serial, "Beginning stepper motor controller init");
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    StepperCtl.connectToPins(PIN_MOTOR_STEP, PIN_MOTOR_DIRECTION);
    StepperCtl.setSpeedInStepsPerSecond(((float)SPEED_DEFAULT / 10) * STEPS_PER_MM);
    StepperCtl.setAccelerationInStepsPerSecondPerSecond(MOTOR_ACCELERATION);
    StepperCtl.setDecelerationInStepsPerSecondPerSecond(MOTOR_DECELERATION);
    if(!digitalRead(PIN_MOTOR_LIMIT))
    {
        StepperCtl.setLimitSwitchActive(ESP_FlexyStepper::LIMIT_SWITCH_BEGIN);
    }
    StepperCtl.startAsService();
    if(StepperCtl.isStartedAsService())
    {
        dprint(Serial, "Stepper motor controller initalized (STEP=%d DIR=%d)", PIN_MOTOR_STEP, PIN_MOTOR_DIRECTION);
    }
    else
    {
        dprint(Serial, "Error: Stepper motor controller failed to initialize! (STEP=%d DIR=%d)", PIN_MOTOR_STEP, PIN_MOTOR_DIRECTION);
    }

    // Stepper motor zeroing process
    dprint(Serial, "Beginning stepper motor zero process");

    // Enable stepper motor and begin moving towards home
    digitalWrite(PIN_MOTOR_ENABLE, HIGH);
    StepperCtl.setDirectionToHome(-1);
    StepperCtl.goToLimitAndSetAsHome();

    // Wait for stepper to trigger and set home position
    while(!StepperCtl.motionComplete())
    {
        if(state != lastState)
        {
            dprint(Serial, "Limit switch changed state to %i", state);
            lastState = state;
        }
    }
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    dprint(Serial, "Stepper motor zero process completed");

    // Load cell reader init
    dprint(Serial, "Beginning HX711 load cell reader init");
    LoadCell.begin(PIN_LC_DATA, PIN_LC_CLOCK, LOADCELL_GAIN);
    if(LoadCell.wait_ready_timeout(100))
    {
        dprint(Serial, "Error: HX711 load cell reader failed to initialize! (DATA=%d CLOCK=%d GAIN=%d)", PIN_LC_DATA, PIN_LC_CLOCK, LOADCELL_GAIN); 
    }
    else
    {
        dprint(Serial, "HX711 load cell reader initalized (DATA=%d CLOCK=%d GAIN=%d)", PIN_LC_DATA, PIN_LC_CLOCK, LOADCELL_GAIN);
    }

    // Nextion touchscreen init
    dprint(Serial, "Beginning nextion touchscreen init");
    if(hmi_init(button_push_handler, button_release_handler))
    {
        // Set to home page
        if(hmi_set_page(Page::Home))
        {
            dprint(Serial, "Nextion touchscreen set to home page");
        }
        else
        {
            dprint(Serial, "Error: Failed to set Nextion touchscreen to home page!");
        }

        // Generate default value strings
        update_home(TEST_DEFAULT, SPEED_DEFAULT, FORCE_DEFAULT, SPRAY_DEFAULT);
        dprint(Serial, "Nextion touchscreen initalized");
    }
    else
    {
        dprint(Serial, "Error: Nextion touchscreen failed to initialize!");
    }
    dprint(Serial, "Init complete");
}

// Main Loop ======================================================================================

void loop()
{
    // Handle touchscreen button events
    hmi_run_button_callbacks();

  Wire.beginTransmission(PRESSURE_I2C_ADDR); 
  int stat = Wire.write(cmd, 3); // write command to the sensor 
  stat |= Wire.endTransmission(); 
  delay(10); 
  Wire.requestFrom(PRESSURE_I2C_ADDR, (uint8_t)7); // read back Sensor data 7 bytes 
  for (int i = 0; i < 7; i++) { 
    data[i] = Wire.read(); 
  } 

  press_counts = data[3] + data[2] * 256 + data[1] * 65536; // calculate digital pressure counts 

  // Calculation of pressure value according to equation 2 of datasheet 
  pressure = ((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin; 

  // Convert pressure to hPa (if necessary, otherwise remove conversion) 
  double pressureTemp = pressure * 68.9475729318; // assuming pressure is in psi, convert to hPa
 
    if(pressureTemp != NAN)
    {
        pressureDelta = (pressureTemp - lastPressureValue) / (millis() - lastPressureTime);
        lastPressureTime = millis();
        lastPressureValue = pressureTemp;

        // Debug output for logging pressure data
        // dprint(Serial, "Pressure: %f hPa", lastPressureValue);
        // dprint(Serial, "Pressure delta: %f hPa", pressureDelta);
    }
    else
    {
        dprint(Serial, "Error: Pressure sensor failed to read value!");
    }

    // Update time elapsed
    if (updateTime)
    {
        elapsedTime += millis() - lastTime;
        lastTime = millis();
        print_time(TextField::TestElapsed, elapsedTime);
    }

    // Handle state machine tasks
    switch(currentState)
    {
        // Idle state: state machine does nothing
        case(MachineState::Idle):
            break;

        // Test initialization state: ensures all state machine global variables and flags are set
        case(MachineState::TestInit):

            // Reset variables and flags
            pressureDeltaExceeded = false;
            depressurizeFailed = false;
            testIndex = 1;
            currentSprayInterval = 0;

            // Stepper motor init
            StepperCtl.setSpeedInStepsPerSecond(((float)stepperVelocity / 10) * STEPS_PER_MM);

            // Set up timing variables and set screen to test in progress page
            if(hmi_set_page(Page::Test))
            {
                elapsedTime = 0;
                lastTime = millis();
                updateTime = false;
                print_touches(TextField::TestNum, 1, totalTests);
                print_time(TextField::TestElapsed, elapsedTime);
                dprint(Serial, "Page set to Test");
            }
            else
            {
                dprint(Serial, "Error: Failed to set page to Test!");

                // Attempt to get back to the home page
                hmi_set_page(Page::Home);
                currentState = MachineState::Idle;
                break;
            }

            // Set state to TestPreVacuum
            currentState = MachineState::TestPreVacuum;
            dprint(Serial, "New state: TestPreVacuum");
            break;

        // Pre-glove depressurization state: configures machine to depressurize glove
        case(MachineState::TestPreVacuum):

            // Stop updating time during vacuum stage
            updateTime = false;

            // Close exhaust valve and open vacuum valve
            digitalWrite(PIN_RELAY_VACUUM_A, LOW);
            digitalWrite(PIN_RELAY_VACUUM_B, HIGH);

            // Start vacuum pump
            ledcWrite(VACUUM_PUMP_PWM_CH, VACUUM_PUMP_DUTY);

            // Record start time and enter TestVacuum
            testVacuumStartTime = millis();
            currentState = MachineState::TestVacuum;
            dprint(Serial, "New state: TestVacuum");
            break;

        // Glove depressurization state: waits for glove to reach correct pressure or timeout to occur
        case(MachineState::TestVacuum):

            // First exit condition: glove has reached target pressure
            if(lastPressureValue <= MINIMUM_PRESSURE)
            {
                currentState = MachineState::TestPostVacuum;
                dprint(Serial, "New state: TestPostVacuum");
            }
            // Second exit condition: timeout period has elapsed
            else if(millis() - testVacuumStartTime > PRESSURE_TIMEOUT)
            {
                dprint(Serial, "Error: Unable to depressurize glove (%f hPa)!", lastPressureValue);

                // Handle as glove failure if not initial depressurization
                if(testIndex != 1)
                {
                    currentState = MachineState::TestPost;
                    pressureDeltaExceeded = true;
                    dprint(Serial, "New state: TestPost");
                }
                else
                {
                    currentState = MachineState::TestAbort;
                    depressurizeFailed = true;
                    dprint(Serial, "New state: TestAbort");
                }
            }
            break;

        // Post-glove depressurization state: configures machine to maintain glove pressure
        case(MachineState::TestPostVacuum):

            // Close vacuum valve
            // Close exhaust valve and open vacuum valve
            digitalWrite(PIN_RELAY_VACUUM_A, LOW);
            digitalWrite(PIN_RELAY_VACUUM_B, LOW);

            // Short delay to ensure valve is fully closed
            delay(100);

            // Stop vacuum pump
            ledcWrite(VACUUM_PUMP_PWM_CH, 0);

            // Enter TestPreTouch
            currentState = MachineState::TestPreTouch;
            dprint(Serial, "New state: TestPreTouch");

            // Resume updating time
            lastTime = millis();
            updateTime = true;
            break;

        // Pre-glove touch state: checks pressure delta and enters glove spray process if enabled and valid, otherwise
        //                        configures machine to advance stepper motor
        case(MachineState::TestPreTouch):

            // Update touch number
            print_touches(TextField::TestNum, testIndex, totalTests);

            check_pressure_delta();

            // If enabled and steps between has elapsed, enter glove spray sequence
            if(sprayEnabled && (testIndex == 1 || testsBetweenSprays == 0 || currentSprayInterval == testsBetweenSprays))
            {
                currentState = MachineState::TestPreSpray;
                dprint(Serial, "New state: TestPreSpray");
            }
            // Otherwise start stepper and enter stepper advance sequence
            else
            {
                // Enable stepper and start moving towards glove
                digitalWrite(PIN_MOTOR_ENABLE, HIGH);
                StepperCtl.startJogging(1);
                dprint(Serial, "Stepper jogging in positive direction");

                // Enter state TestStepperAdvance
                currentState = MachineState::TestStepperAdvance;
                dprint(Serial, "New state: TestStepperAdvance");
            }

            break;

        // Pre-glove spray state: checks pressure delta and configures machine to spray glove
        case(MachineState::TestPreSpray):

            check_pressure_delta();

            // Start water pump
            ledcWrite(WATER_PUMP_PWM_CH, WATER_PUMP_DUTY);

            // Record start time and enter state TestSpray
            testSprayStartTime = millis();
            currentState = MachineState::TestSpray;
            dprint(Serial, "New state: TestSpray");

            break;

        // Glove spray state: checks pressure delta and waits for glove spray time to elapse
        case(MachineState::TestSpray):

            check_pressure_delta();

            // Exit condition: spray time has elapsed
            if(millis() - testSprayStartTime >= WATER_PUMP_SPRAY)
            {
                // Enter state TestPostSpray
                currentState = MachineState::TestPostSpray;
                dprint(Serial, "New state: TestPostSpray");
            }

            break;

        // Post-glove spray state: checks pressure delta and configures machine to stop spraying and advance stepper motor
        case(MachineState::TestPostSpray):

            check_pressure_delta();

            // Clear number of tests between sprays
            currentSprayInterval = 0;

            // Stop water pump
            ledcWrite(WATER_PUMP_PWM_CH, 0);

            // Enable stepper and start moving towards glove
            digitalWrite(PIN_MOTOR_ENABLE, HIGH);
            StepperCtl.startJogging(1);
            dprint(Serial, "Stepper jogging in positive direction");

            // Enter state TestStepperAdvance
            currentState = MachineState::TestStepperAdvance;
            dprint(Serial, "New state: TestStepperAdvance");

            break;

        // Glove stepper advance state: checks pressure delta and waits for abrader to touch glove at specified force
        case(MachineState::TestStepperAdvance):

            check_pressure_delta();

            // Exit condition: load cell registers required force on glove (note: load cell output is reversed)
            if(LoadCell.is_ready())
            {
                // Convert load cell reading to value in mN
                int32_t rawValue = (-1)*LoadCell.read();
                int32_t calibratedValue = (rawValue + 96020.1) / 12.772;

                if(calibratedValue > (int32_t)loadCellThreshold)
                {
                    // Set stepper target position to home and enter state TestStepperRetract
                    StepperCtl.stopJogging();
                    delay(100);
                    StepperCtl.setTargetPositionInSteps(10000); // Retract Abrasive
                    currentState = MachineState::TestStepperRetract;
                    dprint(Serial, "New state: TestStepperRetract");
                }
            }

            break;

        // Glove stepper retract state: checks pressure delta and waits for stepper to return to home position
        case(MachineState::TestStepperRetract):

            check_pressure_delta();

            // Exit condition: stepper has reached home position
            if(StepperCtl.motionComplete())
            {
                // Disable stepper and enter state TestPostTouch
                digitalWrite(PIN_MOTOR_ENABLE, LOW);
                currentState = MachineState::TestPostTouch;
                dprint(Serial, "New state: TestPostTouch");
            }

            break;

        // Post-glove touch state: checks pressure delta and ends testing if target tests reached, otherwise re-depressurizes
        //                         glove if needed and begins next test
        case(MachineState::TestPostTouch):

            check_pressure_delta();

            // Increment test index and end test if exceeding total number of tests to run
            testIndex++;
            currentSprayInterval++;
            if(testIndex > totalTests)
            {
                currentState = MachineState::TestPost;
                dprint(Serial, "New state: TestPost");
            }
            else
            {

                // If glove pressure is above max threshold, re-pressurize glove
                if(lastPressureValue > MAXIMUM_PRESSURE)
                {
                    currentState = MachineState::TestPreVacuum;
                    dprint(Serial, "Glove pressure drifted above max value (%f hPa > %f hPa)", lastPressureValue, MAXIMUM_PRESSURE);
                    dprint(Serial, "New state: TestPreVacuum");
                }
                // Otherwise begin next test
                else
                {
                    currentState = MachineState::TestPreTouch;
                    dprint(Serial, "New state: TestPreTouch");
                }
            }
            break;

        // Post-test state: vents glove to exhaust and saves test data
        case(MachineState::TestPost):

            // Stop timing test
            updateTime = false;

            // Make sure stepper is moving to home if it isn't already
            digitalWrite(PIN_MOTOR_ENABLE, HIGH);
            StepperCtl.setTargetPositionInSteps(0);

            // Open exhaust valve
            digitalWrite(PIN_RELAY_VACUUM_A, HIGH);

            // After delay, close exhaust valve to limit heating of solenoids
            delay(5000);
            digitalWrite(PIN_RELAY_VACUUM_A, LOW);

            // Save test data
            // TODO: Implement test data save process here

            // Wait for stepper to finish moving to home, then disable stepper
            while(!StepperCtl.motionComplete());
            digitalWrite(PIN_MOTOR_ENABLE, LOW);

            // Display test complete page if test completed without glove breaking
            if(!pressureDeltaExceeded)
            {
                if(hmi_set_page(Page::TestComplete))
                {
                    print_time(TextField::TestCompleteElapsed, elapsedTime);
                    print_touches(TextField::TestCompleteNum, totalTests, totalTests);
                    dprint(Serial, "Page set to TestComplete");
                }
                else
                {
                    dprint(Serial, "Error: Failed to set page to TestComplete!");
                }
            }
            // Otherwise display glove broken page
            else
            {
                if(hmi_set_page(Page::TestFail))
                {
                    print_time(TextField::TestFailElapsed, elapsedTime);
                    print_touches(TextField::TestFailNum, testIndex, totalTests);
                    dprint(Serial, "Page set to TestFail");
                }
                else
                {
                    dprint(Serial, "Error: Failed to set page to TestFail!");
                }
            }
            //Print data to SD card file
            /*Record.name="test";
            if(!Record.setup_SD()){
                dprint(Serial, "Error initializing SD card");
            }else{
            Record.name_file();
            Record.numberOfTouchesEntered=totalTests;
            Record.numberOfTouchesCompleted=testIndex;
            Record.gloveBroken=pressureDeltaExceeded;
            Record.startTime=startTime;
            Record.stopTime=endTime;
            Record.sprayInterval=currentSprayInterval;
            //Record.force=loadcell_reading;
            Record.write_data(Record.name_file());
            }*/

            // Return to idle state
            currentState = MachineState::Idle;
            dprint(Serial, "New state: Idle");

            break;

        // Test aborted state: vents glove to exhaust and makes sure all motors are shut off and stepper is in home position
        case(MachineState::TestAbort):

            // Stop timing test
            updateTime = false;

            // Set screen to test aborted page or depressurize failed page
            if(depressurizeFailed)
            {
                if(hmi_set_page(Page::PressFail))
                {
                    dprint(Serial, "Page set to PressFail");
                }
                else
                {
                    dprint(Serial, "Error: Failed to set page to PressFail!");
                }
            }
            else
            {
                if(hmi_set_page(Page::TestAbort))
                {
                    dprint(Serial, "Page set to TestAbort");
                }
                else
                {
                    dprint(Serial, "Error: Failed to set page to TestAbort!");
                }
            }

            // Make sure stepper is moving to home if it isn't already
            digitalWrite(PIN_MOTOR_ENABLE, HIGH);
            StepperCtl.setTargetPositionInSteps(0);

            // Shut off vacuum and water pumps
            ledcWrite(VACUUM_PUMP_PWM_CH, 0);
            ledcWrite(WATER_PUMP_PWM_CH, 0);

            // Open exhaust valve and close vacuum valve
            digitalWrite(PIN_RELAY_VACUUM_A, HIGH);
            digitalWrite(PIN_RELAY_VACUUM_B, LOW);

            // After delay, close exhaust valve to limit heating of solenoids
            delay(5000);
            digitalWrite(PIN_RELAY_VACUUM_A, LOW);

            // Wait for stepper to finish moving to home, then disable stepper
            while(!StepperCtl.motionComplete());
            digitalWrite(PIN_MOTOR_ENABLE, LOW);

            // Set screen to home page if test was aborted
            if(!depressurizeFailed)
            {
                if(hmi_set_page(Page::Home))
                {
                    dprint(Serial, "Page set to Home");
                }
                else
                {
                    dprint(Serial, "Error: Failed to set page to Home!");
                }
                update_home(totalTests, stepperVelocity, loadCellThreshold, testsBetweenSprays);
            }

            // Return to idle state
            currentState = MachineState::Idle;
            dprint(Serial, "New state: Idle");

            break;

        default:
            dprint(Serial, "Error: State machine case statement default!");
    }
}

void update_home(uint16_t testVal, uint16_t speedVal, uint32_t forceVal, int32_t sprayVal)
{
    char numTests[10];
    char speed[10];
    char force[10];
    char spray[10];
    sprintf(numTests, "%i", testVal);
    sprintf(speed, "%.1f mm/s", (float)speedVal / 10);
    sprintf(force, "%i mN", forceVal);
    sprintf(spray, "%i", sprayVal);

    // Set home page values
    if(hmi_set_text_field(TextField::HomeTest, numTests))
    {
        dprint(Serial, "Tests text field set to %i", testVal);
    }
    else
    {
        dprint(Serial, "Error: Failed to set tests text field to %i", testVal);
    }
    if(hmi_set_text_field(TextField::HomeSpeed, speed))
    {
        dprint(Serial, "Speed text field set to %i mm/s", speedVal);
    }
    else
    {
        dprint(Serial, "Error: Failed to set speed text field to %i mm/s", speedVal);
    }
    if(hmi_set_text_field(TextField::HomeForce, force))
    {
        dprint(Serial, "Force text field set to %i mN", forceVal);
    }
    else
    {
        dprint(Serial, "Error: Failed to set force text field to %i mN", forceVal);
    }

    // Spray is a special case because it has a non-numerical display state for -1
    if(!sprayEnabled)
    {
        if(hmi_set_text_field(TextField::HomeSpray, "OFF"))
        {
            dprint(Serial, "Spray text field set to OFF");
        }
        else
        {
            dprint(Serial, "Error: Failed to set spray text field to OFF");
        }
    }
    else
    {
        if(hmi_set_text_field(TextField::HomeSpray, spray))
        {
            dprint(Serial, "Spray text field set to %i", sprayVal);
        }
        else
        {
            dprint(Serial, "Error: Failed to set spray text field to %i", sprayVal);
        }
    }
}

void print_time(TextField text, long time)
{
    char buffer[30];
    long minutes;
    float seconds;

    // Convert to minutes and seconds
    minutes = (time / 1000) / 60;
    seconds = (float)(time % 60000) / 1000.0;

    // Display output
    sprintf(buffer, "Elapsed: %ld:%05.2f", minutes, seconds);
    hmi_set_text_field(text, buffer);
}

void print_touches(TextField text, int thisTouch, int totalTouch)
{
    char buffer[30];
    sprintf(buffer, "Touch %i of %i", thisTouch, totalTouch);
    hmi_set_text_field(text, buffer);
}

void IRAM_ATTR interrupt_stepper()
{
    // Debounce
    if(millis() - limitDebounce > MOTOR_LIMIT_DEBOUNCE)
    {
        limitDebounce = millis();

        if(digitalRead(PIN_MOTOR_LIMIT))
        {
            StepperCtl.clearLimitSwitchActive();
        }
        else
        {
            StepperCtl.setLimitSwitchActive(ESP_FlexyStepper::LIMIT_SWITCH_BEGIN);
        }
    }
}

void check_pressure_delta()
{
    // Print out pressureDelta for debugging.
    if(abs(pressureDelta) * 100 > MAXIMUM_PRESSURE_D)
        {
            // If pressure delta exceeded set flag and enter TestPost
            currentState = MachineState::TestPost;
            pressureDeltaExceeded = true;
            dprint(Serial, "Pressure delta exceeded! (%f hPa/ms > %f hPa/ms)!", pressureDelta, (float)MAXIMUM_PRESSURE_D / 100);
            dprint(Serial, "New state: TestPost");
        }
}

void button_push_handler(ButtonPushEvent event)
{
    switch(event)
    {
        case(ButtonPushEvent::HomeTestIncrease):
            dprint(Serial, "Button push event: HomeTestIncrease");
            break;

        case(ButtonPushEvent::HomeTestDecrease):
            dprint(Serial, "Button push event: HomeTestDecrease");
            break;

        case(ButtonPushEvent::HomeSpeedIncrease):
            dprint(Serial, "Button push event: HomeSpeedIncrease");
            break;

        case(ButtonPushEvent::HomeSpeedDecrease):
            dprint(Serial, "Button push event: HomeSpeedDecrease");
            break;

        case(ButtonPushEvent::HomeForceIncrease):
            dprint(Serial, "Button push event: HomeForceIncrease");
            break;

        case(ButtonPushEvent::HomeForceDecrease):
            dprint(Serial, "Button push event: HomeForceDecrease");
            break;

        case(ButtonPushEvent::HomeSprayIncrease):
            dprint(Serial, "Button push event: HomeSprayIncrease");
            break;

        case(ButtonPushEvent::HomeSprayDecrease):
            dprint(Serial, "Button push event: HomeSprayDecrease");
            break;

        case(ButtonPushEvent::HomeStart):
            dprint(Serial, "Button push event: HomeStart");
            break;

        case(ButtonPushEvent::TestAbort):
            dprint(Serial, "Button push event: TestAbort");
            break;

        case(ButtonPushEvent::TestCompleteOk):
            dprint(Serial, "Button push event: TestCompleteOk");
            break;

        case(ButtonPushEvent::TestFailOk):
            dprint(Serial, "Button push event: TestFailOk");
            break;

        case(ButtonPushEvent::PressFailOk):
            dprint(Serial, "Button push event: PressFailOk");
            break;

        default:
            dprint(Serial, "Error: button_push_handler() switch statement reached default case!");
    }
}

void button_release_handler(ButtonReleaseEvent event)
{
    switch(event)
    {
        case(ButtonReleaseEvent::HomeTestIncrease):
            dprint(Serial, "Button release event: HomeTestIncrease");

            // Increment total tests by TEST_STEP if doing so won't exceed TEST_MAX
            if(totalTests + TEST_STEP <= TEST_MAX)
            {
                totalTests += TEST_STEP;
                char buffer[10];
                sprintf(buffer, "%i", totalTests);
                hmi_set_text_field(TextField::HomeTest, buffer);
            }
            // Otherwise total tests = TEST_MAX
            else
            {
                totalTests = TEST_MAX;
                char buffer[10];
                sprintf(buffer, "%i", TEST_MAX);
                hmi_set_text_field(TextField::HomeTest, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeTestDecrease):
            dprint(Serial, "Button release event: HomeTestDecrease");

            // Decrement total tests by TEST_STEP if doing so won't drop below TEST_MIN
            if(totalTests - TEST_STEP >= TEST_MIN)
            {
                totalTests -= TEST_STEP;
                char buffer[10];
                sprintf(buffer, "%i", totalTests);
                hmi_set_text_field(TextField::HomeTest, buffer);
            }
            // Otherwise total tests = TEST_MIN
            else
            {
                totalTests = TEST_MIN;
                char buffer[10];
                sprintf(buffer, "%i", TEST_MIN);
                hmi_set_text_field(TextField::HomeTest, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeSpeedIncrease):
            dprint(Serial, "Button release event: HomeSpeedIncrease");

            // Increment stepper speed by SPEED_STEP if doing so won't exceed SPEED_MAX
            if(stepperVelocity + SPEED_STEP <= SPEED_MAX)
            {
                stepperVelocity += SPEED_STEP;
                char buffer[10];
                sprintf(buffer, "%.1f mm/s", (float)stepperVelocity / 10);
                hmi_set_text_field(TextField::HomeSpeed, buffer);
            }
            // Otherwise stepper speed = SPEED_MAX
            else
            {
                stepperVelocity = SPEED_MAX;
                char buffer[10];
                sprintf(buffer, "%.1f mm/s", (float)SPEED_MAX / 10);
                hmi_set_text_field(TextField::HomeSpeed, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeSpeedDecrease):
            dprint(Serial, "Button release event: HomeSpeedDecrease");

            // Decrement stepper speed by SPEED_STEP if doing so won't drop below SPEED_MIN
            if(stepperVelocity - SPEED_STEP >= SPEED_MIN)
            {
                stepperVelocity -= SPEED_STEP;
                char buffer[10];
                sprintf(buffer, "%.1f mm/s", (float)stepperVelocity / 10);
                hmi_set_text_field(TextField::HomeSpeed, buffer);
            }
            // Otherwise stepper speed = SPEED_MIN
            else
            {
                stepperVelocity = SPEED_MIN;
                char buffer[10];
                sprintf(buffer, "%.1f mm/s", (float)SPEED_MIN / 10);
                hmi_set_text_field(TextField::HomeSpeed, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeForceIncrease):
            dprint(Serial, "Button release event: HomeForceIncrease");

            // Increment load cell threshold by FORCE_STEP if doing so won't exceed FORCE_MAX
            if(loadCellThreshold + FORCE_STEP <= FORCE_MAX)
            {
                loadCellThreshold += FORCE_STEP;
                char buffer[10];
                sprintf(buffer, "%i mN", loadCellThreshold);
                hmi_set_text_field(TextField::HomeForce, buffer);
            }
            // Otherwise load cell threshold = FORCE_MAX
            else
            {
                loadCellThreshold = FORCE_MAX;
                char buffer[10];
                sprintf(buffer, "%i mN", FORCE_MAX);
                hmi_set_text_field(TextField::HomeForce, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeForceDecrease):
            dprint(Serial, "Button release event: HomeForceDecrease");

            // Decrement load cell threshold by FORCE_STEP if doing so won't drop below FORCE_MIN
            if(loadCellThreshold - FORCE_STEP >= FORCE_MIN)
            {
                loadCellThreshold -= FORCE_STEP;
                char buffer[10];
                sprintf(buffer, "%i mN", loadCellThreshold);
                hmi_set_text_field(TextField::HomeForce, buffer);
            }
            // Otherwise load cell threshold = FORCE_MIN
            else
            {
                loadCellThreshold = FORCE_MIN;
                char buffer[10];
                sprintf(buffer, "%i mN", FORCE_MIN);
                hmi_set_text_field(TextField::HomeForce, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeSprayIncrease):
            dprint(Serial, "Button release event: HomeSprayIncrease");

            // Enable spraying if not already enabled
            if(!sprayEnabled)
            {
                sprayEnabled = true;
            }

            // Increment number of tests between sprays by SPRAY_STEP if doing so won't exceed SPRAY_MAX
            if(testsBetweenSprays + SPRAY_STEP <= SPRAY_MAX)
            {
                testsBetweenSprays += SPRAY_STEP;
                char buffer[10];
                sprintf(buffer, "%i", testsBetweenSprays);
                hmi_set_text_field(TextField::HomeSpray, buffer);
            }
            // Otherwise number of tests between sprays = SPRAY_MAX
            else
            {
                testsBetweenSprays = SPRAY_MAX;
                char buffer[10];
                sprintf(buffer, "%i", SPRAY_MAX);
                hmi_set_text_field(TextField::HomeSpray, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeSprayDecrease):
            dprint(Serial, "Button release event: HomeSprayDecrease");

            // Decrement number of tests between sprays by SPRAY_STEP if doing so won't drop below SPRAY_MIN
            // with a special case disabling sprays if the number = -1
            if(testsBetweenSprays - SPRAY_STEP < 0)
            {
                // Set sprayEnabled to false if not already false
                if(sprayEnabled)
                {
                    sprayEnabled = false;
                }

                testsBetweenSprays = -1;
                char buffer[4];
                sprintf(buffer, "%s", "OFF");
                hmi_set_text_field(TextField::HomeSpray, buffer);
            }
            else
            {
                testsBetweenSprays -= SPRAY_STEP;
                char buffer[10];
                sprintf(buffer, "%i", testsBetweenSprays);
                hmi_set_text_field(TextField::HomeSpray, buffer);
            }
            break;

        case(ButtonReleaseEvent::HomeStart):
            dprint(Serial, "Button release event: HomeStart");
            currentState = MachineState::TestInit;
            break;

        case(ButtonReleaseEvent::TestAbort):
            dprint(Serial, "Button release event: TestAbort");
            currentState = MachineState::TestAbort;
            break;

        case(ButtonReleaseEvent::TestCompleteOk):
            dprint(Serial, "Button release event: TestCompleteOk");
            hmi_set_page(Page::Home);
            update_home(totalTests, stepperVelocity, loadCellThreshold, testsBetweenSprays);
            break;

        case(ButtonReleaseEvent::TestFailOk):
            dprint(Serial, "Button release event: TestFailOk");
            hmi_set_page(Page::Home);
            update_home(totalTests, stepperVelocity, loadCellThreshold, testsBetweenSprays);
            break;

        case(ButtonReleaseEvent::PressFailOk):
            dprint(Serial, "Button release event: PressFailOk");
            hmi_set_page(Page::Home);
            update_home(totalTests, stepperVelocity, loadCellThreshold, testsBetweenSprays);
            break;

        default:
            dprint(Serial, "Error: button_release_handler() switch statement reached default case!");
    }
}
