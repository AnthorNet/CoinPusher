#include <PinChangeInterrupt.h>
// References
// Mega Interrupt (Default)                 : 2, 3, 18, 19, 20, 21
// Arduino Mega (PinChangeInterrupt)        : 10, 11, 12, 13, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69)
#define SLEEP_TIME                  60      // In seconds
// COIN SLOPES SENSORS (PinChangeInterrupt)
#define LEFT_COIN_SENSOR_PIN        10      // Coin inserted in left slot
#define MID_COIN_SENSOR_PIN         11      // Coin inserted in middle slot
#define RIGHT_COIN_SENSOR_PIN       12      // Coin inserted in right slot
// RELAYS
#define MOTOR_RELAY_PIN             25      // Relay used as a switch to enable/disable
void setup()
{
    Serial.begin(9600);
    Serial.println("Starting Coin Pusher");

    setupMotor();
    setupCoinSensors();

    delay(2000);
}


/**
 *  SLEEP MODE
 */
unsigned long lastWakeUpTime    = 0;      // Every actions will trigger this and write millis()
bool inSleepMode                = false;

void loopSleepModeTimer()
{
    unsigned long currentMillis = millis();
    unsigned long sleepMillis   = (lastWakeUpTime + (SLEEP_TIME * 1000UL));

    if(currentMillis >= sleepMillis && inSleepMode == false)
    {
        sleepMode();
    }
}
void sleepMode()
{
    Serial.println("- Entering sleep mode...");

    inSleepMode = true;
    stopMotor();

    //TODO: Write something on the screen...
}
void wakeMe()
{
    if(inSleepMode == true)
    {
        Serial.println("- Leaving sleep mode...");
        startMotor();
    }

    inSleepMode     = false;
    lastWakeUpTime  = millis();    
}


/**
 *  COIN SENSORS
 */
void setupCoinSensors()
{
    pinMode(LEFT_COIN_SENSOR_PIN, INPUT_PULLUP);
    pinMode(MID_COIN_SENSOR_PIN, INPUT_PULLUP);
    pinMode(RIGHT_COIN_SENSOR_PIN, INPUT_PULLUP);

    attachPCINT(digitalPinToPCINT(LEFT_COIN_SENSOR_PIN), triggerLeftCoin, RISING);
    attachPCINT(digitalPinToPCINT(MID_COIN_SENSOR_PIN), triggerMidCoin, RISING);
    attachPCINT(digitalPinToPCINT(RIGHT_COIN_SENSOR_PIN), triggerRightCoin, RISING);
}

void triggerLeftCoin()
{
    wakeMe();

    Serial.println("- Left coin slope...");
}

void triggerMidCoin()
{
    wakeMe();

    Serial.println("- Middle coin slope...");
}

void triggerRightCoin()
{
    wakeMe();

    Serial.println("- Right coin slope...");
}
/**
 *  MOTOR
 *  Update motor relay status to turn on/off
 */
void setupMotor()
{
    pinMode(MOTOR_RELAY_PIN, OUTPUT);
    stopMotor();
}

void stopMotor()
{
    Serial.println("- Stopping motor...");
    digitalWrite(MOTOR_RELAY_PIN, LOW);
}
void startMotor()
{
    Serial.println("- Starting motor...");
    digitalWrite(MOTOR_RELAY_PIN, HIGH);
}

/**
 *  LOOP AT THE END SO EVERY VARIABLES ARE USABLE...
 */
void loop()
{
    loopSleepModeTimer();

}
