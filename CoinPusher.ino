#include <JC_Button.h>
#include <PinChangeInterrupt.h>
// References
// Mega Interrupt (Default)                 : 2, 3, 18, 19, 20, 21

// GENERAL SETTINGS
#define SLEEP_TIME                  60      // In seconds
#define COIN_HOPPER_AMOUNT_FREE     10      // Number of coin to dispatch from the coin hidden button
#define COIN_HOPPER_AMOUNT          50      // NUmber of coin to dispatch from the coin acceptor
// COIN SLOPES SENSORS (PinChangeInterrupt)
// Arduino Mega : 10, 11, 12, 13, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69)
#define LEFT_COIN_SENSOR_PIN        10      // Coin inserted in left slot
#define MID_COIN_SENSOR_PIN         11      // Coin inserted in middle slot
#define RIGHT_COIN_SENSOR_PIN       12      // Coin inserted in right slot
// RELAYS
#define MOTOR_RELAY_PIN             25      // Relay used as a switch to enable/disable
#define COIN_HOPPER_RELAY_PIN       27      // Relay used as a switch to enable/disable

#define COIN_SWITCH_PIN             41      // Behind the cabinet, gives a small amout of coins to play with
#define PAUSE_SWITCH_PIN            43      // Activated when the motor is ON to turn the SKILL mode ON
#define GREEN_SWITCH_PIN            45      // Activated once the PAUSE switch has been pulled, make the SKILL mode OFF
#define COIN_ACCEPTOR_PIN           51      // Signal coming in, read as a switch, as it passes into a 12V/5V relay to get proper Arduino signal
#define COIN_HOPPER_PIN             53      // Signal coming in (Uses PinChangeInterrupt port)

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting Coin Pusher");

    setupMotor();
    setupCoinHopper();
    setupCoinSensors();

    setupSkillMode();

    delay(2000);
    Serial.println("Coin Pusher Ready!");
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

/*
 *  SKILL MODE
 *  TODO: Add timer to skill mode
 */
bool inSkillMode                = false;

Button pauseSwitch(PAUSE_SWITCH_PIN);
Button greenSwitch(GREEN_SWITCH_PIN);

void setupSkillMode()
{
    pauseSwitch.begin();
    greenSwitch.begin();    
}

void loopSkillMode()
{
    pauseSwitch.read();
    if(pauseSwitch.wasReleased() && inSkillMode == false)
    {
        Serial.println("- Pause switch released... Starting skill mode!");
        inSkillMode = true;
        stopMotor();
        updatePauseSwitchLEDStatus();
        updateGreenSwitchLEDStatus();
    }

    greenSwitch.read();
    if(greenSwitch.wasReleased() && inSkillMode == true)
    {
        Serial.println("- Green switch released... Stopping skill mode!");
        inSkillMode = false;
        startMotor();
        updatePauseSwitchLEDStatus();
        updateGreenSwitchLEDStatus();
    }       
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
 *  COIN HOPPER
 *  Update coin hopper relay status to turn on/off
 */
byte coinHopperAmountDispensed      = 0;
byte coinHopperAmountToBeDispensed  = 0;

Button coinSwitch(COIN_SWITCH_PIN);
Button coinAcceptorSwitch(COIN_ACCEPTOR_PIN);

void setupCoinHopper()
{
    pinMode(COIN_HOPPER_RELAY_PIN, OUTPUT);
    pinMode(COIN_HOPPER_PIN, INPUT_PULLUP);

    stopCoinHopper();

    // Starts button readings...
    coinSwitch.begin();
    coinAcceptorSwitch.begin();

    // Use interrupts to fetch coin hopper signal
    attachInterrupt(digitalPinToInterrupt(COIN_HOPPER_PIN), dispensedFromCoinHopper, FALLING);
}

void loopCoinHopper()
{
    coinSwitch.read();
    if(coinSwitch.wasReleased())
    {
        Serial.println("- Secret coin switch released...");
        coinHopperAmountToBeDispensed += COIN_HOPPER_AMOUNT_FREE;
        startCoinHopper();
    }

    coinAcceptorSwitch.read();
    if(coinAcceptorSwitch.wasReleased())
    {
        Serial.println("- Coin acceptor switch released...");
        coinHopperAmountToBeDispensed += COIN_HOPPER_AMOUNT;
        startCoinHopper();
    }
}

void stopCoinHopper()
{
    wakeMe();

    Serial.println("- Stopping coin hopper...");
    digitalWrite(COIN_HOPPER_RELAY_PIN, LOW);
    
}

void startCoinHopper()
{
    wakeMe();

    Serial.println("- Starting coin hopper...");
    digitalWrite(COIN_HOPPER_RELAY_PIN, HIGH);
    
}

void dispensedFromCoinHopper()
{
    if(coinHopperAmountToBeDispensed > 0)
    {
        ++coinHopperAmountDispensed;
        Serial.println((String) "- Dispensed from coin hopper: " + coinHopperAmountDispensed + "/ " + coinHopperAmountToBeDispensed);

        if(coinHopperAmountDispensed >= coinHopperAmountToBeDispensed)
        {
            stopCoinHopper();
            coinHopperAmountDispensed     = 0;
            coinHopperAmountToBeDispensed = 0;
        }
    }
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
    loopSkillMode();
    loopCoinHopper();

}
