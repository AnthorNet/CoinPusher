#define SLEEP_TIME                  60      // In seconds
// RELAYS
#define MOTOR_RELAY_PIN             25      // Relay used as a switch to enable/disable
void setup()
{
    Serial.begin(9600);
    Serial.println("Starting Coin Pusher");

    setupMotor();

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
