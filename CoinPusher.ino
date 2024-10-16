#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
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

// SWITCHES
#define COIN_SWITCH_PIN             41      // Behind the cabinet, gives a small amout of coins to play with
#define PAUSE_SWITCH_PIN            43      // Activated when the motor is ON to turn the SKILL mode ON
#define GREEN_SWITCH_PIN            45      // Activated once the PAUSE switch has been pulled, make the SKILL mode OFF
#define RED_SWITCH_PIN              47      // Behind the cabinet, used to switch LED animation mode

// SWITCHES LED, uses PWM for fading
#define COIN_SWITCH_LED_PIN          3
#define PAUSE_SWITCH_LED_PIN         4
#define GREEN_SWITCH_LED_PIN         5
#define RED_SWITCH_LED_PIN           6

#define COIN_SWITCH_LED_TIMEOUT     2500
#define PAUSE_SWITCH_LED_TIMEOUT    COIN_SWITCH_LED_TIMEOUT
#define GREEN_SWITCH_LED_TIMEOUT    COIN_SWITCH_LED_TIMEOUT
#define RED_SWITCH_LED_TIMEOUT      COIN_SWITCH_LED_TIMEOUT

#define COIN_ACCEPTOR_PIN           51      // Signal coming in, read as a switch, as it passes into a 12V/5V relay to get proper Arduino signal
#define COIN_HOPPER_PIN             53      // Signal coming in (Uses PinChangeInterrupt port)

// Not required, but be careful not to use them, they are reserved for the LCD
#define LCD_SDA                     20
#define LCD_SCL                     21

/** 
 *  LED STRIPES
 */
#define LED_STRIPE_CHIPSET      WS2812B
#define LED_FRAMES_PER_SECOND   60

#define LEFT_LED_STRIPE_PIN     31
#define MID_LED_STRIPE_PIN      33
#define RIGHT_LED_STRIPE_PIN    35

#define SIDE_LED_NUM            36
#define TOP_LED_NUM             24

// Convenient functions
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting Coin Pusher");

    setupMotor();
    setupCoinHopper();
    setupCoinSensors();

    setupLcdMonitor();
    setupSwitchLED();
    setupLED();

    setupSkillMode();

    delay(2000);
    Serial.println("Coin Pusher Ready!");
}

/*
 *  LCD MONITOR
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setupLcdMonitor()
{
    lcd.init();                // initialize the lcd
    lcd.backlight();           // Turn on backlight
    lcd.setCursor(0, 0);
    lcd.print("  La Chaclaude  ");// Print a message to the LCD
    lcd.setCursor(0, 1);
    lcd.print("   Coin Pusher  ");
    lcd.clear();
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

/*
 *  LED STRIPES
 */
CRGB leftLedStripe[SIDE_LED_NUM];
CRGB midLedStripe[TOP_LED_NUM];
CRGB rightLedStripe[SIDE_LED_NUM];

// Read from EEPROM to check the value of the last mode used
bool sleepSwitchState   = false; 
int sleepAnimationMode  = 0; 
int activeAnimationMode = 0;

Button redSwitch(RED_SWITCH_PIN);

// List of sleep pattern available
typedef void (*SleepPatternList[])();
SleepPatternList sleepPatterns  = { sleepAnimationRainbow, sleepAnimationRainbowWithGlitter, sleepAnimationConfetti };
uint8_t sleepHue                = 0;

void setupLED()
{
    FastLED.clear(true);
    FastLED.setBrightness(255);

    FastLED.addLeds<LED_STRIPE_CHIPSET, LEFT_LED_STRIPE_PIN, GRB>(leftLedStripe, SIDE_LED_NUM);
    FastLED.addLeds<LED_STRIPE_CHIPSET, MID_LED_STRIPE_PIN, GRB>(midLedStripe, TOP_LED_NUM);
    FastLED.addLeds<LED_STRIPE_CHIPSET, RIGHT_LED_STRIPE_PIN, GRB>(rightLedStripe, SIDE_LED_NUM);

    EEPROM.get(0, sleepAnimationMode);
    EEPROM.get(sizeof(sleepAnimationMode), activeAnimationMode);

    redSwitch.begin();
}

void loopRedSwitch()
{
    if(redSwitch.pressedFor(1000))
    {
        Serial.println("- Red switch long press...");
        sleepSwitchState = true;
        sleepMode();
    }
    else
    {
        if(redSwitch.wasReleased())
        {
            if(sleepSwitchState == true) // Just track ther long press release...
            {
                sleepSwitchState = false;
            }
            else
            {
                updateAnimationMode();
            }
        }
    }
}

void updateAnimationMode()
{
    if(inSleepMode == true) // Loop available sleep mode...
    {
        sleepAnimationMode = (sleepAnimationMode + 1) % ARRAY_SIZE(sleepPatterns);
        Serial.println((String) "- New sleepAnimationMode: " + sleepAnimationMode);

        EEPROM.put(0, sleepAnimationMode);        
    }
    else
    {
        //TODO: Change main color?
    }
}

void updateLEDStripeFrame()
{
    if(inSleepMode == true)
    {
        // While in sleep mode, we slowly fade to black the top stripe
        if(midLedStripe != CRGB(0,0,0))
        {
            fadeToBlackBy(midLedStripe, TOP_LED_NUM, 10); // 10/255 = 4%
        }
        
        // Run the current selected sleep animation
        sleepPatterns[sleepAnimationMode]();
    }
    else
    {   
        // TODO: We need to go into white mode
        //fadeToWhiteBy(midLedStripe, TOP_LED_NUM, 10);
    }
}

/**
 * SWITCH LED
 */
int coinLedState                = LOW;
int pauseLedState               = LOW;
int greenLedState               = LOW;

void setupSwitchLED()
{
    pinMode(COIN_SWITCH_LED_PIN, OUTPUT);
    pinMode(PAUSE_SWITCH_LED_PIN, OUTPUT);
    pinMode(GREEN_SWITCH_LED_PIN, OUTPUT);
    pinMode(RED_SWITCH_LED_PIN, OUTPUT);
}
void updateCoinSwitchLEDStatus()
{
    coinLedState = !coinLedState;
    digitalWrite(COIN_SWITCH_LED_PIN, coinLedState);
}

void updatePauseSwitchLEDStatus()
{
    // Turn PAUSE led off if we are in sleep or skill mode
    if(inSleepMode == true || inSkillMode == true)
    {
        pauseLedState = LOW;
    }
    else
    {
        pauseLedState = !pauseLedState;
    }
    
    digitalWrite(PAUSE_SWITCH_LED_PIN, pauseLedState);
}

void updateGreenSwitchLEDStatus()
{
    greenLedState = !greenLedState;
    digitalWrite(GREEN_SWITCH_LED_PIN, greenLedState);
}

int redSwitchLedBrightness      = 0;
bool redSwitchLedIncreasing     = true;
void updateRedSwitchLEDStatus()
{
    (redSwitchLedIncreasing == true) ? ++redSwitchLedBrightness : --redSwitchLedBrightness;
    redSwitchLedIncreasing = (redSwitchLedBrightness >= 255) ? false: true;

    analogWrite(RED_SWITCH_LED_PIN, redSwitchLedBrightness);
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
 *  ANIMATIONS FRAMES
 */
void sleepAnimationRainbow()
{
    fill_rainbow(leftLedStripe, SIDE_LED_NUM, sleepHue, 7);
    fill_rainbow(rightLedStripe, SIDE_LED_NUM, sleepHue, 7);
}

void sleepAnimationRainbowWithGlitter()
{
    sleepAnimationRainbow();
    uint8_t chanceOfGlitter = 80;

    if(random8() < chanceOfGlitter)
    {
        leftLedStripe[random16(SIDE_LED_NUM)] += CRGB::White;
    }
    if(random8() < chanceOfGlitter)
    {
        rightLedStripe[random16(SIDE_LED_NUM)] += CRGB::White;
    }
}

void sleepAnimationConfetti()
{
    fadeToBlackBy(leftLedStripe, SIDE_LED_NUM, 10);
    fadeToBlackBy(rightLedStripe, SIDE_LED_NUM, 10);

    int posLeft                 = random16(SIDE_LED_NUM);
    leftLedStripe[posLeft]     += CHSV(sleepHue + random8(64), 200, 255);

    int posRight                = random16(SIDE_LED_NUM);
    rightLedStripe[posRight]   += CHSV(sleepHue + random8(64), 200, 255);
}


/**
 *  LOOP AT THE END SO EVERY VARIABLES ARE USABLE...
 */
void loop()
{
    loopSleepModeTimer();
    loopSkillMode();
    loopCoinHopper();
    loopRedSwitch();

    EVERY_N_MILLISECONDS( 20 ) { sleepHue++; }
    EVERY_N_MILLISECONDS(1000 / LED_FRAMES_PER_SECOND)
    {
        updateLEDStripeFrame();
        FastLED.show();
    }

    // Blink?
    EVERY_N_MILLISECONDS(COIN_SWITCH_LED_TIMEOUT){ updateCoinSwitchLEDStatus(); }
    EVERY_N_MILLISECONDS(PAUSE_SWITCH_LED_TIMEOUT){ updatePauseSwitchLEDStatus(); }
    EVERY_N_MILLISECONDS(GREEN_SWITCH_LED_TIMEOUT){ updateGreenSwitchLEDStatus(); }
    EVERY_N_MILLISECONDS(RED_SWITCH_LED_TIMEOUT){ updateRedSwitchLEDStatus(); }
}

