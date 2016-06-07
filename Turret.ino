
#include <FatReader.h>
#include <SdReader.h>
#include <avr/pgmspace.h>
#include <SoftwareServo.h>
#include "WaveUtil.h"
#include "WaveHC.h"

/*************************************************************************
 * Arduino Portal Turret - (v 1.0)
 *
 * Author: Mark Alexander Barros
 * Arduino Turret.ino http://www.themadhermit.net/geek-out-build-your-own-arduino-portal-turret/ https://www.youtube.com/watch?v=7Z0CRfBR7MY
 * Description: 
 * Replicates the behavior of a Portal Turret using a PIR Sensor to detect
 * infra red motion (to wake up sleeping turret), a PING Sensor and Servo 
 * (to locate target within a predefined distance of the turret), a bright 
 * red LED (to simulate the laser designator - much safer then using an 
 * actual laser). A wave board is used to play the various portal sounds.
 * 
 * License:
 * This work is licenced under Crative Commons Attribution-ShareAlike 3.0 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * THE LICENSOR OFFERS THE WORK AS-IS AND MAKES NO REPRESENTATIONS OR 
 * WARRANTIES OF ANY KIND CONCERNING THE WORK, EXPRESS, IMPLIED, STATUTORY
 * OR OTHERWISE, INCLUDING, WITHOUT LIMITATION, WARRANTIES OF TITLE, 
 * MERCHANTIBILITY, FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT, OR
 * THE ABSENCE OF LATENT OR OTHER DEFECTS, ACCURACY, OR THE PRESENCE OF 
 * ABSENCE OF ERRORS, WHETHER OR NOT DISCOVERABLE.
 *  
 * Project Website
 *  + http://www.themadhermit.net/geek-out-build-your-own-arduino-portal-turret
 *
 *  Part List
 *  + Arduino Uno with Breadboard http://www.amazon.com/dp/B0051QHPJM
 *  + Ada Fruit Wave Shield       https://www.adafruit.com/products/94
 *  + PIR Sensor Module           http://www.radioshack.com/product/index.jsp?productId=12330342
 *  + Parallax PING Sensor        http://www.radioshack.com/product/index.jsp?productId=12326359 
 *  + Parallax Standard Servo     http://www.radioshack.com/product/index.jsp?productId=12296088
 *  + Super Bright Red LED        https://www.sparkfun.com/products/528
 *  + Round PCB Kit               http://www.radioshack.com/product/index.jsp?productId=3173937
 *  + Posts (Metal Standoffs)     http://www.radioshack.com/product/index.jsp?productId=2102848                       
 *  + Hook-up Wire                https://www.sparkfun.com/products/11367
 *  + Resistor (150 Ohm)
 *  + SD Card (64 MB or greater)
 *
 *  Required Libraries
 *  + WaveHC http://wavehc.googlecode.com/files/wavehc20110919.zip
 *    The WaveHC Library was developed for the Adafruit Arduino Wave Shield. 
 *    It supports both standard SD and high capacity SDHC flash cards. Sound
 *    files need to be copied to the root of the SD card using a PC. This 
 *    library will then be used to play them at the appropriate times.
 *  + Software Servo http://arduino.cc/playground/uploads/ComponentLib/SoftwareServo.zip
 *    
 * Documentation
 *  + WaveHC (see WaveHC.html located in zip File)
 *  + Software Servo http://www.arduino.cc/playground/ComponentLib/servo
 *
 * Other Files
 *  + Portal Turret Sound Files were downloaded from: http://theportalwiki.com/wiki/Category:Turret_voice_files
 *
 * Notes
 *  + For a great tutorial on WaveHC see http://www.ladyada.net/make/waveshield/libraryhc.html
 *    In fact, audio code in this files borrows heavily from it.
 * 
 ************************************************************************/
/**********************************************************************
 * Errata:
 * Need to use SoftwareServo.h instead of Servo.h due to coflict with 
 * timers of the WaveHC. The library for this can be downloaded here:
 * http://www.arduino.cc/playground/ComponentLib/Servo and placed it
 * in the user "libraries" folder:
 * C:\Users\<YourUsername>\Documents\Arduino\libraries
 * Also need to add the following line to the top of SoftwareServo.h
 * or you will get a slew of compiler errors.
 * #if ARDUINO >= 100
 *   #include "Arduino.h"
 * #else
 *   #include "wiring.h"
 *   #include "WProgram.h"
 * #endif
 **********************************************************************/
 
// This handy macro lets us determine how big the array up above is, by checking the size
#define countof(a) ((sizeof(a)/sizeof((a)[0])))

/**********************************************************************
 * THINGS YOU MAY WANT TO CUSTOMIZE
 **********************************************************************/
// This is the distance from the Ping sensor an object must be for the 
// laser turret to be able to lock-on and fire at it. 
#define INTRUDER_RANGE_DETECTION_IN_FEET                   2  

/**********************************************************************
 * PIN ASSIGNMENTS: (Due to the waveboard we have to use Analog pins)
 * NOTE: Pins 11,12, and 13 are the SPI connection to the SD card. 
 * Pin 10 is the chip select pin for the SD card.
 * You can use analog pins as digital pins. To do this you must refer 
 * to the analog pins by their digital name 14, 15, 16, 17, 18, and 19
 **********************************************************************/
#define INFRARED_MOTION_SENSOR      14 // This is how you reference Analog Pin A0 digitally
#define PING_PROXIMITY_SENSOR       15 // This is how you reference Analog Pin A1 digitally
#define RED_TARGETING_LED           16 // This is how you reference Analog Pin A2 digitally
#define SERVO                        6 // This is an available PWM pin

// TIMING (DELAYS AND PAUSES)
#define DIALOGUE_PAUSE             500 // Time (in ms) to wait after each command spoken by turret
#define CALL_OUT_TO_TARGET_DELAY  5000 // Time (in ms) to wait between commands spoken in "SEARCHING_STATE"
#define NO_MOTION_DETECTED           3 * CALL_OUT_TO_TARGET_DELAY
#define PIR_SETUP_TIME              30 // Time in seconds needed for PIR Sensor to stabalize on power-up

// TURRET STATES
#define SLEEPING_STATE               1
#define SEARCHING_STATE              2
#define TARGET_AQUIRED_STATE         3
#define FIRING_STATE                 4
#define TARGET_LOST_STATE            5
#define SLEEP_MODE_ACTIVATED         6

// MEASUREMENTS
#define FOOT_IN_INCHES                                    12
#define INTRUDER_RANGE_DETECTION_IN_INCHES    FOOT_IN_INCHES * INTRUDER_RANGE_DETECTION_IN_FEET
#define MICROSECONDS_PER_INCH                         73.746

//SERVO
#define SERVO_MIN_POSTION                  0
#define SERVO_MIDDLE_POSTION              90
#define SERVO_MAX_POSTION                180
#define SERVO_DELTA                       10 // The amount we want to increment or decrement the servo 

/**********************************************************************
 * VARIABLES
 **********************************************************************/
// FLAGS
boolean motionDetected         = false; // Holds the high/low value of the PIR sensor used to detect IR movement
boolean targetSeenByPingSensor = false; // Holds the high/low value of the PING sensor used to detect whether object is in front of turret

// OTHER VARIABLES
int currentState               = SLEEPING_STATE; // Keeps track of the current state the portal turret is in.

// TIME TRACKERS
unsigned long currentMillis    = 0;
unsigned long previousMillis   = currentMillis;
unsigned long startOfSearch    = 0;    // Used to keep track of when we started our current search

// SD CARD VARIABLES
SdReader  card;   // This object holds the information for the card
FatVolume vol;    // This holds the information for the partition on the card
FatReader root;   // This holds the information for the filesystem on the card
FatReader f;      // This holds the information for the file we're play
WaveHC    wave;   // This is the only wave (audio) object, since we will only play one at a time

// SOUND FILES - FILES WE COPIED TO THE ROOT OF THE SD CARD. NOTE: THEY HAD TO BE IN 8.3 FORMAT
//               HENCE THE GOOFY NAMES
char* turretDetectionWavFiles[]  = {"iseeyou.wav", "targetac.wav", "thereur.wav", "thereur2.wav"};
char* turretActivationWavFiles[] = {"activate.wav", "shutdown.wav"};
char* turretSearchingWavFiles[]  = {"search.wav", "rustill.wav", "whosther.wav", "canihelp.wav", "targlost.wav"};
char* turretPowerDownWavFiles[]  = {"sleepmod.wav", "hibernat.wav", "resting.wav", "goodnigh.wav"};
char* turretCriticalWavFiles[]   = {"critical.wav"};
char* turretAttackWavFiles[]     = {"fire4.wav", "fire5.wav", "fire6.wav"};
char* turretMovedWavFiles[]      = {"hey.wav", "pputmedn.wav", "putmedwn.wav"};

// SERVO
SoftwareServo   servo; // Create servo Object
int             servoPosition = SERVO_MIDDLE_POSTION;
boolean         sweepingRight = true;

/**********************************************************************
 * SETUP - BY CONVENTION, THIS IS THE 1ST FUNCTION ARDUINO ALWAYS CALLS.
 * WE USE IT TO SETUP THE HARDWARE
 **********************************************************************/
void setup() {

  // set up serial port
  Serial.begin(9600);
  putstring_nl("Portal Turret Initializing");
  
  // Set the pins to input or output as needed
  pinMode(INFRARED_MOTION_SENSOR, INPUT);
  pinMode(PING_PROXIMITY_SENSOR, INPUT);
  pinMode(RED_TARGETING_LED, OUTPUT);
  
  servo.attach(SERVO);
  
  /****************************************************************************************
   * Initialize/Setup SD Card : Borrowed from Adafruit WaveHC Tutorial (See Notes) 
   ****************************************************************************************/
  putstring("Free RAM: ");       // This can help with debugging, running out of RAM is bad
  Serial.println(freeRam());     // if this is under 150 bytes it may spell trouble!
  
  if (!card.init())              //play with 8 MHz spi (default faster!)  
  {
    putstring_nl("Card init. failed!");  // Something went wrong, lets print out why
    sdErrorCheck();
    while(1);                            // then 'halt' - do nothing!
  }
  
  // enable optimize read - some cards may timeout. Disable if you're having problems
  card.partialBlockRead(true);
 
// Now we will look for a FAT partition!
  uint8_t part;
  for (part = 0; part < 5; part++)      // we have up to 5 slots to look in
  {     
    if (vol.init(card, part)) 
      break;                            // we found one, lets bail
  }
  
  if (part == 5)                        // if we ended up not finding one  :(
  {
    putstring_nl("No valid FAT partition!");
    sdErrorCheck();                    // Something went wrong, lets print out why
    while(1);                          // then 'halt' - do nothing!
  }
  
  // Lets tell the user about what we found
  putstring("Using partition ");
  Serial.print(part, DEC);
  putstring(", type is FAT");
  Serial.println(vol.fatType(),DEC);     // FAT16 or FAT32?
  
  // Try to open the root directory
  if (!root.openRoot(vol)) 
  {
    putstring_nl("Can't open root dir!"); // Something went wrong,
    while(1);                             // then 'halt' - do nothing!
  }

  /****************************************************************************************
   * END Initialize/Setup SD Card : Borrowed from Adafruit WaveHC Tutorial (See Notes) 
   ****************************************************************************************/  
  
  // Make sure the turret is facing forward on power-up
  moveTurretHome();
  
  // PIR Motion sensor needs to sample abient conditions for at least 30 seconds  
  // to establish a baseline before it's ready. This loop will flash an LED to 
  // provide a visible indication to the user. 
  for (int i = 0; i < (PIR_SETUP_TIME * 2); i++)
  { 
    digitalWrite(RED_TARGETING_LED, HIGH);
    delay(250); 
    digitalWrite(RED_TARGETING_LED, LOW);
    delay(250); 
  }  

  // Whew! We got past the tough parts.
  putstring_nl("Ready!");
  
  playcomplete(turretActivationWavFiles[0]);
}




/**********************************************************************
 * LOOP - THIS IS THE SECOUND FUNCTION ARDUINO CALLS. THIS IS WHERE WE 
 * DO ALL OUR WORK. THE DEVICE WILL COUNTINOUSLY CYCLE THROUGH THIS 
 * LOOP UNTIL THE POWER IS PULLED.
 **********************************************************************/

void loop() 
{

  // Check the status of the infrared sensor. Is a carbon-based life-form moving within range?
  motionDetected = digitalRead(INFRARED_MOTION_SENSOR);

  // Check PING Motion Sensor to see if something is all up in the turrets grill.
  // NOTE: Only do this when the turret is awake 
  if (currentState != SLEEPING_STATE)
  {
      delay(40); // Make sure servo has stopped before checking otherwise we will get a false detection
      targetSeenByPingSensor = searchForObjectInRange();
   
  }
  
  // Enter the state machine where the turret performs specific actions depending on the state it is in.
  switch (currentState)
  {
    case SLEEPING_STATE:

      // Just stay sleeping unless motion is detected
      putstring_nl("SLEEPING_STATE");
    
      if(motionDetected)
      {
        putstring_nl("SLEEPING_STATE - Motion Detected");
        
        // Turn on laser  
        digitalWrite(RED_TARGETING_LED , HIGH);
       
        // Say: "Searching"
        playcomplete(turretSearchingWavFiles[0]);
      
        // Let's get the current time so we can periodically call out to the user as long as his/her presence is "sensed".
        previousMillis = millis();
      
        delay(DIALOGUE_PAUSE);
        
        // Go to next state
        currentState = SEARCHING_STATE;
      }

      break;
      
    case SEARCHING_STATE:
      
      // Let's get the current time in milliseconds so we know when to abandon our search.
      // NOTE: Need to make sure that we set this variable (startOfSearch to zero prior to 
      // leaving this state so the next time we enter it we can get the time again).
      if (startOfSearch == 0)
        startOfSearch = millis();
        
      // Print debug message
      putstring_nl("SEARCHING_STATE");
       
      if (targetSeenByPingSensor)
      {
        putstring_nl("SEARCHING_STATE - Target Seen");
        currentState = TARGET_AQUIRED_STATE;
        startOfSearch = 0; // Since we are changing state we need to reset this
      }
      else // Continue to search for a little while
      {
        putstring_nl("SEARCHING_STATE - Searching");
        
        // Figure out how much time has passed
        currentMillis = millis();

        //moveTurret(servo, servoPosition, sweepingRight);
        moveTurret();
        
        if (currentMillis > previousMillis + CALL_OUT_TO_TARGET_DELAY)
        {
          // Decide what we are going to say. 
          if (!motionDetected && currentMillis > startOfSearch + NO_MOTION_DETECTED)
          {
            putstring_nl("SEARCHING_STATE - Switch to Sleep Mode");
            currentState = SLEEP_MODE_ACTIVATED;
            startOfSearch = 0;  // Since we are changing state we need to reset this
          } 
          else
          {
            putstring_nl("SEARCHING_STATE - Call out to intruder");
           
            playcomplete(turretSearchingWavFiles[(rand()%4)]);
            delay(DIALOGUE_PAUSE);
           
            previousMillis = millis();  
          }
        }
      }  
    
      break;
      
    case TARGET_AQUIRED_STATE:
      
      putstring_nl("TARGET_AQUIRED_STATE - Acknowledge Object Before Firing");

      playcomplete(turretDetectionWavFiles[(rand()%4)]);
      delay(DIALOGUE_PAUSE/2);

      currentState = FIRING_STATE;

      break;
    
    case FIRING_STATE:
    
      if (targetSeenByPingSensor)
      {  
        putstring_nl("FIRING_STATE - Shoot to kill");
        playcomplete(turretAttackWavFiles[(rand()%3)]);
      }
      else
      {
          currentState = TARGET_LOST_STATE;
      }
    
      break;
 
    case TARGET_LOST_STATE:
    
      putstring_nl("TARGET_LOST_STATE");
      
      playcomplete(turretSearchingWavFiles[4]);
      delay(2 * DIALOGUE_PAUSE);
      
      currentState = SEARCHING_STATE;
      
      break;

    case SLEEP_MODE_ACTIVATED:

      putstring_nl("SLEEPING_MODE_ACTIVATED");
            
      moveTurretHome();
      
      // Turn off wake LED
      digitalWrite(RED_TARGETING_LED , LOW);
      
      playcomplete(turretPowerDownWavFiles[rand()%4]);
      delay(DIALOGUE_PAUSE);
      
      currentState = SLEEPING_STATE;
      
      break;
      
    default:
      putstring_nl("DEFAULT");
        
  } // End Switch/Case 

} // End loop()





/**********************************************************************
 * PING SENSOR FUNCTIONS
 **********************************************************************/
 
boolean searchForObjectInRange()
{
  long duration;
  float inches;
  boolean result = false;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(PING_PROXIMITY_SENSOR, OUTPUT);
  digitalWrite(PING_PROXIMITY_SENSOR, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PROXIMITY_SENSOR, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PROXIMITY_SENSOR, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PING_PROXIMITY_SENSOR, INPUT);
  duration = pulseIn(PING_PROXIMITY_SENSOR, HIGH);
  
  // convert the time into a distance
  inches = microsecondsToInches(duration);

  if (inches <= (INTRUDER_RANGE_DETECTION_IN_INCHES))
    result = true;
  
  return (result);
}

float microsecondsToInches(float microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / MICROSECONDS_PER_INCH / 2;
}



/**********************************************************************
 * AUDIO FUNCTIONS - Borrowed from AdaFruit WaveHC Tutorial (See notes)
 **********************************************************************/

void playTurretSound(char** sounds, int numberOfSounds, int pause)
{
  putstring("Number of Files Found: ");
  Serial.println(numberOfSounds);  

  for (int i = 0; i < numberOfSounds; i++)
  {
    putstring("Playing: "); 
    Serial.println(sounds[i]); 
    playcomplete(sounds[i]);
    delay (pause);
  } 
}



// Plays a full file from beginning to end with no pause.
void playcomplete(char *name) 
{
  // call our helper to find and play this name
  playfile(name);
  while (wave.isplaying) 
  {
    // do nothing while its playing
  }
  // now its done playing
}


void playfile(char *name) 
{
  // see if the wave object is currently doing something
  if (wave.isplaying) 
  {// already playing something, so stop it!
    wave.stop(); // stop it
  }
  // look in the root directory and open the file
  if (!f.open(root, name)) 
  {
    putstring("Couldn't open file "); Serial.print(name); return;
  }
  // OK read the file and turn it into a wave object
  if (!wave.create(f)) 
  {
    putstring_nl("Not a valid WAV"); return;
  }
  
  // ok time to play! start playback
  wave.play();
}


/**********************************************************************
 * SERVO FUNCTIONS
 **********************************************************************/ 

void moveTurret()
{
  if (sweepingRight)
  {
    if (servoPosition < SERVO_MAX_POSTION)
    {
      servoPosition += SERVO_DELTA;
    }
    else
    {
      sweepingRight = false;
      servoPosition -= SERVO_DELTA;
    }
  }
  else // Sweep Left
  {
    if (servoPosition > SERVO_MIN_POSTION)
    {
      servoPosition -= SERVO_DELTA;
    }
    else
    {
      sweepingRight = true;
      servoPosition += SERVO_DELTA;
    }
  }
  
  putstring_nl("Moving Servo To Postion ");
  Serial.println(servoPosition);

  // Move the survo
  servo.write(servoPosition);
  delay(20);
  SoftwareServo::refresh();
}


void moveTurretHome()
{
  unsigned long tStart, tEnd;
  
  putstring_nl("Moving Servo Home: ");
  Serial.println(SERVO_MIDDLE_POSTION);
  
  
  // Move the survo home
  servo.write(SERVO_MIDDLE_POSTION);

  tStart = millis();
  tEnd = tStart;
  
  while (tStart < tEnd + 2000)
  {
    delay(20); 
    SoftwareServo::refresh();
    tStart = millis();
  }
}

/**********************************************************************
 * SD CARD FUNCTIONS
 **********************************************************************/ 
 
// this handy function will return the number of bytes currently free in RAM, great for debugging!   
int freeRam(void)
{
  extern int  __bss_end; 
  extern int  *__brkval; 
  int free_memory; 
  if((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end); 
  }
  else {
    free_memory = ((int)&free_memory) - ((int)__brkval); 
  }
  return free_memory; 
} 

void sdErrorCheck(void)
{
  if (!card.errorCode()) return;
  putstring("\n\rSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  putstring(", ");
  Serial.println(card.errorData(), HEX);
  while(1);
}

