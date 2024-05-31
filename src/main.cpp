///////////////////////////////////////////////////////////////////////////////////////////////////
//        INFORMATION                                                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
/*
    This is a code developed as part of a final project for the degree in Electronic and Automatic
    Industrial Engineering. The code will be used in a Heltec Cubecell HTCC-AB01 to monitor the
    vitals of cattle, using an accelerometer and temperature module. The GPS location will also be
    gathered, and this information will be sent to a private database using LoRaWAN communication.

    Álvaro Alemany Suárez , July 2023
*/

///////////////////////////////////////////////////////////////////////////////////////////////////
//        INCLUSIONS AND DEFINITIONS                                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "Arduino.h"		                      // basic
#include "LoRaWan_APP.h"	                    // LoRa
#include "TTNvalues.h"                        // LoRa
#include <Adafruit_MPU6050.h>                 // accelerometer
#include <SoftSerial.h>		                    // GPS
#include <TinyGPS++.h>		                    // GPS

///////////////////////////////////////////////////////////////////////////////////////////////////
//        DECLARATION OF VARIABLES AND FUNCTIONS                                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////

// ACCELEROMETER ----------------------------------------------------------------------------------
Adafruit_MPU6050 mpu;                         // We create an object for the sensor. This object will have certain parameters stored.
void Init_Motion();                           // declaring the function to initiate the accelerometer as a motion sensor
void Detect_Motion();                         // declaring the function to detect the motion and print the data on screen
bool mov = 0;                                 // variable for the presence of movement

// GPS MODULE -------------------------------------------------------------------------------------
TinyGPSPlus gps;                              // object for the GPS
long        lat = 0 , lon = 0 ;               // global variables for latitude and longitude
int         age = 0;                          // "age" of the signal : time ellapsed since the last GPS fix
void        Get_GPS_Data();                   // function that will read, process and store GPS data
void        Smart_Delay(unsigned long);       // a delay function that still gets GPS data
void        Wait_for_Data(unsigned long ms);  // similar to Smart_Delay, but with the option to return as soon as valid data is received
static const uint32_t GPSBaud = 9600;         // baudrate for communicating with the GPS
softSerial ss(GPIO3 ,                         /* Pin of the CubeCell used as TX -> orange wire -> to GPS-RX */
              GPIO2 );                        /* Pin of the CubeCell used as RX -> green wire -> to GPS-TX */

// LoRaWAN ---------------------------------------------------------------------------------------
void  LoRaWAN_Loop();                         // will be used for simplicity of the loop function, and won't be touched
void  Prepare_Data( uint8_t );                // the most important function, prepares the data to be sent

///////////////////////////////////////////////////////////////////////////////////////////////////
//        Miscellaneous functions                                                                //
///////////////////////////////////////////////////////////////////////////////////////////////////

void Prepare_Data( uint8_t port )    // obtains data from the sensors, processes them and prepares the message
{
  Smart_Delay(10);                            // gives them a small time to reach steady state
  Serial.println("GPS Searching...");         // and prints the message on screen

  Wait_for_Data(1000);                        // we give it some time to get a proper fix. Default: 10000

  if(gps.location.age() < 1000)               // if we get a proper fix of the GPS
  { Get_GPS_Data(); }                         // we store the desired data and print it on screen
  
  else                                        // if we can't get a proper fix of the GPS data
  {                                           // we do two things:
    Serial.println("No GPS signal");          // - we print a warning on screen
    Smart_Delay(2000);                        // - and wait                                          
  }

  if(mpu.getMotionInterruptStatus()){         // then, we obtain the data from the accelerometer
    Serial.println("Motion: Yes");            // if motion is detected, we print this
    mov = 1;                                  // and update the variable
  }
  else{                                       // if motion was not detected by the filter
    Serial.println("Motion: No");             // we print this message
    mov = 0;                                  // and update the variable
  }
  
  sensors_event_t a, g, temp;     // this is a structure that provides a single sensor event in a common format
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Temp: ");   int temp_cents_deg = static_cast<int>(temp.temperature*100);  Serial.print(temp_cents_deg); Serial.println(" cents of a ºC");

  // Now we are going to divide our data into bytes. There is a payload formatter in TTN that will reconstruct the message.

  appDataSize = 11;                           // Number of characters that are going to be sent

  appData[3] = lat;                           // as the latitude is going to be sent as a "long" integer
  appData[2] = lat >> 8;                      // it has 32 bits of length, that is, 4 bytes
  appData[1] = lat >> 16;                     // so we separate it into four byte pieces
  appData[0] = lat >> 24;                     // in order to reconstruct it later in TTN with its payload formatter

  appData[7] = lon;                           // we do the same for the longitude
  appData[6] = lon >> 8;
  appData[5] = lon >> 16;
  appData[4] = lon >> 24;

  appData[8] = highByte(temp_cents_deg);      // and we do the same for the temperature, separated in two bytes
  appData[9] = lowByte(temp_cents_deg);       // which means the maximum temp is 655.35 ºC

  appData[10] = mov;                          // this will be used as a boolean to indicate movement or lack thereof        
}

// ------------------------------------------------------------------------------------------------
void LoRaWAN_Loop()
{
  switch( deviceState )                         // looks at the state of the CubeCell device
  {
    case DEVICE_STATE_INIT:                     // at the beginning, it will be in its initialization state
    {
      printDevParam();                          // We show its parameters on screen
      LoRaWAN.init(loraWanClass,loraWanRegion); // And initialize the device according to the values defined
      deviceState = DEVICE_STATE_JOIN;          // When it is done, switches to the next state (join)
      break;
    }

    case DEVICE_STATE_JOIN:                     // in this state, it tries to connect to TTN
    {    
      LoRaWAN.join();                           // trying to establish the connection again and again. 
      break;                                    // Only after a successful connection will the device get out of this state
    }

    case DEVICE_STATE_SEND:                     // When the connection is established and TTN is ready to receive the message...
    {
      Prepare_Data( appPort );                  // We call the function to prepare the data
      LoRaWAN.send();                           // and send the payload

      deviceState = DEVICE_STATE_CYCLE;         // when the message is sent, we switch to the next state
      break;                                    
    }

    case DEVICE_STATE_CYCLE:                    // in this state we define when we will try to send the next message
    {
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );  // The time has a minimum part and a random addition
      LoRaWAN.cycle(txDutyCycleTime);           // then, we "set an alarm" to wake up LoRaWAN communication
      deviceState = DEVICE_STATE_SLEEP;         // and go to sleep
      break;
    }

    case DEVICE_STATE_SLEEP:                    // here, all we do is sleep until txDutyCycleTime has passed
    {
      LoRaWAN.sleep();
      break;
    }

    default:                                    // if no case is selected or it is not one of those before
    {
      deviceState = DEVICE_STATE_INIT;          // we restart the process
      break;
    }
  }
}

// ------------------------------------------------------------------------------------------------
void Get_GPS_Data()
{
    Serial.print("Prec: ");  Serial.print(gps.hdop.hdop());
    
    float flat = gps.location.lat()*1000000;
    lat = static_cast<long>(flat);
    Serial.print("  Lat_int: ");  Serial.printf("%d",lat);

    float flon = gps.location.lng()*1000000;
    lon = static_cast<long>(flon);
    Serial.print("  Lon_int: ");  Serial.printf("%d",lon);

    age = gps.location.age();
    Serial.print("  Age: ");      Serial.printf("%d",age); 

    Serial.println();
}

// ------------------------------------------------------------------------------------------------
void Smart_Delay(unsigned long ms)  // This custom version of delay() ensures that the gps object is being "fed".
{
  unsigned long start = millis();   // records the current time in milliseconds
  do {                              // and does the two following lines
    while (ss.available())          //    (first checking if there is available data and if so,
      gps.encode(ss.read());        //     then encoding the GPS data so nothing is lost)
  } while (millis() - start < ms);  // until the specified number of ms has passed
}

// ------------------------------------------------------------------------------------------------
void Wait_for_Data(unsigned long ms)          // this is a function similar to Smart_Delay but it stops early when it gets a fix
{
  uint32_t start = millis();        // records the current time in milliseconds
  while( (millis()-start) < ms )    // as long as the specified time hasn't ellapsed
  {                                 // it does the following lines:
    while (ss.available() > 0)        gps.encode(ss.read());  // it encodes a new piece of data whenever it is available
    if( gps.location.age() < 1000 )   
    {
      Serial.println("Data acquired");
      break;                        // and gets out of the function if the data is valid
    }
  }
}

// ------------------------------------------------------------------------------------------------
void Init_Motion()                             // It initializes the MPU6050 as a motion sensor
{
  if (!mpu.begin()) {                                 // Tries to initiallize the sensor. If it can't:
    Serial.println("Failed to find MPU6050 chip. Check wiring and restart");    // Shows the error on screen
    while (1)       delay(10);                        // And gets stuck here forever, requiring a restart
  }
  Serial.println("MPU6050 Found!");                   // If the begin() call was successful, we continue.

  // Now we configure the motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);    // Options for the filter: 5, 2.5, 1.25, 0.63, Unused or Hold.
  mpu.setMotionDetectionThreshold(1);                 // Threshold for the movement. 1 seems to work just fine. 2 does not detect slow moves.
  mpu.setMotionDetectionDuration(20);                 // Duration of the detection. Must be an integer. 20 works just fine.
  mpu.setMotionInterrupt(true);                       // enables the motion interrupt, based on Threshold and Duration
}

// ------------------------------------------------------------------------------------------------
void Detect_Motion()   // Only used when we need to see the data on screen. Otherwise we use just use "if( mpu.getMotionInterruptStatus() )"
{
  if(mpu.getMotionInterruptStatus()) 
  {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("AccelX: ");    Serial.print(a.acceleration.x);   Serial.print(", ");
    Serial.print("AccelY: ");    Serial.print(a.acceleration.y);   Serial.print(", ");
    Serial.print("AccelZ: ");    Serial.print(a.acceleration.z);   Serial.print(",   ");
    Serial.print("GyroX: ");     Serial.print(g.gyro.x);           Serial.print(", ");
    Serial.print("GyroY: ");     Serial.print(g.gyro.y);           Serial.print(", ");
    Serial.print("GyroZ: ");     Serial.print(g.gyro.z);           Serial.println();
  }

  delay(10);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//        SETUP FUNCTION - will be run only once, at the beginning                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(9600);               // We begin serial communication with the screen, with a baudrate of 9600 baud
  Wire.begin();                     // We start the Wire communication with the accelerometer
  Init_Motion();

  ss.begin(GPSBaud);                // We start communication with the GPS module using SoftSerial
  deviceState = DEVICE_STATE_INIT;  // We establish the initial state of the Cubecell
  LoRaWAN.ifskipjoin();             // Uses the stored OTAA credentials to join session
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//        LOOP FUNCTION - will be run repeatedly after setup                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  LoRaWAN_Loop();
}