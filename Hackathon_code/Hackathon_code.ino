// Hackathon code for running tracking

//**********************************************************************Librarys*************************************************************************

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "BMP085.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//**********************************************************************Constants*************************************************************************

// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7); // bmp180 pins
Adafruit_GPS GPS(&mySerial); // GPS object
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // liquid crystal pinout
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // liquid crystal callout
#define DHTPIN 10     // Digital pin connected to the DHT sensor
const int startstop = 13; 

//**********************************************************************Configurations*************************************************************************

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE); // for humidity sensor

//**********************************************************************Veriables*************************************************************************

float distance = 0;
short elevation_gain = 0;
short elevation_loss = 0;
short elevation = 0;
short elevation_previous = 0;
float speed_val = 0;
float temperature;
float pressure;
double timercounterstart = 0;
double timercounterstop = 0;
double timetotal = 0;
double time_elapsed = 0;
int user_weight = 77; // units in kg
int tiltsensor = 1; // tilt sensor pin
int sensorval;// tilt sensor value
int prevSensVal;// old tilt sensor value
int counter, state = 0, btnVal;// step counter, stop watch status, value of button
int running_MET = 1.65;
int calories = 0;

BMP085 myBarometer; // sructure for the bmp180 ( operates the same as the bmp085 )

//**********************************************************************Function Prototypes*************************************************************************

//*************************Set up prototypes*********************
void Initialize_sensors(void);
void Start_Com(void);
void GPS_Config(void);
void GPS_Aquire(void);

//*************************Running program prototypes*********************
void stepcount(void);
void Start_Stop_Timer(void);
void Display_GPS_Data(void);
void Display_Weather_Data(void);

//**********************************************************************Board Setup*************************************************************************

void setup()
{

  pinMode(startstop, INPUT);      // set pin 13 as an input for the start stop timer
  pinMode(tiltsensor, INPUT);     // setting the tilt sensor to an output
  sensor_t sensor;                // humidity sensor type

  Initialize_sensors();           // warm up sensors
  Start_Com();                    // begin communication protocalls
  GPS_Config();                   // set up configuration for GPS
  GPS_Aquire();                   // start GPS collection for initial positioning
}

//**********************************************************************Timing Initialization************************************************************

uint32_t timer = millis();
uint32_t timer2 = millis();

//**********************************************************************Main Loop*************************************************************************

void loop()                      // run over and over again
{

    stepcount();                 // check the step count state to see if the step count changed
    Start_Stop_Timer();          // Aquire the start and stop time of the running clock then calculate data for readback
    Display_GPS_Data();          // Display the GPS data to the user for readback
    Display_Weather_Data();      // Display current weather data
     
}

//**********************************************************************Functions*************************************************************************

//***************************************Setup Functions*************************************************

void stepcount(void){
  // reading the status of the tiltsensor
  sensorval = digitalRead(tiltsensor);
  
  // comparing the previous value of the tilt sensor with the new value to increment the stepcounter
  if(sensorval != prevSensVal){
    counter++;// incrementing the step counter if the tilt sensor has changed
  }
  
  // updating the previous value of sensor with the most recent value
  prevSensVal = sensorval;
  
 // Serial.println(counter); // print to serial port for debug output perpouses
}

void Initialize_sensors(void){
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  myBarometer.init(); // initialize the barometric pressure sensor
  
  dht.begin(); // start the humidity sensor
  
  delay(5000); // delay 5 seconds for warmup
}

void Start_Com(void){
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  //Serial.begin(115200); // uncomment for serial communication
  // GPS communication speed
  GPS.begin(9600);
}

void GPS_Config(void){
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
}

void GPS_Aquire(void){
  lcd.print(F("Waiting for Sat")); // display wait
    lcd.setCursor(0,1); // home curser
    lcd.print(F("connection...")); // display wait

    while(!GPS.fix){ // loop as long as there is no GPS data

      char c = GPS.read();
      // if a sentence is received, we can check the checksum, parse it...
        if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          return;  // we can fail to parse a sentence in which case we should just wait for another
        }
       elevation_previous= GPS.altitude; // get initial elevation at the start of the run
    }

    lcd.setCursor(0,0); // home curser
    lcd.clear(); // clear screen
    lcd.print(F("Connection Made!"));
}

//*****************************************Running Functions*********************************************

void Start_Stop_Timer(void){
  if (digitalRead(startstop) == HIGH){ // check if the start stop button is pressed
    
      while(digitalRead(startstop) == HIGH){ // while the  button is held
        delay(10); // endlessly loop untill the button is released
      }

      if (timercounterstart == 0){ // if the start counter is notactivated
        timercounterstart = millis(); // aquire the start time
        timercounterstop = 0; // set the stop time to 0 for new data aquisition
      }
      
      else{
        timercounterstop = millis(); // aquire the stop time
        timetotal = timercounterstop - timercounterstart; // calculate time in exeresize
        timercounterstart = 0; // reset start timer
        time_elapsed = ((timetotal/1000)); // convert time to seconds
         
        lcd.setCursor(0,0); // home curser
        lcd.clear(); // clear screen

        lcd.print(F("Tm:")); // display distance on the lcd
        lcd.print(time_elapsed); // display value
        lcd.print(F(" DST:")); // display distance on the lcd
        lcd.print(distance,2); // display value

        lcd.setCursor(0,1); // home curser

        lcd.print(F("STP:")); // display Step on the lcd
        lcd.print(counter); // display value

        calories = (0.0175*running_MET*user_weight)*(distance);

        lcd.print(F(" CAL:")); // display Step on the lcd
        lcd.print(calories); // display value
        
      }
    }
}

void Display_GPS_Data(void){
  // approximately every 2 seconds or so, print out the current stats
  if ((millis() - timer > 2000) && (timercounterstart != 0) ) { // only run function if the timer has started and if 2 seconds have elapsed
    timer = millis(); // reset the timer for next itteration

      char c = GPS.read(); // read the data from the GPS register
      delay(10); // delay to allow CPU to catch up
      
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
   }

    if (GPS.fix) { // if we have GPS data, display it, otherwise move along

      lcd.setCursor(0,0); // home curser
      lcd.clear(); // clear screen

      //Serial.print(F("Spd: ")); // send speed via serial debug
      speed_val = GPS.speed; // record speed 
      //Serial.println(speed,1); // 1.6878 ft/sec
      lcd.print(F("SPD: ")); // display speed on the lcd
      lcd.print(speed_val,1); // display value

      // Run calculation for the distance traveled
      distance = distance + ((speed_val * 1.6878)/5280)*2; // distance to be recorded in miles
      //Serial.print(F("Dst: ")); // display distance
      //Serial.println(distance,2); // display distance

      lcd.print(F("DST:")); // display distance on the lcd
      lcd.print(distance,2); // display value


      elevation = GPS.altitude; // display altitude
      //Serial.print(F("Elv: ")); // print elevation
      //Serial.println(elevation,1);

      // Record the increase or decrease in elevation over the run
      if (elevation > elevation_previous){ // calculate elevation gain
        elevation_gain = elevation_gain + (elevation - elevation_previous); // increase count of gain
      }
      else if (elevation < elevation_previous){ // calculate elevation loss
        elevation_loss = elevation_loss + (elevation_previous - elevation); // increase elevation loss
      }
      else{
          // no elevation change
      }


      // display the elevation data tothe user on the LCD
          lcd.setCursor(0,1); // go to second row
          lcd.print(F("ELV:")); // display elevation on the lcd
          lcd.print(elevation,1); // display value
          lcd.print(F(" ELG:")); // display elevation on the lcd
          lcd.print(elevation_gain,1); //display elevation gain
       elevation_previous = elevation; // update previous elevation 

    }
  }
}

void Display_Weather_Data(void){
  
   if ((millis() - timer2 > 8000) && (timercounterstart != 0) ) { // only run function if the timer has started and if the 8 seconds have passed
      timer2 = millis();                              // reset the timer

      sensors_event_t event;                          // set up a sensor type
     
      lcd.setCursor(0,0);                             // home curser
      lcd.clear();                                    // clear screen

      temperature = myBarometer.bmp085GetTemperature( // aquire temperature from the bmp180
                      myBarometer.bmp085ReadUT());    //Get the temperature, bmp085ReadUT MUST be called first
      pressure = myBarometer.bmp085GetPressure(myBarometer.bmp085ReadUP());//Get the temperature
      
      //Serial.print(F("Temp: "));                    // print temperature
      //Serial.println(temperature,1);
      lcd.print(F("Temp:"));                          // display temperature on the lcd
      lcd.print(temperature,1);                       // display value

      //dht.temperature().getEvent(&event);
      dht.humidity().getEvent(&event);
      //Serial.print(F("Hum: "));                     // display humidity
      //Serial.print(event.relative_humidity,1);
      //Serial.println(F("%"));

      lcd.print(F(" HUM:"));
      lcd.print(event.relative_humidity,1);           // print humidity data

      lcd.setCursor(0,1);                             // go to second row

      //Serial.print(F("Prs: "));                     // print pressure
      //Serial.println(pressure,1);
      lcd.print(F("PRS:"));                           // display pressure on the lcd
      lcd.print(pressure,1);                          // display value
   }
}

  
