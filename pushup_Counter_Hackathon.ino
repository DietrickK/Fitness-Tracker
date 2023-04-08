//William Worthy, Sky Jared Santos Butlay
//WSU Everett Hackathon Push-Up Counter
//Program based on: PUSH UP COUNTER BY: Kyle Glerum
//Team Exofit: Dietrick Kooyman, William Worthy, Sky Butlay

#include <LiquidCrystal.h> //LCD library

// initialize the library by associating any needed LCD interface pin with the arduino pin number it is connected to
const int trigPin = 8; //set  pins for the ultrasonic sensor, button and buzzer
const int echoPin = 9;
const int rstPin = 6;
const int recordBroken = 13;
const int buzzRecord = 10;
const int notRecBroken = 7;

//define variables for later use
long duration; 
int  distance;
int pushNum;
int btnState;
int pushRecord;
bool buzzBool; 
int clearCnt;

//initialze lcd pins on arduino
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {
  
  // put your setup code here, to run once:
  lcd.begin(16, 2); //sets dimentions of screen (16 chars x 2 rows)
  lcd.clear();

  //sets pin modes for all pins
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(rstPin, INPUT_PULLUP);
  pinMode(recordBroken, OUTPUT);
  pinMode(buzzRecord, OUTPUT);
  pinMode(notRecBroken, OUTPUT);

  //variable initializations
  duration = 0;
  distance = 0;
  pushNum = 0;
  btnState = 0;
  pushRecord = 0;
  clearCnt = 0;
  buzzBool = true;

  digitalWrite(notRecBroken, HIGH); //turns ON red LED

}


void loop() {
  // put your main code here, to run repeatedly:
  
  distance = get_Sensor_Data(trigPin, echoPin); //reads data from sonar sensor  
      
  
  
  btnState = digitalRead(rstPin); //polls reset button 

  clearCnt = 0;
  if (btnState == LOW) {//if reset button is pressed, clear pushup count 

    if(pushNum > pushRecord) {//set new record
      pushRecord = pushNum;
      buzzBool = true;
    }
    
    pushNum = 0; //resets number of pushups
    lcd_Out(pushNum, btnState, pushRecord); //prints text to LCD
    digitalWrite(recordBroken,LOW); //LED turns OFF when reset button is pressed  


    digitalWrite(notRecBroken, HIGH); //turns ON red LED

    
    //counts how long user has held down the reset button
    while((btnState == LOW) && (clearCnt <= 3)){
      btnState = digitalRead(rstPin); //polls reset button
      clearCnt++;
      delay(700);
    }

    if(clearCnt >= 3){//if reset button has been held down for a certain length of time, reset current pushup record
      pushRecord = 0;
    }

    lcd_Out(pushNum, btnState, pushRecord); //prints text to LCD


     //----
   
  }//end reset if



  if (distance <= 10) {//if user does a pushup, record it 
    
    pushNum ++;
    
  }

  lcd_Out(pushNum, btnState, pushRecord); //prints text to LCD

    if((pushNum > pushRecord) && buzzBool && !(pushRecord == 0)) {//green LED lights when record is broken
    digitalWrite(recordBroken,HIGH);//turns ON green LED
    digitalWrite(notRecBroken,LOW);//turns OFF green LED
     
        tone(buzzRecord, 1000, 1000); // Sends 1KHz sound signal
        buzzBool = false; 
    
    }
  
   while (distance <= 10)  { //if the distance stays smaller then ten for a while, this piece of code makes sure that only one point is given for one pushup
     
    distance = get_Sensor_Data(trigPin, echoPin);//gets distance reading 
  
  }//end while


  }//end loop


  //Function name: get_Sensor_Data
  //Purpose: to read distance data from sonar, convert it to a distance measure in cm
  //parameters: sensor ping, sensor echo
  //returns: distance reading
  int get_Sensor_Data(const int trig, const int echo) {

      int dur = 0;
      int dist = 0;
      //reads distance data
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      dur = pulseIn(echo, HIGH);
      dist = dur * 0.034/2;
      delay(100);

      return dist;

  }//end get_Sensor_Data



  //Function name: lcd_Out
  //Purpose: to print relevent text to LCD display 
  //parameters: number of pushups, reset button, current pushup record
  //returns: none
  void lcd_Out(int pushNum, int btnSt, int rec) {

    if(btnSt == LOW) {
      lcd.clear();
    }

    lcd.print("Push-Ups: ");
    lcd.setCursor(10, 0); // set cursor to the 10th char of 1st row
    lcd.print(pushNum);
    lcd.setCursor(0, 1); // set cursor to second row
    lcd.print("Record: ");
    lcd.setCursor(9, 1); // set cursor to ninth char on second row
    lcd.print(rec);    
    lcd.setCursor(0, 0); // set cursor to first row

  }//end lcd_Print

 