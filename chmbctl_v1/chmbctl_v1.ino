//User definable parameters
  //program control
  //0 = Tsoil control
  //1 = vpd control
  //2 = Tair control
  int hotbox_ctl = 1;
  float vpdSet = 1.0; //(kPa)
  int tempSet = 0; // although this can be changed initially, i typically leave it blank because the code will change this once is reads the current air temperature of the chamber
  int tsoilSet = 0;


//wiring
  //3.3V - I2C hub, 
  //GND
  //AO
  //A1
  //A2
  //A3
  //SCK - Thermocouples SCK
  //MOSI - Thermocouples MO
  //MISO - thermocouples
  //D0 - Thermocouple1 signal
  //D1 - Thermocouple2 signal
  //SDA - SDA of I2C hub (green)
  //SCL - SCL of I2C hub (white)
  //D5 - relay 4
  //D6 - relay 3
  //D7 - relay 2
  //D8 - relay 1
  //VBUS - +5V to thermistors,

//Import Libaries
  #include <Arduino.h>
  #include <Wire.h>
  #include <SPI.h>
  #include "Adafruit_SHT31.h"
  #include <Adafruit_SSD1306.h>
  #include <RTClib.h>
  #include <Adafruit_I2CDevice.h>
  #include <Adafruit_I2CRegister.h>

//global control variables


  //VPD control parameters
  bool vpdRun = true;
  float vpdInc = 0.1; //VPD increase (kPa)
  int vpdTimeInc = 100; //VPD increase time interval (min)
  int vpdTimeMin = 60; //the number of minutes VPD is held for (approximatly)
  float vpdMin = 5; // max VPD (kPa)

  //Tair control parameters
  int TempInc = 1; //temperature increase (Celsius)
  int TimeInc = 1; //time interval (minutes) for temperature increases
  int TairMax = 25; // max temperature (Celsius)
  int TsoilMax = 65;
  int TempTimeMax = 240; //the number of minutes tempearture is held (minutes)
  //vpd variables

  bool run = true;

//time interval variables
  unsigned long previousMillis = 0; // For storing previous timestep
  int counter = 1;
  int INTERVAL = 5000;        // speed the sketch runs (ms)
  float TimeRm;
  int TiTotalRm;
  float MaxTiRm;
  char* units[] = {"sec"};
  float TiRm;


  //temperarture control variables
  float Td;
  int Td_last = 0;
  float Tref = 20; //reference temperature (close to Tair) used to refine T PID
  float Tdmin;
  float Tdmax;

  //Temperature measurement variables
  float Tair;
  float tsoil1;
  float tsoil2;

//vpd PID control variables
  float vpdd = 10;
  byte flagVpd = 0;

  //vpd measurement variables
  float rh;
  float vpd = 0;
  float vpd_sat;
  float const_a = 0.611;
  float const_b = 17.502;
  float const_c = 240.97;

//for SHT31
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  bool enableHeater = false;

//for SD module
  #include <SD.h>
  File myFile;
  #define FILE_NAME "VPDBOXv1.csv"
  byte CS_pin = 4;

//for RTC
  //RTC_DS3231 rtc;
     
  //for SSD1306 I2C OLED Display
  #define SCREEN_WIDTH 128 //lower if RAM malfunctions
  #define SCREEN_HEIGHT 32
  #define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Pin assignments and flag creation
  byte heatPin1 = 13;
  byte pumpPin = 11;
  byte heatswitch = 0;
  byte humid = 0;
  int count = 0;

//variables for serial user input
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
String strRecv;



void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(10);  

  Serial.println("Beginning Setup Routine");

  //OLED setup
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
        //set additional display parameters
      display.setTextSize(0.5);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.clearDisplay(); display.setCursor(0, 0);display.write("loading program");display.display();
      delay(2000);


  // if (! rtc.begin()) {
  //   Serial.println("Couldn't find RTC");
  //   display.clearDisplay(); display.setCursor(0, 0);display.write("no RTC");display.display();
  //   delay(1000);
  //   Serial.flush();
  //   while (1) delay(10);
  // }

  // if (rtc.lostPower()) {
  //   Serial.println("RTC lost power, let's set the time!");
  //   // When time needs to be set on a new device, or after a power loss, the
  //   // following line sets the RTC to the date & time this sketch was compiled
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //   // This line sets the RTC with an explicit date & time, for example to set
  //   // January 21, 2014 at 3am you would call:
  //   // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  // }


  //SHT31 test
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
      Serial.println(F("Couldn't find SHT31"));
      display.clearDisplay(); display.setCursor(0, 0);display.write("no SHT31");display.display();
      delay(1000);
    }

//SD test
    // Serial.print("\nInitializing SD card...");
    // if (!SD.begin(CS_pin)) {
    //   Serial.println(F("SD card initialization failed!"));
    //   display.clearDisplay(); display.setCursor(0, 0);display.write("no SD card");display.display();
    //   delay(1000);
    // }else{
    //   Serial.println(F("card initialized"));
    //   display.clearDisplay(); display.setCursor(0, 0);display.write("card init");display.display();
    //   delay(1000);
    // }

    // myFile = SD.open(FILE_NAME, FILE_WRITE);
    // myFile.print("TairMax, TsoilMax, TimeIncrement, TempTimeMax, VPDmax, Time, tempSet, Tleaf_1, Tleaf_2, Tsoil_1, Tsoil_2, Tair, vpdSet, VPDair"); myFile.println(""); myFile.close();


  //pin assignments
    pinMode(heatPin1, OUTPUT);
    digitalWrite(heatPin1, HIGH);
    pinMode(pumpPin, OUTPUT);
    digitalWrite(pumpPin, HIGH);
    delay(10);

  Serial.println("Done with setup");
}



//additional functions
void Tpid(byte heatPin1){
  int Td_last = 0;
  float Td = Tair - tempSet;
  float Tdmin = -1*(1-(tempSet - Tref)/45);
  float Tdmax = 0.2;

  if (Td < Tdmin * 4){
    digitalWrite(heatPin1, LOW);
    //digitalWrite(heatPin2, LOW);
  }
  else if(Td > Tdmin*4 & Td < Tdmin){
    digitalWrite(heatPin1, LOW);
    //digitalWrite(heatPin2, LOW);
  }
  else if(Td > Tdmin and Td < Tdmax){
    digitalWrite(heatPin1, LOW);
    //digitalWrite(heatPin2, LOW);
  }
  else if(Td > Tdmax){
    digitalWrite(heatPin1, HIGH);
    //digitalWrite(heatPin2, HIGH);
  }

  Td_last <- Td_last - Td;
}


void VPDpid(int pumpPin){
    float vpdd = vpdSet - vpd;
    set_vpdd(vpdd);
  
  if (vpdd < -0.2){
    digitalWrite(pumpPin, LOW);
    //Serial.println(F("Humidity, ON, Humidity Control 5 (sec)"));
  }
  else if (vpdd > -0.2  & vpdd < 0){
    digitalWrite(pumpPin, LOW);
    //Serial.println(F("Humidity, ON, Humidity Control 1 (sec)."));
  }
  else if(vpdd >= 0){
    digitalWrite(pumpPin, HIGH);
    //Serial.println(F("Humidity, OFF, Humidity Control 1 (sec)."));
  }

}

//fundtion to read 10kOhm thermistors
float Tread(byte xpin){
      float Traw = analogRead(xpin);
      float logR2 = log(10000 * (1023.0 / Traw - 1.0));
      float T = ((1.0 / (1.009249522e-03 + 2.378405444e-04*logR2 + 2.019202697e-07*logR2*logR2*logR2))) - 273.15;
      return T;
}


//setter functions
  int set_tempSet(int x){tempSet = x; return tempSet;}
  int set_tsoilSet(int x){tsoilSet = x; return tempSet;}
  float set_Tair(float x){Tair = x; return Tair;}
  float set_vpd(float x){vpd = x; return vpd;}
  float set_vpdd(float x){vpdd = x; return vpdd;}
  float set_vpdSet(float x){vpdSet = x; return vpdSet;}
  // float set_tsoil1(float x){tsoil1 = x; return tsoil1;}
  // float set_tsoil2(float x){tsoil2 = x; return tsoil2;}
  bool set_run(bool x){run = x; return run;}
  bool set_vpdRun(bool x){vpdRun = x; return vpdRun;}
  int set_counter(int x){counter = x; return counter;}


void TsoilCTL(){
//set initial tempSet

  if (tsoilSet == 0){ // set at 0 initially
    tsoilSet = Tair+1; // set the temperature to 1 degree above the integer of room temperature
  }

  if (tsoilSet < TairMax){

    //Changes tempSet and updates time arrays if conditions are met
    if(counter >= (TimeInc*60)){ 
      counter = 1;
      tsoilSet += TempInc;
      Serial.println(); Serial.println();Serial.println();
      Serial.print("New tempSet is:  ");Serial.println(tsoilSet);
      Serial.println(); Serial.println();Serial.println();
    }
    //determine upper/lower limits of temperature control
    float Tdmin = -1*(1-(tsoilSet - Tref)/45);
    float Tdmax = 0.2;

    VPDpid(pumpPin);
    Tpid(heatPin1);
    TimeRm = TimeInc*60 - counter;
    TiTotalRm = (TairMax - tsoilSet) * (TimeInc*60) + (TempTimeMax*60);

  }else if(tsoilSet == TairMax){
      
      float tsoilmin = min(tsoil1,tsoil2);
      VPDpid(pumpPin);
      Tpid(heatPin1);
      TimeRm = TimeInc*60 - counter;
      TiTotalRm = (TairMax - tsoilSet) * (TimeInc*60) + (TempTimeMax*60);
      if(tsoilmin >= TsoilMax){
        tempSet += TempInc;
      }

  }else if (tsoilSet > TairMax){
      digitalWrite(heatPin1, HIGH);
      digitalWrite(pumpPin,HIGH);
      TimeRm = 0;
      run = false;
      tempSet = 0;
  }
  set_tsoilSet(tsoilSet);
  set_run(run);
}

void TairCTL(){

 if (tempSet == 0){ // set at 0 initially
    tempSet = Tair+1; // set the temperature to 1 degree above the integer of room temperature
  }

  if (tempSet < TairMax){

    //Changes tempSet and updates time arrays if conditions are met
    if(counter >= (TimeInc*60)){ 
      counter = 1;
      tempSet += TempInc;
      Serial.println(); Serial.println();Serial.println();
      Serial.print("New tempSet is:  ");Serial.println(tempSet);
      Serial.println(); Serial.println();Serial.println();
    }
    //determine upper/lower limits of temperature control
    float Tdmin = -1*(1-(tempSet - Tref)/45);
    float Tdmax = 0.2;

    //Tpid(heatPin1);
    TimeRm = TimeInc*60 - counter;
    TiTotalRm = (TairMax - tempSet) * (TimeInc*60) + (TempTimeMax*60);
  }else if(tempSet == TairMax){
      vpdRun = true;
      
      Tpid(heatPin1);
  }
       //determine upper/lower limits of temperature control
      float Tdmin = -1*(1-(tempSet - Tref)/45);
      float Tdmax = 0.2;
 Tpid(heatPin1);
      TimeRm = vpdTimeInc*60 - counter;
      TiTotalRm = (vpdMin - vpdSet) * (vpdTimeInc*60) + (vpdTimeMin*60);

}


void vpdCTL(){
  Serial.println("running VPDctl");

//set initial tempSet

  if (vpdRun == true){
    
    if (vpdSet == 0){ // set at 0 initially
      vpdSet = round(vpd) + vpdInc; // set the vpd to 0.1 degree above the integer of room temperature
    }

    //if (vpdSet >= vpdMin){

      //Changes tempSet and updates time arrays if conditions are met
      if(counter >= (vpdTimeInc*60)){ 
        counter = 1;
        vpdSet = vpdSet - vpdInc;
        Serial.println(); Serial.println();Serial.println();
        Serial.print("New vpdSet is:  ");Serial.println(vpdSet);
        Serial.println(); Serial.println();Serial.println();
      }
 
      VPDpid(pumpPin);
     

    //}else if (vpdSet < vpdMin){
    //     digitalWrite(pumpPin,HIGH);
    //     TimeRm = 0;
    //     run = false;
    //     vpdSet = 0;
    //     tempSet = 0;
    // }
    
    set_vpdSet(vpdSet);
    set_tempSet(tempSet);
    set_run(run);
    set_vpdRun (vpdRun);
  }
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
 
 // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
    receivedChars[ndx] = rc;
    ndx++;
    if (ndx >= numChars) {
    ndx = numChars - 1;
    }
    }
    else {
    receivedChars[ndx] = '\0'; // terminate the string
    ndx = 0;
    newData = true;
    }
  }
}

void showNewData() {
 if (newData == true) {
  strRecv = String(receivedChars);
  String firstFive = strRecv.substring(0,5);
  Serial.print("This just in ... ");
  Serial.println(receivedChars);

  if(firstFive == "vpset"){
    Serial.print("new vpd set point = ");
    Serial.println(strRecv.substring(6,strRecv.length()));
    float strFloat = strRecv.toFloat();
    set_vpdSet(strFloat);
  }else if(firstFive == "Taset"){
    Serial.print("new Tair setting = ");
    Serial.println(strRecv.substring(6,strRecv.length()));
    float strFloat = strRecv.toFloat();
    set_tempSet(strFloat);
  }else if(firstFive == "Tsset"){
    Serial.print("new Tsoil setting = ");
    Serial.println(strRecv.substring(6,strRecv.length()));
    float strFloat = strRecv.toFloat();
    set_tsoilSet(strFloat);
  }else{
    Serial.println("serial input incorrect");
  }

  newData = false;

 }
}


void loop() {
  //read time and check if it is time to run the code

    //check for serial user inptut
    recvWithEndMarker();
    showNewData();

  unsigned long currentMillis = millis(); 

  if ((currentMillis - previousMillis) >= INTERVAL) {
      // ..If yes, save current time.  Then update the LED pin and LED state.
    previousMillis = currentMillis;  //reset previous time for interval
    

    
    //get data from SHT31 - air T and RH
    Tair = sht31.readTemperature();
    rh = sht31.readHumidity();//read humidity from sht31
    vpd_sat = 0.611*exp((17.502*Tair)/(Tair+240.97));
    vpd = vpd_sat - (vpd_sat*(rh/100));
    
    //read leaf temperature
    // thermocouple1.readInternal();
    // float tleaf1 = thermocouple1.readCelsius();
    // delay(10);
    // thermocouple2.readInternal();
    // float tleaf2 = thermocouple2.readCelsius();
    // delay(10);
    
    //read soil temperature
    // tsoil1 = Tread(tsoil1_pin);
    // tsoil2 = Tread(tsoil2_pin);

    //update global variables
    set_Tair(Tair);
    set_vpd(vpd);
    set_tempSet(tempSet);
    // set_tsoil1(tsoil1);
    // set_tsoil2(tsoil2);
      
    //run the main loop 
    if(run == true){

      switch(hotbox_ctl){
        case 0:
          TsoilCTL();
          break;
        case 1:
          vpdCTL();
          break;
        case 2:
          TairCTL();
          break;
      }
    }
    
    counter = counter + 1;
    set_counter(counter);

    //print data to serial monitor
    Serial.print(F("tempSet:"));Serial.println(tempSet);
    Serial.print(F("T_air: ")); Serial.print(Tair);Serial.print(F("   RH: ")); Serial.println(rh);
    Serial.print("vpdd:  ");Serial.println(vpdd);
    //Serial.print(F("Tleaf1: ")); Serial.print(tleaf1);Serial.print(F("   Tleaf2:")); Serial.println(tleaf2);
    //Serial.print(F("Tsoil1: ")); Serial.print(tsoil1);Serial.print(F("   Tsoil2:")); Serial.println(tsoil2);
    Serial.print("Heater: ");Serial.println(heatswitch);
    Serial.print("counter:  ");Serial.print(counter);Serial.print("  nexttime:   ");Serial.println(TimeRm);

    //get state of heatpin to display
    if(digitalRead(heatPin1) == HIGH){
      heatswitch = 0;
    }else{
      heatswitch = 1;
    }

        //get state of heatpin to display
    if(digitalRead(pumpPin) == HIGH){
      humid = 0;
    }else{
      humid = 1;
    }


    if(TiTotalRm >60){
      TiTotalRm = TiTotalRm/60;
      units[0] = "m";
      units[1] = "i";
      units[2] = "n";
    }

  

    display.clearDisplay(); display.setCursor(0, 0);
    //display.write("T:   ");display.print(Tair);display.println();
    //display.write("T/tempSet:   ");display.print(Tair); display.write("| "); display.print(tempSet); display.println();
    display.write("vpd/vset:  "); display.print(vpd);display.write("/ "); display.print(vpdSet); display.println();
    //display.write("Tsoil 1/2: "); display.print(tsoil1,1);display.write("| "); display.print(tsoil2,1); display.println();
    display.write("T/RH: ");display.print(Tair);display.write("/ ");display.print(rh);display.println();
    display.write("heater: ");display.print(heatswitch);display.write("  hum: ");display.print(humid); display.println();
    display.write("TimeRm:");display.println(TimeRm,0);
    display.write("TiTotalRm:");display.print(TiTotalRm);display.write(" ");
    //display.write(units[0]);display.write(units[1]);display.write(units[2])
    display.display();
  
    // myFile = SD.open(FILE_NAME, FILE_WRITE);
    // myFile.print(float(TairMax));myFile.print(",");myFile.print(float(TsoilMax));myFile.print(",");myFile.print(float(TimeInc));myFile.print(",");myFile.print(float(TempTimeMax));myFile.print(",");myFile.print(float(vpdMin)); myFile.print(",");myFile.print(float(currentMillis)); myFile.print(",");myFile.print(float(tempSet)); myFile.print(",");myFile.print(tleaf1); myFile.print(",");myFile.print(tleaf2); myFile.print(",");myFile.print(tsoil1); myFile.print(",");myFile.print(tsoil2);
    // myFile.print(",");myFile.print(Tair); myFile.print(",");myFile.print(vpdSet); myFile.print(","); myFile.print(vpd); myFile.println("");myFile.close();
      
  }
}


