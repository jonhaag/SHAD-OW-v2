// =======================================================================================
//                 SHADOW :  Small Handheld Arduino Droid Operating Wand
// =======================================================================================
//                          Last Revised Date: 4/15/19
//                             Written By: KnightShade
//                        Inspired by the PADAWAN by danf
//                      Bug Fixes from BlackSnake and vint43
//	       Contributions for PWM Motor Controllers by JoyMonkey/Paul Murphy
//                            With credit to Brad/BHD
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it .
//         This program is distributed in the hope that it will be useful,
//         but WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// =======================================================================================
//   Note: You will need a Arduino Mega 1280/2560 to run this sketch,
//   as a normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH
//
//   This is written to be a UNIVERSAL Sketch - supporting multiple controller options
//      - Single PS3 Move Navigation
//      - Pair of PS3 Move Navigation
//      - Android Phone (Limited Controls)
//      Future Planned Enhancements:
//      - XBox 360 Controller  (Why not, these also uses the USB Host Shield)
//      - PS3 Dual Shock Controller
//      - PS4 Dual Shock Controller
//
//   PS3 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//   For more information visit my blog: http://blog.tkjelectronics.dk/ or
//
//   Holoprojector Support:
//      Legacy Holoprojector Support:  was based on Padawan, using a second Arduino (Teeces)
//          This used a Padawan Dome sketch that was loaded to the Teeces Logics.
//          It leveraged the EasyTransfer libraries by Bill Porter
//          Legacy support will likely be deprecated and removed in time
//      Long Term Holoprojector Support:
//          SHADOW control will be isolated from particular logic hardware.
//          We will migrate to I2C dome commands with PWM support:
//          Holoprojector Servos and LEDs will be driven by:  
//            http://www.adafruit.com/product/815
//            This can drive 6 servos, and 3 LEDs.  PWM will allow for LED brightness "flicker"
//  
//   Sabertooth (Foot Drive):
//         Set Sabertooth 2x32 or 2x25 Dip Switches: 1 and 2 Down, All Others Up
//
//   SyRen 10 Dome Drive:
//         For SyRen packetized Serial Set Switches: 1, 2 and 4 Down, All Others Up
//         NOTE:  Support for SyRen Simple Serial has been removed, due to problems.
//         Please contact DimensionEngineering to get an RMA to flash your firmware
//         Some place a 10K ohm resistor between S1 & GND on the SyRen 10 itself
//
// =======================================================================================
//
// ---------------------------------------------------------------------------------------
//                          User Settings
// ---------------------------------------------------------------------------------------

//Primary Controller bound to Gmyle Class 1 Adapter 
//String PS3MoveNavigatonPrimaryMAC = "04:76:6E:87:B0:F5"; //If using multiple controlers, designate a primary

//Primary Controller bound to Parani UD-100 
String PS3MoveNavigatonPrimaryMAC = "00:07:04:05:EA:DF"; //If using multiple controlers, designate a primary

byte drivespeed1 = 25;   //set these 2 to whatever speeds work for you. 0-stop, 127-full speed.
byte drivespeed2 = 65;  //Recommend beginner: 50 to 75, experienced: 100 to 127, I like 100.

byte ramping = 6; //3;        // Ramping- the lower this number the longer R2 will take to speedup or slow down,
                         // change this by increments of 1
int footDriveSpeed = 0;  //This was moved to be global to support better ramping of NPC Motors

byte joystickFootDeadZoneRange = 15;  // For controllers that centering problems, use the lowest number with no drift
byte joystickDomeDeadZoneRange = 10;  // For controllers that centering problems, use the lowest number with no drift
byte driveDeadBandRange = 10;     // Used to set the Sabertooth DeadZone for foot motors


//#define TEST_CONROLLER   //Support coming soon
//#define SHADOW_DEBUG       //uncomment this for console DEBUG output
//#define SHADOW_VERBOSE     //uncomment this for console VERBOSE output
//#define BLUETOOTH_SERIAL     //uncomment this for console output via bluetooth.  
// NOTE:  BLUETOOTH_SERIAL is suspected of adding CPU load in high traffic areas

// R/C Mode settings...
#define leftFootPin 44    //connect this pin to motor controller for left foot (R/C mode)
#define rightFootPin 45   //connect this pin to motor controller for right foot (R/C mode)
#define leftDirection 1   //change this if your motor is spinning the wrong way
#define rightDirection 0  //change this if your motor is spinning the wrong way  






// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <SPP.h>
#include <usbhub.h>
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <Servo.h>
#include <Wire.h>


// ---------------------------------------------------------------------------------------
//                          Variables
// ---------------------------------------------------------------------------------------

long previousDomeMillis = millis();
long previousFootMillis = millis();
long currentMillis = millis();
int serialLatency = 25;   //This is a delay factor in ms to prevent queueing of the Serial data.
                          //25ms seems appropriate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation

///////Setup for USB and Bluetooth Devices////////////////////////////
USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS3BT *PS3Nav=new PS3BT(&Btd);
PS3BT *PS3Nav2=new PS3BT(&Btd);
//Used for PS3 Fault Detection
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;

#ifdef BLUETOOTH_SERIAL
SPP SerialBT(&Btd,"Astromech:R2","1977"); // Create a BT Serial device(defaults: "Arduino" and the pin to "0000" if not set)
boolean firstMessage = true;
#endif
String output = "";

boolean isFootMotorStopped = true;
boolean isDomeMotorStopped = true;

boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

byte vol = 0; // 0 = full volume, 255 off
boolean isStickEnabled = true;
unsigned long automateMillis = 0;

unsigned long DriveMillis = 0;

Servo leftFootSignal;
Servo rightFootSignal;

int dir1pin =13; //Motor Direction pin (goes to DIR1)
int spe1pin =12; //Motor Speed pin (goes to PWM1)
int dir2pin =11; //Motor Direction pin (goes to DIR2)
int spe2pin =10; //Motor Speed pin (goes to PWM2)
int mspeed = 10; 
int turnspeed=50; 



int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=20;
float pid_i=20;
float pid_d=0;
////////////////////////PID CONSTANST/////////////////////
float kp=25;
float ki=0;
float kd=0.8;
float desired_angle = 0;//////////////TARGET ANGLE/////////////

int pin1 = 3;  // This is input for RC (tank mixed) drive 1
int pin2 = 4;  // This is input for RC (tank mixed) drive 2
int duration1 = 1500; // Duration of the pulse from the RC
int duration2 = 1500; // Duration of the pulse from the RC
int motorspeed1 = 0;
int motordirection1 = HIGH;
int motorspeed2 = 0 ;
int motordirection2 = HIGH;



// =======================================================================================
//                          Main Program
// =======================================================================================

void setup()
{
    //Debug Serial for use with USB Debugging
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    if (Usb.Init() == -1)
    {
        Serial.print(F("\r\nOSC did not start"));
        while (1); //halt
    }
    Serial.print(F("\r\nBluetooth Library Started"));
    output.reserve(200); // Reserve 200 bytes for the output string

    //Setup for PS3
    PS3Nav->attachOnInit(onInitPS3); // onInit() is called upon a new connection - you can call the function whatever you like
    PS3Nav2->attachOnInit(onInitPS3Nav2); 

    //The Arduino Mega has three additional serial ports: 
    // - Serial1 on pins 19 (RX) and 18 (TX), 
    // - Serial2 on pins 17 (RX) and 16 (TX), 
    // - Serial3 on pins 15 (RX) and 14 (TX). 

    leftFootSignal.attach(leftFootPin);
    rightFootSignal.attach(rightFootPin);
 
    stopFeet();

     Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    ////////////////PIN MODE DEFINATIONS//////////////////////
    pinMode(dir1pin,OUTPUT);
    pinMode(spe1pin,OUTPUT);
    pinMode(dir2pin,OUTPUT);
    pinMode(spe2pin,OUTPUT);
    //Serial.begin(9600);
    time = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////
  
}

boolean readUSB()
{
    //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
    Usb.Task();
    if (PS3Nav->PS3NavigationConnected ) Usb.Task();
    if (PS3Nav2->PS3NavigationConnected ) Usb.Task();
    if ( criticalFaultDetect() )
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      flushAndroidTerminal();
      return false;
    }
	//Fix backported from Shadow_MD to fix "Dome Twitch"
    if (criticalFaultDetectNav2())
    { 
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      return false;
    }
    return true;
}

void loop()
{
    initAndroidTerminal();
    
    //Useful to enable with serial console when having controller issues.
    #ifdef TEST_CONROLLER
      testPS3Controller();
    #endif

    //LOOP through functions from highest to lowest priority.

    if ( !readUSB() )
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      return;
    }
    footMotorDrive();
        
    if ( !readUSB() )
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      return;
    }
    
    flushAndroidTerminal();
}


void onInitPS3()
{
    String btAddress = getLastConnectedBtMAC();
    PS3Nav->setLedOn(LED1);
    isPS3NavigatonInitialized = true;
    badPS3Data = 0;
    #ifdef SHADOW_DEBUG
      output += "\r\nBT Address of Last connected Device when Primary PS3 Connected: ";
      output += btAddress;
      if (btAddress == PS3MoveNavigatonPrimaryMAC)
      {
          output += "\r\nWe have our primary controller connected.\r\n";
      }
      else
      {
          output += "\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n";
      }
    #endif
}

void onInitPS3Nav2()
{
    String btAddress = getLastConnectedBtMAC();
    PS3Nav2->setLedOn(LED1);
    isSecondaryPS3NavigatonInitialized = true;
    badPS3Data = 0;
    if (btAddress == PS3MoveNavigatonPrimaryMAC) swapPS3NavControllers();
    #ifdef SHADOW_DEBUG
      output += "\r\nBT Address of Last connected Device when Secondary PS3 Connected: ";
      output += btAddress;
      if (btAddress == PS3MoveNavigatonPrimaryMAC)
      {
          output += "\r\nWe have our primary controller connecting out of order.  Swapping locations\r\n";
      }
      else
      {
          output += "\r\nWe have a secondary controller connected.\r\n";
      }
    #endif
}

String getLastConnectedBtMAC()
{
    String btAddress = "";
    for(int8_t i = 5; i >= 0; i--)
    {
        if (btAddress.length() > 0)
    {
            btAddress +=(":");
        }
        if (Btd.disc_bdaddr[i]<0x10)
        {
            btAddress +="0";
        }
        btAddress += String(Btd.disc_bdaddr[i], HEX);
    }
    btAddress.toUpperCase();
    return btAddress; 
}

void swapPS3NavControllers()
{
    PS3BT* temp = PS3Nav;
    PS3Nav = PS3Nav2;
    PS3Nav2 = temp;
    //Correct the status for Initialization
    boolean tempStatus = isPS3NavigatonInitialized;
    isPS3NavigatonInitialized = isSecondaryPS3NavigatonInitialized;
    isSecondaryPS3NavigatonInitialized = tempStatus;
    //Must relink the correct onInit calls
    PS3Nav->attachOnInit(onInitPS3);
    PS3Nav2->attachOnInit(onInitPS3Nav2); 
}


void initAndroidTerminal()
{
    #ifdef BLUETOOTH_SERIAL
    //Setup for Bluetooth Serial Monitoring
    if (SerialBT.connected)
    {
        if (firstMessage)
        {
            firstMessage = false;
            SerialBT.println(F("Hello from S.H.A.D.O.W.")); // Send welcome message
        }
        //TODO:  Process input from the SerialBT
        //if (SerialBT.available())
        //    Serial.write(SerialBT.read());
    }
    else
    {
        firstMessage = true;
    }
    #endif
}

void flushAndroidTerminal()
{
    if (output != "")
    {
        if (Serial) Serial.println(output);
        #ifdef BLUETOOTH_SERIAL
        if (SerialBT.connected)
            SerialBT.println(output);
            SerialBT.send();
        #endif
        output = ""; // Reset output string
    }
}


// =======================================================================================
// //////////////////////////Process PS3 Controller Fault Detection///////////////////////
// =======================================================================================
//TODO:  boolean criticalFaultDetect(PS3BT* myPS3 = PS3Nav, int controllerNumber = 1)
boolean criticalFaultDetect()
{
    if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected)
    {
        lastMsgTime = PS3Nav->getLastMessageTime();
        currentTime = millis();
        if ( currentTime >= lastMsgTime)
        {
          msgLagTime = currentTime - lastMsgTime;
        } else
        {
             #ifdef SHADOW_DEBUG
               output += "Waiting for PS3Nav Controller Data\r\n";
             #endif
             badPS3Data++;
             msgLagTime = 0;
        }
        
        if (msgLagTime > 100 && !isFootMotorStopped)
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 100ms since we heard from the PS3 Controller\r\n";
              output += "Shut downing motors, and watching for a new PS3 message\r\n";
            #endif
            stopFeet();
            isFootMotorStopped = true;
            return true;
        }
        if ( msgLagTime > 30000 )
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 30s since we heard from the PS3 Controller\r\n";
              output += "msgLagTime:";
              output += msgLagTime;
              output += "  lastMsgTime:";
              output += lastMsgTime;
              output += "  millis:";
              output += millis();            
              output += "\r\nDisconnecting the controller.\r\n";
            #endif
            PS3Nav->disconnect();
        }

        //Check PS3 Signal Data
        if(!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
        {
            // We don't have good data from the controller.
            //Wait 10ms, Update USB, and try again
            delay(10);
            Usb.Task();
            if(!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
            {
                badPS3Data++;
                #ifdef SHADOW_DEBUG
                    output += "\r\nInvalid data from PS3 Controller.";
                #endif
                return true;
            }
        }
        else if (badPS3Data > 0)
        {
            //output += "\r\nPS3 Controller  - Recovered from noisy connection after: ";
            //output += badPS3Data;
            badPS3Data = 0;
        }
        if ( badPS3Data > 10 )
        {
            #ifdef SHADOW_DEBUG
                output += "Too much bad data coming from the PS3 Controller\r\n";
                output += "Disconnecting the controller.\r\n";
            #endif
            PS3Nav->disconnect();
        }
    }
    else if (!isFootMotorStopped)
    {
        #ifdef SHADOW_DEBUG      
            output += "No Connected Controllers were found\r\n";
            output += "Shuting downing motors, and watching for a new PS3 message\r\n";
        #endif
        stopFeet();
        isFootMotorStopped = true;
        return true;
    }
    return false;
}
// =======================================================================================
// //////////////////////////END of PS3 Controller Fault Detection///////////////////////
// =======================================================================================

// =======================================================================================
// //////////////////////////Process of PS3 Secondary Controller Fault Detection//////////
// =======================================================================================
//TODO:  This is moved as is, but should merge with above function.
boolean criticalFaultDetectNav2()
{
  if (PS3Nav2->PS3NavigationConnected || PS3Nav2->PS3Connected)
  {
    lastMsgTime = PS3Nav2->getLastMessageTime();
    currentTime = millis();
    
    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;
    } 
    else
    {
      #ifdef SHADOW_DEBUG
        output += "Waiting for PS3Nav Secondary Controller Data\r\n";
      #endif
      badPS3Data++;
      msgLagTime = 0;
    }
    
    if ( msgLagTime > 10000 )
    {
      #ifdef SHADOW_DEBUG
        output += "It has been 10s since we heard from the PS3 secondary Controller\r\n";
        output += "msgLagTime:";
        output += msgLagTime;
        output += " lastMsgTime:";
        output += lastMsgTime;
        output += " millis:";
        output += millis(); 
        output += "\r\nDisconnecting the secondary controller.\r\n";
      #endif

      PS3Nav2->disconnect();
      return true;
    }
    
    //Check PS3 Signal Data
    if(!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged))
    {
      // We don't have good data from the controller.
      //Wait 15ms, Update USB, and try again
      delay(15);
      Usb.Task();
      if(!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged))
      {
        badPS3Data++;
        #ifdef SHADOW_DEBUG
          output += "\r\nInvalid data from PS3 Secondary Controller.";
        #endif
        return true;
      }
    }
    else if (badPS3Data > 0)
    {
      badPS3Data = 0;
    }
  
    if ( badPS3Data > 10 )
    {
      #ifdef SHADOW_DEBUG
        output += "Too much bad data coming from the PS3 Secondary Controller\r\n";
        output += "Disconnecting the controller.\r\n";
      #endif

      PS3Nav2->disconnect();
      return true;
    }
  }
  
  return false;
}
// =======================================================================================
// //////////////////////////END of PS3 Secondary Controller Fault Detection//////////////
// =======================================================================================


// =======================================================================================
// //////////////////////////Mixing Function for R/C Mode////////////////////////////////
// =======================================================================================

int leftFoot,rightFoot; //will hold foot speed values (-100 to 100)

void mixBHD(byte stickX, byte stickY, byte maxDriveSpeed){  
    // This is BigHappyDude's mixing function, for differential (tank) style drive using two motor controllers.
    // Takes a joysticks X and Y values, mixes using the diamind mix, and output a value 0-180 for left and right motors.     
    // 180,180 = both feet full speed forward.
    // 000,000 = both feet full speed reverse.
    // 180,000 = left foot full forward, right foot full reverse (spin droid clockwise)
    // 000,180 = left foot full reverse, right foot full forward (spin droid counter-clockwise)
    // 090,090 = no movement
    // for simplicity, we think of this diamond matrix as a range from -100 to +100 , then map the final values to servo range (0-180) at the end 
    //  Ramping and Speed mode applied on the droid.  
    if(((stickX <= 113) || (stickX >= 141)) || ((stickY <= 113) || (stickY >= 141))){  //  if movement outside deadzone
      //  Map to easy grid -100 to 100 in both axis, including deadzones.
      int YDist = 0;  // set to 0 as a default value if no if used.
      int XDist = 0;
      if(stickY <= 113){
       YDist = (map(stickY, 0, 113, 100, 1));           //  Map the up direction stick value to Drive speed
      } else if(stickY >= 141){
       YDist = (map(stickY, 141, 255, -1, -100));       //  Map the down direction stick value to Drive speed
      }
      if(stickX <= 113){
       XDist = (map(stickX, 0, 113, -100, -1));       //  Map the left direction stick value to Turn speed
      } else if(stickX >= 141){
       XDist = (map(stickX, 141, 255, 1, 100));   //  Map the right direction stick value to Turn speed
      }

      /* Debugging by KnightShade 
      //Driving is TOO sensitive.   Need to dial down the turning to a different scale factor.
      This code will map teh linear values to a flatter value range.

      //The larger SteeringFactor is the less senstitive steering is...  
      //Smaller values give more accuracy in making fine steering corrections
        XDist*sqrt(XDist+SteeringFactor)
      */
      //Convert from Linear to a scaled/exponential Steering system
      int SteeringFactor = 100; //TODO - move a constant at top of script
      int TempScaledXDist =  (int) (abs(XDist)*sqrt(abs(XDist)+SteeringFactor));
      int MaxScale = (100*sqrt(100+SteeringFactor));
      XDist = (map(stickX, 0, MaxScale, 1, 100));       //  Map the left direction stick value to Turn speed
            
      if(stickX <= 113){
        XDist = -1*(map(TempScaledXDist, 0, MaxScale, 1, 100));  //  Map the left direction stick value to Turn speed
      } else if(stickX >= 141){
        XDist = (map(TempScaledXDist, 0, MaxScale, 1, 100));   //  Map the right direction stick value to Turn speed
      }
      //END Convert from Linear to a scaled/exponential Steering system
      
      //  Constrain to Diamond values.  using 2 line equations and find the intersect, boiled down to the minimum
      //  This was the inspiration; https://github.com/declanshanaghy/JabberBot/raw/master/Docs/Using%20Diamond%20Coordinates%20to%20Power%20a%20Differential%20Drive.pdf 
      float TempYDist = YDist;
      float TempXDist = XDist;
      if (YDist>(XDist+100)) {  //  if outside top left.  equation of line is y=x+Max, so if y > x+Max then it is above line
        // OK, the first fun bit. :)  so for the 2 lines this is always true y = m1*x + b1 and y = m2*x - b2
        // y - y = m1*x + b1  - m2*x - b2  or 0 = (m1 - m2)*x + b1 - b2
        // We have y = x+100 and y = ((change in y)/Change in x))x
        // So:   x = -100/(1-(change in y)/Change in x)) and using y = x+100 we can find y with the new x
        // Not too bad when simplified. :P
        TempXDist = -100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist+100;
      } else if (YDist>(100-XDist)) {  //  if outside top right
        // repeat intesection for y = 100 - x
        TempXDist = -100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist+100;
      } else if (YDist<(-XDist-100)) {  //  if outside bottom left
        // repeat intesection for y = -x - 100
        TempXDist = 100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist-100;
      } else if (YDist<(XDist-100)) {  //  if outside bottom right
        // repeat intesection for y = x - 100
        TempXDist = 100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist-100;
      }
      //  all coordinates now in diamond. next translate to the diamond coordinates.
      //  for the left.  send ray to y = x + Max from coordinates along y = -x + b
      //  find for b, solve for coordinates and resut in y then scale using y = (y - max/2)*2
      float LeftSpeed = ((TempXDist+TempYDist-100)/2)+100;
      LeftSpeed = (LeftSpeed-50)*2;
      //  for right send ray to y = -x + Max from coordinates along y = x + b find intersction coordinates and then use the Y vaule and scale.
      float RightSpeed = ((TempYDist-TempXDist-100)/2)+100;
      RightSpeed = (RightSpeed-50)*2;
      //  this results in a -100 to 100 range of speeds, so shift to servo range.

      /* Debugging by KnightShade - this didn't do the speed like we expected.  Notice that they are constant values.....
      // map(maxDriveSpeed, 0, 127, 90, 180); //drivespeed was defined as 0 to 127 for Sabertooth serial, now we want something in an upper servo range (90 to 180)
      #if leftDirection == 0
      leftFoot=map(LeftSpeed, -100, 100, 180, 0);
      #else
      leftFoot=map(LeftSpeed, -100, 100, 0, 180);
      #endif
      #if rightDirection == 0
      rightFoot=map(RightSpeed, -100, 100, 180, 0);
      #else
      rightFoot=map(RightSpeed, -100, 100, 0, 180);
      #endif
      
      First pass, treat the throttle as ON/OFF - not an Analog shift (as Sabertooth code does)
      Based on that Paul passed in Drive Speed 1 or 2.
      */
      int maxServoForward = map(maxDriveSpeed, 0, 127, 0, 255); //drivespeed was defined as 0 to 127 for Sabertooth serial, now we want something in an upper servo range (90 to 180)
      int maxServoReverse = map(maxDriveSpeed, 0, 127, -255, 0); //drivespeed was defined as 0 to 127 for Sabertooth serial, now we want something in an upper servo range (90 to 0)
      #if leftDirection == 0
      leftFoot=map(LeftSpeed, -100, 100, maxServoForward, maxServoReverse);
      #else
      leftFoot=map(LeftSpeed, -100, 100, maxServoReverse, maxServoForward);
      #endif
      #if rightDirection == 0
      rightFoot=map(RightSpeed, -100, 100, maxServoForward, maxServoReverse);
      #else
      rightFoot=map(RightSpeed, -100, 100, maxServoReverse, maxServoForward);
      #endif
      /*  END Knightshade Debug */
    } else {
      leftFoot=0;
      rightFoot=0;
    }      
}                                   

// =======================================================================================
// ////////////////////////END:  Mixing Function for R/C Mode/////////////////////////////
// =======================================================================================


// quick function to stop the feet depending on which drive system we're using...
void stopFeet() {

  leftFootSignal.write(90);
  rightFootSignal.write(90);

}


boolean ps3FootMotorDrive(PS3BT* myPS3 = PS3Nav)
{
  int footDriveSpeed = 0;
  int stickSpeed = 0;
  int turnnum = 0;

  if (isPS3NavigatonInitialized)
  {
      // Additional fault control.  Do NOT send additional commands to Sabertooth if no controllers have initialized.
      if (!isStickEnabled)
      {
            #ifdef SHADOW_VERBOSE
              if ( abs(myPS3->getAnalogHat(LeftHatY)-128) > joystickFootDeadZoneRange)
              {
                output += "Drive Stick is disabled\r\n";
              }
            #endif
          stopFeet();
          isFootMotorStopped = true;
      } else if (!myPS3->PS3NavigationConnected)
      {
          stopFeet();
          isFootMotorStopped = true;
      } else if ( myPS3->getButtonPress(L1) )
      {
          //TODO:  Does this need to change this when we support dual controller, or covered by improved isStickEnabled
          stopFeet();
          isFootMotorStopped = true;
      } else
      {
          //make those feet move!!!///////////////////////////////////////////////////
          int joystickPosition = myPS3->getAnalogHat(LeftHatY);
          isFootMotorStopped = false;
          #if FOOT_CONTROLLER == 0
          if (myPS3->getButtonPress(L2))
          {
            int throttle = 0;
            if (joystickPosition < 127)
            {
                throttle = joystickPosition - myPS3->getAnalogButton(L2);
            } else
            {
                throttle = joystickPosition + myPS3->getAnalogButton(L2);
            }
            stickSpeed = (map(throttle, -255, 510, -drivespeed2, drivespeed2));                
          } else 
          {
            stickSpeed = (map(joystickPosition, 0, 255, -drivespeed1, drivespeed1));
          }          

          if ( abs(joystickPosition-128) < joystickFootDeadZoneRange)
          {
              footDriveSpeed = 0;
          } else if (footDriveSpeed < stickSpeed)
          {
              if (stickSpeed-footDriveSpeed<(ramping+1))
                  footDriveSpeed+=ramping;
              else
                  footDriveSpeed = stickSpeed;
          }
          else if (footDriveSpeed > stickSpeed)
          {
              if (footDriveSpeed-stickSpeed<(ramping+1))
                  footDriveSpeed-=ramping;
              else
                  footDriveSpeed = stickSpeed;  
          }
          
          turnnum = (myPS3->getAnalogHat(LeftHatX));

          //TODO:  Is there a better algorithm here?  
          if ( abs(footDriveSpeed) > 50)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 54, 200, -(turnspeed/4), (turnspeed/4)));
          else if (turnnum <= 200 && turnnum >= 54)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 54, 200, -(turnspeed/3), (turnspeed/3)));
          else if (turnnum > 200)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 201, 255, turnspeed/3, turnspeed));
          else if (turnnum < 54)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 0, 53, -turnspeed, -(turnspeed/3)));
          #endif

          currentMillis = millis();
          if ( (currentMillis - previousFootMillis) > serialLatency  )
          {

          #ifdef SHADOW_VERBOSE      
          if ( footDriveSpeed < -driveDeadBandRange || footDriveSpeed > driveDeadBandRange)
          {
            output += "Driving Droid at footSpeed: ";
            output += footDriveSpeed;
            output += "!  DriveStick is Enabled\r\n";
            output += "Joystick: ";              
            output += myPS3->getAnalogHat(LeftHatX);
            output += "/";              
            output += myPS3->getAnalogHat(LeftHatY);
            output += " turnnum: ";              
            output += turnnum;
            output += "/";              
            output += footDriveSpeed;
            output += " Time of command: ";              
            output += millis();
          }
          #endif


            //Experimental Q85. Untested Madness!!! Use at your own risk and expect your droid to run away in flames.
            //use BigHappyDude's mixing algorythm to get values for each foot...
            if (myPS3->getButtonPress(L2)) mixBHD(myPS3->getAnalogHat(LeftHatX),myPS3->getAnalogHat(LeftHatY),drivespeed2);
            else mixBHD(myPS3->getAnalogHat(LeftHatX),myPS3->getAnalogHat(LeftHatY),drivespeed1);
            //now we've got values for leftFoot and rightFoot, output those somehow...
            
            
            motorspeed1 = map (duration1,1000,2000,-255,255); //Maps the duration to the motorspeed from the stick
            motorspeed2 = map (duration2,1000,2000,-255,255); //Maps the duration to the motorspeed from the stick
            
              /*////////////////////////WARNING//////////////////////
               * DO NOT USE ANY DELAYS INSIDE THE LOOP OTHERWISE THE BOT WON'T BE 
               * ABLE TO CORRECT THE BALANCE FAST ENOUGH
               * ALSO, DONT USE ANY SERIAL PRINTS. BASICALLY DONT SLOW DOWN THE LOOP SPEED.
              */
                timePrev = time;  
                time = millis();  
                elapsedTime = (time - timePrev) / 1000; 
                Wire.beginTransmission(0x68);
                Wire.write(0x3B); 
                Wire.endTransmission(false);
                Wire.requestFrom(0x68,6,true);
                ////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU///////////////// 
                Acc_rawX=Wire.read()<<8|Wire.read(); 
                Acc_rawY=Wire.read()<<8|Wire.read();
                Acc_rawZ=Wire.read()<<8|Wire.read(); 
                /////////////////////CONVERTING RAW DATA TO ANGLES/////////////////////
                Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
                Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
                Wire.beginTransmission(0x68);
                Wire.write(0x43); 
                Wire.endTransmission(false);
                Wire.requestFrom(0x68,4,true); 
                //////////////////PULLING RAW GYRO DATA FROM IMU/////////////////////////
                Gyr_rawX=Wire.read()<<8|Wire.read(); 
                Gyr_rawY=Wire.read()<<8|Wire.read(); 
                ////////////////////CONVERTING RAW DATA TO ANGLES///////////////////////
                Gyro_angle[0] = Gyr_rawX/131.0; 
                Gyro_angle[1] = Gyr_rawY/131.0;
                //////////////////////////////COMBINING BOTH ANGLES USING COMPLIMENTARY FILTER////////////////////////
                Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
                Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
                ////TOTAL_ANGLE[0] IS THE PITCH ANGLE WHICH WE NEED////////////
                error = Total_angle[0] - desired_angle; /////////////////ERROR CALCULATION////////////////////
                ///////////////////////PROPORTIONAL ERROR//////////////
                pid_p = kp*error;
                ///////////////////////INTERGRAL ERROR/////////////////
                pid_i = pid_i+(ki*error);  
                ///////////////////////DIFFERENTIAL ERROR//////////////
                pid_d = kd*((error - previous_error)/elapsedTime);
                ///////////////////////TOTAL PID VALUE/////////////////
                PID = pid_p + pid_d;
                ///////////////////////UPDATING THE ERROR VALUE////////
                previous_error = error;
                //Serial.println(PID);                     //////////UNCOMMENT FOR DDEBUGGING//////////////
                //delay(60);                               //////////UNCOMMENT FOR DDEBUGGING//////////////
               //Serial.println(Total_angle[0]);          //////////UNCOMMENT FOR DDEBUGGING//////////////
                /////////////////CONVERTING PID VALUES TO ABSOLUTE VALUES//////////////////////////////////
                mspeed = abs(PID);
            
                mspeed=map(mspeed,0,2100,0,700);
                
                //Serial.println(mspeed);                  //////////UNCOMMENT FOR DDEBUGGING//////////////
                ///////////////SELF EXPLANATORY///////////////
               
                if(Total_angle[0]<0)
                  {
                   mspeed=-mspeed;
                  }
                if(Total_angle[0]>0)
                  {
                  mspeed = mspeed;
                  }
                
            motorspeed1=motorspeed1+mspeed; //This add the PS3 drive to the correction drive, motorspeed is the PS3, mspeed from the IMU
            motorspeed2=motorspeed2-mspeed; //This add the PS3 drive to the correction drive, motorspeed is the PS3, mspeed from the IMU
            
            if (motorspeed1<0) {
              motordirection1 = LOW;
              motorspeed1=-motorspeed1;
            }
            
            else if (motorspeed1>0) {
             motordirection1 = HIGH;  
            }
            
            
            if (motorspeed2>-15 && motorspeed2<15){
              motorspeed2=0;
            
            }
              
            if (motorspeed1>-15 && motorspeed1<15){
              motorspeed2=0;
            }
            
            
            if (motorspeed2<0) {
              motordirection2 = LOW;
              motorspeed2=-motorspeed2;
            }
            
            else if (motorspeed2>0) {
             motordirection2 = HIGH;  
            }
            
            if (motorspeed1 >254){
              motorspeed1=255;
            }
            
            if (motorspeed2 >254){
              motorspeed2=255;
            }
            
            //Serial.print (motorspeed1);
            //Serial.print (" ");
            //Serial.print (motorspeed2);
            //Serial.println (" ");
            
            digitalWrite(dir1pin,motordirection1);
            analogWrite(spe1pin,leftFoot); //increase the speed of the motor from 0 to 255
            digitalWrite(dir2pin,motordirection2);
            analogWrite(spe2pin,rightFoot); //increase the speed of the motor from 0 to 255
            
           previousFootMillis = currentMillis;
          return true; //we sent a foot command   
          }
      }
  }
  return false;
}



void ps3ToggleSettings(PS3BT* myPS3 = PS3Nav)
{
    if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(L3))
    {
      //Quick Shutdown of PS3 Controller
      output += "\r\nDisconnecting the controller.\r\n";
      myPS3->disconnect();
    }

  
    // enable / disable Drive stick & play sound
    if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(CROSS))
    {
        #ifdef SHADOW_DEBUG
          output += "Disabling the DriveStick\r\n";
        #endif
        isStickEnabled = false;
//        trigger.play(52);
    }
    if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(CIRCLE))
    {
        #ifdef SHADOW_DEBUG
          output += "Enabling the DriveStick\r\n";
        #endif
        isStickEnabled = true;
//        trigger.play(53);
    }


}


void footMotorDrive()
{
  //Flood control prevention
  if ((millis() - previousFootMillis) < serialLatency) return;  
  if (PS3Nav->PS3NavigationConnected) ps3FootMotorDrive(PS3Nav);
  //TODO:  Drive control must be mutually exclusive - for safety
  //Future: I'm not ready to test that before FanExpo
  //if (PS3Nav2->PS3NavigationConnected) ps3FootMotorDrive(PS3Nav2);
}  



void toggleSettings()
{
   if (PS3Nav->PS3NavigationConnected) ps3ToggleSettings(PS3Nav);
   if (PS3Nav2->PS3NavigationConnected) ps3ToggleSettings(PS3Nav2);
}  
 
