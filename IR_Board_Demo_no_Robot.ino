/*********************************IR DEV CIRCUIT TEST DEVELOPMENT*******************************
*                                         May 31 2017
*                                         Erik Cegnar
*                                      Rob Helvestine
***********************************************************************************************/
#include <IOShieldOled2.h>
#include <IR_Board_defs.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

/*******************************************VARIABLES******************************************/
int i=0;
volatile int Analog[6];                                     //index for recorded Analog values, when transmitter are on
volatile int Analog_Last[6];                                //index for previously recorded Analog values
volatile int Analog_dV_dt[6];                               //index for derivative signals
volatile int Analog_Receive[6];                             //index for recorded Analog values, when transmitters are off
volatile int VisibleLED[18] = {0};                          //index for each visible LED, when transmitters are on
volatile int VisibleLED_Receive[18] = {0};                  //index for each visible LED, when transmitters are off
volatile int VisibleLED_PWM[18] = {0};                      //index for duty cycles for visible LEDs
volatile int VisibleLED_Previous_PWM[18] = {0};             //previous PWM values used for comparison
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();    //used for hardware PWM
float smoothedVal[7];                                       //array for digital filtering, when transmitters are on
float smoothedVal_Receive[7];                               //array for digital filtering, when transmitters are off
float filterVal = 0.7;                                      //number between 0 (no filter) and <1 (very slow filter)
float A0angle = 0.0;                                        //fixed angles for LED pairs, in radians
float A5angle = 1.047;                                      //fixed angles for LED pairs, in radians
float A4angle = 2.094;                                      //fixed angles for LED pairs, in radians
float A3angle = 3.142;                                      //fixed angles for LED pairs, in radians
float A2angle = 4.189;                                      //fixed angles for LED pairs, in radians
float A1angle = 5.236;                                      //fixed angles for LED pairs, in radians
int A0baseline = 360;                                       //baseline for analog in channel A0
int A1baseline = 340;                                       //baseline for analog in channel A1
int A2baseline = 375;                                       //baseline for analog in channel A2
int A3baseline = 350;                                       //baseline for analog in channel A3
int A4baseline = 370;                                       //baseline for analog in channel A4
int A5baseline = 360;                                       //baseline for analog in channel A5
int A0TXbaseline = 520;                                     //baseline for analog in channel A0, when Tx is on
int A1TXbaseline = 365;                                     //baseline for analog in channel A1, when Tx is on
int A2TXbaseline = 410;                                     //baseline for analog in channel A2, when Tx is on
int A3TXbaseline = 385;                                     //baseline for analog in channel A3, when Tx is on
int A4TXbaseline = 400;                                     //baseline for analog in channel A4, when Tx is on
int A5TXbaseline = 370;                                     //baseline for analog in channel A5, when Tx is on
volatile int TX_on1_off0;                                   //1 = Tx is on, 0 = Tx is off
int multipleLEDs1_singleLED0;                               //1 = multiple visible LEDs, 0 = single visible (max) LED
int obstacledetect1_vector0;                                //1 = obstacle detect mode, 0 = incoming vector mode
char vector_array[5] = "    ";                              //initialize char array to eventually hold vector angle
double vector;                                              //degree of incoming light
unsigned long timer;                                        //used to keep track of time for gesture control
int VLED_on_time = 40;                                      //on time for visible LEDs
int VLED_off_time = 5;                                      //off time for visible LEDs
volatile uint32_t resetcounter = 0;                         //counter that gets reset every successfully received packet
volatile uint32_t slowcounter = 0;                          //12Hz slow counter variable
volatile uint32_t counter = 0;                              //10kHz counter variable
volatile uint32_t gesture_reset_counter = 0;                //10Hz gesture counter variable, reset at begining to prevent false gestures
volatile uint32_t gesture_counter = 0;                      //10Hz gesture counter variable
volatile uint32_t timer_hold = 0;                           //hold variable for 10kHz counter
volatile int bit_number = 0;                                //between 0-31 for indexing receive packet
volatile unsigned long receive_packet = 0x00000000;         //receive packet of 4 bytes
volatile unsigned long successful_packet = 0x00000000;      //variable to transfer receive packet into
volatile int first_read = 0;                                //variable to read pin at specific time
volatile int second_read = 0;                               //variable to read pin at specific time
volatile int cmdbyte_received = 0b00000000;                 //command byte variable
volatile int databyte1_received = 0b00000000;               //first data byte variable
volatile int databyte2_received = 0b00000000;               //second data byte variable
volatile int successful_cmdbyte = 0b00000000;               //successful command byte variable
volatile int successful_databyte1 = 0b00000000;             //successful databyte1 variable
volatile int successful_databyte2 = 0b00000000;             //successful databyte2 variable
volatile int checksum_received = 0b00000000;                //checksum variable
volatile int pintoread;                                     //variable to read digital input
volatile unsigned long flagtoclear;                         //variable to clear interrupt flag
volatile int i_plus_gesture;                                //positive index for next gesture
volatile int i_minus_gesture;                               //negative index for next gesture
volatile int gesture_threshold = 40;                        //threshold for derivative signal
volatile int i_gesture = 0;                                 //index for gesturing
volatile int gesture_flag = 0;                              //flag for gesturing
volatile int gesture_timer = 0;                             //timer used for gesturing
volatile int light_show = 0;                                //flag for what light show to perform
volatile int saved_index;                                   //index for CW or CCW gesture light show
volatile int start_bit_read;                                //time elapsed after initial ext int to read bit info
char interrupt_pin[5] = "    ";                             //character array to display triggered interrupt
int i_VLED2 = 0;                                            //index for visible LEDs, used in Set_Visible_LEDs
double A0reading_xcomp;                                     //X component of analog reading of the A0 channel
double A0reading_ycomp;                                     //Y component of analog reading of the A0 channel
double A5reading_xcomp;                                     //X component of analog reading of the A5 channel
double A5reading_ycomp;                                     //Y component of analog reading of the A5 channel
double A4reading_xcomp;                                     //X component of analog reading of the A4 channel
double A4reading_ycomp;                                     //Y component of analog reading of the A4 channel
double A1reading_xcomp;                                     //X component of analog reading of the A1 channel
double A1reading_ycomp;                                     //Y component of analog reading of the A1 channel
double A2reading_xcomp;                                     //X component of analog reading of the A2 channel
double A2reading_ycomp;                                     //Y component of analog reading of the A2 channel
double A3reading_xcomp;                                     //X component of analog reading of the A3 channel
double A3reading_ycomp;                                     //Y component of analog reading of the A3 channel
int vector_degrees_int;                                     //vector expressed in degrees as an int
double vector_degrees;                                      //vector expressed in degrees as a double
double x_sum;                                               //sum of x components of channels
double y_sum;                                               //sum of y components of channels

/*******************************************FUNCTIONS******************************************/
void Set_Analog_CHs_HWPWM(void);                            //sets PWM values using the hardware PWM board
void Set_Visible_LEDs(void);                                //This is the main function that lights the visible LEDs
void Light_Max_LED_18(void);                                //Lights one (of 18) LED based on max reading
int digital_filter(int ADCinput);                           //Digital filter 
void Read_Analog_Set_Mag_All_VLEDs(void);                   //Reads analog channels, sets vector magnitude for each visible LED
void CCW_Visible_LEDs_index(int index);                     //program to light visible LEDs in CCW direction
void CW_Visible_LEDs_index(int index);                      //program to light visible LEDs in CW direction   
void Optical_Transmit_Start_Byte(void);                     //Transmit of start byte via optical communication
void start_timer_1(uint32_t frequency_12hz);                //Starts timer 1 used in visual displays
void start_timer_2(uint32_t frequency_VLED);                //Starts timer 2 used in visible LEDS 16 & 17
void start_timer_3(uint32_t frequency);                     //Starts timer 3 used in accurate delays / counting      
void start_timer_4(uint32_t frequency_10hz);                //Starts timer 4 used in emitting info
void start_timer_5(uint32_t frequency_1khz);                //Starts timer 5 used in external interrupts
void Optical_Transmit_Packet(int cmdbyte, int databyte1, int databyte2);  //optically transmit packet of data
void Test_Board(void);                                      //Program that tests newly built boards.
void Main_Program_Demo(void);                               //Main program for demo

/*****************************************START PROGRAM****************************************/
/*******************************************INTERRUPTS*****************************************/
void __attribute__((interrupt)) VisualDisplayISR() {              //12Hz counter isr (Timer 1)
  slowcounter++;                                                  //increment counter
  resetcounter++;                                                 //increment another counter that gets periodically reset
  if(slowcounter % 3 == 0){                                       //slows down refresh of display
    Wire.endTransmission();
    Set_Visible_LEDs();                                           //Set PWM of visible LEDs
    if((vector_degrees > 0.5) && (obstacledetect1_vector0 == 0)){                                     //display received vector angle when present
      IOShieldOled.setCursor(0, 0);
      IOShieldOled.putString("Vector angle:");
      IOShieldOled.setCursor(4, 1);
      IOShieldOled.putString("degrees");
      IOShieldOled.setCursor(0, 1);
      IOShieldOled.putString("   ");
      IOShieldOled.setCursor(0, 1);
      IOShieldOled.putString(vector_array);                       //print vector  
    }
    else{                                                         //clear display if vector angle is not present
      IOShieldOled.setCursor(0, 0);
      IOShieldOled.putString("IR Board Demo");
      IOShieldOled.setCursor(4, 1);
      IOShieldOled.putString("       ");
      IOShieldOled.setCursor(0, 1);
      IOShieldOled.putString("   ");
    }
    if((resetcounter < 61) && (successful_packet > 0)){           //if data has been received in the last 5 seconds, display on screen
      IOShieldOled.setCursor(0, 2);
      IOShieldOled.putString("Rcv data...");
      IOShieldOled.setCursor(11, 2);
      IOShieldOled.putString(interrupt_pin);
      if((successful_databyte1 == R2D2_byte1) && (successful_databyte2 == R2D2_byte2)){
        IOShieldOled.setCursor(0, 3);
        IOShieldOled.putString("R2D2");
      }
      if((successful_databyte1 == BB9E_byte1) && (successful_databyte2 == BB9E_byte2)){
        IOShieldOled.setCursor(0, 3);
        IOShieldOled.putString("BB9E");
      }
      if((successful_databyte1 == LMQ_byte1) && (successful_databyte2 == LMQ_byte2)){
        IOShieldOled.setCursor(0, 3);
        IOShieldOled.putString("LMQ");
      }
    }
    else{
      IOShieldOled.setCursor(0, 2);                               //if data hasn't been received in the last 5 seconds, erase screen
      IOShieldOled.putString("                ");
      IOShieldOled.setCursor(0, 3);
      IOShieldOled.putString("                ");
    }
  }
  clearIntFlag(_TIMER_1_IRQ);                                     //clear Timer 1 flag
}

void __attribute__((interrupt)) myCounterISR() {                  //10khz counter isr (Timer 3)
  counter++;
  clearIntFlag(_TIMER_3_IRQ);                                     //Clear this interrupt flag (Timer 3)  
}

void __attribute__((interrupt)) EmitterISR() {                    //10hz (semi random) counter isr (Timer 4)
  gesture_reset_counter++;
  gesture_counter++; 
  if(gesture_counter % 10 == 0){                                  //emit optical comm data every 10th clock cycle
//    light_show = 3;                                               //uncomment if we want a light blink per optical transmit, not recommended
    Optical_Transmit_Packet(identification_byte, BB9E_byte1, BB9E_byte2);
//    Optical_Transmit_Packet(identification_byte, R2D2_byte1, R2D2_byte2);
    start_timer_4(random(8, 11));                                 //random number between 8-10 Hz 
    clearIntFlag(_TIMER_4_IRQ);                                   //Clear EmitterISR flag
  } 
  else{    
    TX_on1_off0 = 0;  
    Analog_Receive[0] = digital_filter(A0);                       //fill in Analog_Receive array before turning on emitters
    Analog_Receive[1] = digital_filter(A1);
    Analog_Receive[2] = digital_filter(A2);
    Analog_Receive[3] = digital_filter(A3);
    Analog_Receive[4] = digital_filter(A4);
    Analog_Receive[5] = digital_filter(A5);
    Read_Analog_Set_Mag_All_VLEDs();                              //Set LED values
    volatile int i_AN=0;
    for(i_AN=0; i_AN<6; i_AN++){
      Analog_Last[i_AN] = Analog[i_AN];                           //save previously recorded values in _Last array
    }
    TX_on1_off0 = 1;  
    digitalWrite(PWM0_EN, HIGH);                                  //turn on all PWMs, wait 1us, take data, turn off all PWMs.
    digitalWrite(PWM1_EN, HIGH); 
    digitalWrite(PWM2_EN, HIGH); 
    digitalWrite(PWM3_EN, HIGH); 
    digitalWrite(PWM4_EN, HIGH); 
    digitalWrite(PWM5_EN, HIGH); 
    delayMicroseconds(1000);
    Analog[0] = digital_filter(A0);
    Analog[1] = digital_filter(A1);
    Analog[2] = digital_filter(A2);
    Analog[3] = digital_filter(A3);
    Analog[4] = digital_filter(A4);
    Analog[5] = digital_filter(A5);
    digitalWrite(PWM0_EN, LOW); 
    digitalWrite(PWM1_EN, LOW); 
    digitalWrite(PWM2_EN, LOW); 
    digitalWrite(PWM3_EN, LOW); 
    digitalWrite(PWM4_EN, LOW); 
    digitalWrite(PWM5_EN, LOW); 
    Read_Analog_Set_Mag_All_VLEDs();
    switch(multipleLEDs1_singleLED0){                               //determine PWM of visible LEDs
      case 0:
        Light_Max_LED_18();
        break;
      case 1:
        Set_Analog_CHs_HWPWM();
        break;  
    }
    i_AN=0;
    for(i_AN=0; i_AN<6; i_AN++){
      Analog_dV_dt[i_AN] = Analog[i_AN] - Analog_Last[i_AN];        //every 100ms determine derivatives
    }
    // start of gesture detect
    if(gesture_reset_counter > 10){                                 //takes one second after reset counter before gestures can occur
      if(gesture_flag == 0){                                        //gesture flag low
        for (i_gesture = 0; i_gesture < 6; i_gesture++){            //cycle through all six channels
          if (Analog_dV_dt[i_gesture] > gesture_threshold){         //derivative must be greater than threshold
            gesture_flag = 1;                                       //set gesture flag high
            gesture_timer = 0;                                      //set gesture timer to zero
            if (i_gesture == 5){                                    //determine next index
              i_plus_gesture = 0;
              i_minus_gesture = 4;
            }
            if (i_gesture == 0){
              i_minus_gesture = 5;
              i_plus_gesture = 1;
            }
            else if (i_gesture == 1 || i_gesture == 2 || i_gesture == 3 || i_gesture == 4){
              i_minus_gesture = i_gesture - 1;
              i_plus_gesture = i_gesture + 1;
            }   
          }
        }
      }
      else{                                                         //gesture flag high
        if(gesture_timer < 5){                                      //if timer has not run out
          if ((Analog_dV_dt[i_plus_gesture]) > gesture_threshold){
            saved_index = i_plus_gesture;                           //save the index
            light_show = 1;                                         //Set flag for CCW LED show
            gesture_flag = 0;                                       //clear gesture flag
          }
          if ((Analog_dV_dt[i_minus_gesture]) > gesture_threshold){
            saved_index = i_minus_gesture;                          //save the index
            light_show = 2;                                         //Set flag for CW LED show
            gesture_flag = 0;                                       //clear gesture flag
          }
          gesture_timer++;                                          //increment timer
        }
        else{
          gesture_flag = 0;                                         //if timer expires, clear gesture flag
        }
      }
    }  
  } 
  clearIntFlag(_TIMER_4_IRQ);                                       //Clear this interrupt flag (Timer 3)  
}

void __attribute__((interrupt)) my1khzCounterISR() {                //Timer 5 Interrupt
  if(bit_number > 0){
    receive_packet = receive_packet | (digitalRead(pintoread) << (bit_number - 1));    //bit shift read value into byte
    bit_number--;
    IFS0CLR = Timer5_mask;                                          //Clear 1khz counter flag (Timer 5)
  }
  else if(bit_number == 0){
    cmdbyte_received = receive_packet >> 24;                        //bit shift to get subset of data
    databyte1_received = ((receive_packet >> 16) & 0xff);
    databyte2_received = ((receive_packet >> 8) & 0xff);
    checksum_received = receive_packet & 0xff;
    if((((cmdbyte_received + databyte1_received + databyte2_received) ^ 0xff) & 0xff) == checksum_received){    //calculate the expected check sum and compare
      successful_packet = ((receive_packet  >> 8) & 0x00ffffff);    //store receive packet minus checksum somewhere else
      successful_cmdbyte = cmdbyte_received;
      successful_databyte1 = databyte1_received;
      successful_databyte2 = databyte2_received;
      if(pintoread == INT0){                                        //display what INT triggered optical comm
        memcpy(interrupt_pin, "INT0", sizeof interrupt_pin);
      }
      else if(pintoread == INT1){
        memcpy(interrupt_pin, "INT1", sizeof interrupt_pin);
      }
      else if(pintoread == INT2){
        memcpy(interrupt_pin, "INT2", sizeof interrupt_pin);
      }
      else if(pintoread == INT3){
        memcpy(interrupt_pin, "INT3", sizeof interrupt_pin);
      }
      else if(pintoread == INT4){                                 //INT4 is actually INT4 and INT5 due to limited INTs
        memcpy(interrupt_pin, "INT4", sizeof interrupt_pin);
      }
      resetcounter = 0;                                           //reset LCD display timer
    }
    IEC0CLR = Timer5_mask;                                        //disable this int (Timer 5)
    IFS0CLR = Ext_Int0_mask;                                      //clear and enable all external interrupts
    IEC0SET = Ext_Int0_mask;
    IFS0CLR = Ext_Int1_mask;
    IEC0SET = Ext_Int1_mask;
    IFS0CLR = Ext_Int2_mask;
    IEC0SET = Ext_Int2_mask;
    IFS0CLR = Ext_Int3_mask;
    IEC0SET = Ext_Int3_mask;
    IFS0CLR = Ext_Int4_mask;
    IEC0SET = Ext_Int4_mask;
  }
}

void __attribute__((interrupt)) myextISR() {                    //External Interrupt routine
  if(((IFS0 >> 3) & 0x00000001) > 0){                           //first, determine which ext int was triggered
    pintoread = INT0;                                           //set correct digital pin to be read
    flagtoclear = Ext_Int0_mask;                                //set correct interrupt flag to be cleared
  }
  else if(((IFS0 >> 7) & 0x00000001) > 0){
    pintoread = INT1;
    flagtoclear = Ext_Int1_mask;
  }
  else if(((IFS0 >> 11) & 0x00000001) > 0){
    pintoread = INT2;
    flagtoclear = Ext_Int2_mask;
  }
  else if(((IFS0 >> 15) & 0x00000001) > 0){
    pintoread = INT3;
    flagtoclear = Ext_Int3_mask;
  }
  else if(((IFS0 >> 19) & 0x00000001) > 0){
    pintoread = INT4;
    flagtoclear = Ext_Int4_mask;
  }
  IFS0SET = Ext_Int0_mask;                                      //disable external INTs from re-triggering
  IEC0CLR = Ext_Int0_mask;
  IFS0SET = Ext_Int1_mask;
  IEC0CLR = Ext_Int1_mask;
  IFS0SET = Ext_Int2_mask;
  IEC0CLR = Ext_Int2_mask;
  IFS0SET = Ext_Int3_mask;
  IEC0CLR = Ext_Int3_mask;
  IFS0SET = Ext_Int4_mask;
  IEC0CLR = Ext_Int4_mask;
  bit_number = 32;                                              //initialize index number
  TMR3 = 0;                                                     //clear timer 3 (10kHz) unsure if needed
  timer_hold = counter;
  while(counter < timer_hold + 10){                             //10 = 1000us
  }
  first_read = digitalRead(pintoread);                          //read interrupt pin value
  TMR3 = 0;                                                     //clear 10kHz timer
  timer_hold = counter;
  while(counter < timer_hold + 6){                              //6 = 600us
  }
  second_read = digitalRead(pintoread);                         //read interrupt pin value
  if(first_read && !second_read == 1){                          //first_read == 1 and second_read == 0 (unique start bit)
    TMR3 = 0;  
    timer_hold = counter;
    while(counter < timer_hold + 5){                            //total delay before 1kHz timer kicks in is 2.1ms (21 10kHz counts)
    }  
    receive_packet = 0x00000000;                                //receive packet is back to 0
    TMR5 = 0;                                                   //clear Timer 5 counter
    IFS0CLR = Timer5_mask;                                      //clear flag for 1khz counter isr (Timer 5)
    IEC0SET = Timer5_mask;                                      //enable 1khz counter isr (Timer 5)
  }
  else{
    IFS0CLR = Ext_Int0_mask;                                    //clear and enable all external interrupts
    IEC0SET = Ext_Int0_mask;
    IFS0CLR = Ext_Int1_mask;
    IEC0SET = Ext_Int1_mask;
    IFS0CLR = Ext_Int2_mask;
    IEC0SET = Ext_Int2_mask;
    IFS0CLR = Ext_Int3_mask;
    IEC0SET = Ext_Int3_mask;
    IFS0CLR = Ext_Int4_mask;
    IEC0SET = Ext_Int4_mask;
  }
}

void setup() {
  for(i=0;i<8;i++) {Analog[i]=0;}                           //initialize analog variables to 0
  for(i=0;i<8;i++) {Analog_Last[i]=0;}
  for(i=0;i<8;i++) {Analog_dV_dt[i]=0;}
  pinMode(PWM0_EN, OUTPUT);                                 //initialize these pins as outputs
  pinMode(PWM1_EN, OUTPUT);
  pinMode(PWM2_EN, OUTPUT);
  pinMode(PWM3_EN, OUTPUT);
  pinMode(PWM4_EN, OUTPUT);
  pinMode(PWM5_EN, OUTPUT);
  pinMode(RF1, OUTPUT);                                     //RF1 is used for debugging
  digitalWrite(RF1, LOW);               
  digitalWrite(PWM0_EN, LOW);                               //do not emit IR at start
  digitalWrite(PWM1_EN, LOW);                               
  digitalWrite(PWM2_EN, LOW);                           
  digitalWrite(PWM3_EN, LOW);   
  digitalWrite(PWM4_EN, LOW);   
  digitalWrite(PWM5_EN, LOW);   
  randomSeed(analogRead(A11));                              //initialize random number differently each time. A11 is unused. 
  
  Serial.begin(115200); 
  pwm.begin();                                              //used for HW PWM
  pwm.setPWMFreq(100);                                      //100 Hz used to avoid interference with 10kHz filter
//  I2C1BRG = 0x033;                                          //671kHz I2C freq (max)

  start_timer_2(95);                                  //95Hz (used for VisibleLED16 and 17)
  
  start_timer_1(12);                                  //12 Hz (Visual display timer)
  setIntVector(_TIMER_1_VECTOR, VisualDisplayISR);    //Set Timer 1 IRQ to VisualDisplayISR
  setIntPriority(_TIMER_1_VECTOR, 2, 0);              //12 HZ VisualDisplayISR gets priority 2
  clearIntFlag(_TIMER_1_IRQ);                         //Clear VisualDisplayISR flag
  
  start_timer_3(10000);                               //10 kHz (counter timer)
  setIntVector(_TIMER_3_VECTOR, myCounterISR);        //Set Timer 3 IRQ to MyCounterISR
  setIntPriority(_TIMER_3_VECTOR, 6, 0);              //10 khZ MyCounterISR gets priority 6
  clearIntFlag(_TIMER_3_IRQ);                         //Clear MyCounterISR flag

  start_timer_4(10);                                  //10 Hz (but changes randomly)
  setIntVector(_TIMER_4_VECTOR, EmitterISR);          //Set Timer 4 IRQ to EmitterISR
  setIntPriority(_TIMER_4_VECTOR, 3, 0);              //10 HZ EmitterISR gets priority 3
  clearIntFlag(_TIMER_4_IRQ);                         //Clear EmitterISR flag
  
  start_timer_5(1000);                                //1 kHz (used for optical comm)
  setIntVector(_TIMER_5_VECTOR, my1khzCounterISR);    //Set Timer 5 IRQ to my1khzCounterISR
  setIntPriority(_TIMER_5_VECTOR, 7, 0);              //1 khZ my1khzCounterISR gets priority 7
  clearIntFlag(_TIMER_5_IRQ);                         //Clear my1khzCounterISR flag
  
  setIntEnable(_TIMER_1_IRQ);                         //Enable 12Hz timer
  setIntEnable(_TIMER_3_IRQ);                         //Enable 10kHz timer
  setIntEnable(_TIMER_4_IRQ);                         //Enable 10Hz (random) timer

  //  setup external interrupts
  IEC0CLR = Ext_Int0_mask;                      //disable external INT0
  IEC0CLR = Ext_Int1_mask;                      //disable external INT1
  IEC0CLR = Ext_Int2_mask;                      //disable external INT2
  IEC0CLR = Ext_Int3_mask;                      //disable external INT3
  IEC0CLR = Ext_Int4_mask;                      //disable external INT4
  INTCONCLR = 0x1F;                             //Clear rising edge trigger bits
  INTCONSET = 0x1F;                             //Set all to rising edge trigger
  
// setup Ext INT0
  setIntVector(_EXTERNAL_0_VECTOR, myextISR);   //set interrupt vector to myextisr
  IPC0CLR = 0x0F000000;                         //Clear priority bits
  IPC0SET = 0x0F000000;                         //Set priority to 3, sub-priority to 3
  IFS0CLR = Ext_Int0_mask;                      //Clear interrupt flag
  IEC0SET = Ext_Int0_mask;                      //enable INT0

// setup Ext INT1
  setIntVector(_EXTERNAL_1_VECTOR, myextISR);   //set interrupt vector to myextisr
  IPC1CLR = 0x0F000000;                         //Clear priority bits
  IPC1SET = 0x0F000000;                         //Set priority to 3, sub-priority to 3
  IFS0CLR = Ext_Int1_mask;                      //Clear interrupt flag
  IEC0SET = Ext_Int1_mask;                      //enable INT1

// setup Ext INT2
  setIntVector(_EXTERNAL_2_VECTOR, myextISR);   //set interrupt vector to myextisr
  IPC2CLR = 0x0F000000;                         //Clear priority bits
  IPC2SET = 0x0F000000;                         //Set priority to 3, sub-priority to 3
  IFS0CLR = Ext_Int2_mask;                      //Clear interrupt flag
  IEC0SET = Ext_Int2_mask;                      //enable INT2

// setup Ext INT3
  setIntVector(_EXTERNAL_3_VECTOR, myextISR);   //set interrupt vector to myextisr
  IPC3CLR = 0x0F000000;                         //Clear priority bits
  IPC3SET = 0x0F000000;                         //Set priority to 3, sub-priority to 3
  IFS0CLR = Ext_Int3_mask;                      //Clear interrupt flag
  IEC0SET = Ext_Int3_mask;                      //enable INT3

// setup Ext INT4
  setIntVector(_EXTERNAL_4_VECTOR, myextISR);   //set interrupt vector to myextisr
  IPC4CLR = 0x0F000000;                         //Clear priority bits
  IPC4SET = 0x0F000000;                         //Set priority to 3, sub-priority to 3
  IFS0CLR = Ext_Int4_mask;                      //Clear interrupt flag
  IEC0SET = Ext_Int4_mask;                      //enable INT4

//  setup OC1 (PWM output) to follow timer 3 10kHz signal  
  OC1CON = 0x0000;                              // Turn off the OC1 when performing the setup
  OC1R = 0x00000FA0;                            // Initialize primary Compare register (duty cycle set)
  OC1RS = 0x00000FA0;                           // Initialize secondary Compare register (same as above)
  OC1CON = 0x000E;                              // Configure for PWM mode without Fault pin enabled, Timer 3 is clock source
  OC1CONSET = 0x8000;                           // Enable OC1 in 16-bit mode.

//  setup OC2 (Visible LED16) to follow timer 2 1kHz signal  
  OC2CON = 0x0000;                              // Turn off the OC2 when performing the setup
  OC2R = 0x00000000;                            // Initialize primary Compare register (duty cycle set) 
  OC2RS = 0x00000000;                           // Initialize secondary Compare register (same as above)
  OC2CON = 0x0006;                              // Configure for PWM mode without Fault pin enabled, Timer 2 is clock source
  OC2CONSET = 0x8000;                           // Enable OC2 in 16-bit mode.

//  setup OC3 (Visible LED17) to follow timer 2 1kHz signal  
  OC3CON = 0x0000;                              // Turn off the OC3 when performing the setup
  OC3R = 0x00000000;                            // Initialize primary Compare register (duty cycle set) 
  OC3RS = 0x00000000;                           // Initialize secondary Compare register (same as above)
  OC3CON = 0x0006;                              // Configure for PWM mode without Fault pin enabled, Timer 2 is clock source
  OC3CONSET = 0x8000;                           // Enable OC3 in 16-bit mode.
}

void loop() {
  Main_Program_Demo();
  }

void Main_Program_Demo(void){
  IOShieldOled.begin();                                                                              
  IOShieldOled.clearBuffer();
  gesture_reset_counter = 0;                                //reset this counter to prevent false gesturing from ocurring
  light_show = 0;                                           //gesture not detected
  multipleLEDs1_singleLED0 = 1;                             //I prefer multiple LEDs lit                             
  obstacledetect1_vector0 = 0;                              //vector looks more aesthetic
  for(;;){                
    switch(light_show){
      case 0:                                               //do nothing, everything is in INTs
        break;
      case 1:
        Wire.endTransmission();
        CCW_Visible_LEDs_index(saved_index);                //do a CCW LED show
        light_show = 0;
        break;
      case 2:
        Wire.endTransmission();
        CW_Visible_LEDs_index(saved_index);                 //do a CW LED show
        light_show = 0;
        break;
      case 3:                                               //Case 3 will never be entered, but left in                                                 
        Wire.endTransmission();
        OC2RS = 0x00000000;                                 //do a light blink before every transmit of data
        OC3RS = 0x00000000;
        for(int i=0; i<16; i++) pwm.setPWM(i, 0, 0);
        delay(20);
        OC2RS = 0x0000ABA9;                
        OC3RS = 0x0000ABA9;
        for(int i=0; i<16; i++) pwm.setPWM(i, 0, 3500);
        delay(20);
        OC2RS = 0x00000000;                         
        OC3RS = 0x00000000;
        for(int i=0; i<16; i++) pwm.setPWM(i, 0, 0);
        light_show = 0;
        break;
    }
  }
}

void Read_Analog_Set_Mag_All_VLEDs(void){                       //This function reads the analog channels and sets the visible LED
    switch(TX_on1_off0){
      case 0:                                                   //transmitters are off
        VisibleLED_Receive[0] = Analog_Receive[0];
        VisibleLED_Receive[3] = Analog_Receive[5];
        VisibleLED_Receive[6] = Analog_Receive[4];
        VisibleLED_Receive[9] = Analog_Receive[3];
        VisibleLED_Receive[12] = Analog_Receive[2];
        VisibleLED_Receive[15] = Analog_Receive[1];
        if (VisibleLED_Receive[0] <= A0baseline) VisibleLED_Receive[0] = 0;                                       //If less than noise floor, reading is zero
        else VisibleLED_Receive[0] = (VisibleLED_Receive[0] - A0baseline)*100 / (990 - A0baseline);               //If greater than noise floor, scale 0 - 100
        if (VisibleLED_Receive[3] <= A5baseline) VisibleLED_Receive[3] = 0;                               
        else VisibleLED_Receive[3] = (VisibleLED_Receive[3] - A5baseline)*100 / (990 - A5baseline);       
        if (VisibleLED_Receive[6] <= A4baseline) VisibleLED_Receive[6] = 0;                               
        else VisibleLED_Receive[6] = (VisibleLED_Receive[6] - A4baseline)*100 / (990 - A4baseline);       
        if (VisibleLED_Receive[9] <= A3baseline) VisibleLED_Receive[9] = 0;                               
        else VisibleLED_Receive[9] = (VisibleLED_Receive[9] - A3baseline)*100 / (990 - A3baseline);       
        if (VisibleLED_Receive[12] <= A2baseline) VisibleLED_Receive[12] = 0;                             
        else VisibleLED_Receive[12] = (VisibleLED_Receive[12] - A2baseline)*100 / (990 - A2baseline);     
        if (VisibleLED_Receive[15] <= A1baseline) VisibleLED_Receive[15] = 0;                             
        else VisibleLED_Receive[15] = (VisibleLED_Receive[15] - A1baseline)*100 / (990 - A1baseline);
        A0reading_xcomp = 0;                                                           //Angle is zero, no x-component
        A0reading_ycomp = VisibleLED_Receive[0];                                       //Angle is zero, reading is y-component
        A5reading_xcomp = VisibleLED_Receive[3] * sin(A5angle);                        //calculate x-component of reading based on angle
        A5reading_ycomp = VisibleLED_Receive[3] * cos(A5angle);                        //calculate y-component of reading based on angle
        A4reading_xcomp = VisibleLED_Receive[6] * sin(A4angle);
        A4reading_ycomp = VisibleLED_Receive[6] * cos(A4angle);
        A1reading_xcomp = VisibleLED_Receive[15] * sin(A1angle);
        A1reading_ycomp = VisibleLED_Receive[15] * cos(A1angle);    
        A2reading_xcomp = VisibleLED_Receive[12] * sin(A2angle);
        A2reading_ycomp = VisibleLED_Receive[12] * cos(A2angle);
        A3reading_xcomp = 0;
        A3reading_ycomp = VisibleLED_Receive[9] * -1;
        break;
      case 1:                                                                           //transmitters are on
        VisibleLED[0] = Analog[0];
        VisibleLED[3] = Analog[5];
        VisibleLED[6] = Analog[4];
        VisibleLED[9] = Analog[3];
        VisibleLED[12] = Analog[2];
        VisibleLED[15] = Analog[1];
        if (VisibleLED[0] <= A0TXbaseline) VisibleLED[0] = 0;                                      //If less than noise floor, reading is zero
        else VisibleLED[0] = (VisibleLED[0] - A0TXbaseline)*100 / (990 - A0TXbaseline);            //If greater than noise floor, scale 0 - 100
        if (VisibleLED[3] <= A5TXbaseline) VisibleLED[3] = 0;                                      
        else VisibleLED[3] = (VisibleLED[3] - A5TXbaseline)*100 / (990 - A5TXbaseline);            
        if (VisibleLED[6] <= A4TXbaseline) VisibleLED[6] = 0;                                      
        else VisibleLED[6] = (VisibleLED[6] - A4TXbaseline)*100 / (990 - A4TXbaseline);            
        if (VisibleLED[9] <= A3TXbaseline) VisibleLED[9] = 0;                                      
        else VisibleLED[9] = (VisibleLED[9] - A3TXbaseline)*100 / (990 - A3TXbaseline);            
        if (VisibleLED[12] <= A2TXbaseline) VisibleLED[12] = 0;                                    
        else VisibleLED[12] = (VisibleLED[12] - A2TXbaseline)*100 / (990 - A2TXbaseline);          
        if (VisibleLED[15] <= A1TXbaseline) VisibleLED[15] = 0;                                    
        else VisibleLED[15] = (VisibleLED[15] - A1TXbaseline)*100 / (990 - A1TXbaseline);   
        A0reading_xcomp = 0;                                                   //Angle is zero, no x-component
        A0reading_ycomp = VisibleLED[0];                                       //Angle is zero, reading is y-component
        A5reading_xcomp = VisibleLED[3] * sin(A5angle);                        //calculate x-component of reading based on angle
        A5reading_ycomp = VisibleLED[3] * cos(A5angle);                        //calculate y-component of reading based on angle
        A4reading_xcomp = VisibleLED[6] * sin(A4angle);
        A4reading_ycomp = VisibleLED[6] * cos(A4angle);
        A1reading_xcomp = VisibleLED[15] * sin(A1angle);
        A1reading_ycomp = VisibleLED[15] * cos(A1angle);    
        A2reading_xcomp = VisibleLED[12] * sin(A2angle);
        A2reading_ycomp = VisibleLED[12] * cos(A2angle);
        A3reading_xcomp = 0;
        A3reading_ycomp = VisibleLED[9] * -1;       
        break;
    }
    double x_sum_A0_A5 = A0reading_xcomp + A5reading_xcomp;                     //calculate x-sum between two channels
    double x_sum_A5_A4 = A5reading_xcomp + A4reading_xcomp;
    double x_sum_A4_A3 = A4reading_xcomp + A3reading_xcomp;
    double x_sum_A3_A2 = A3reading_xcomp + A2reading_xcomp;
    double x_sum_A2_A1 = A2reading_xcomp + A1reading_xcomp;
    double x_sum_A1_A0 = A1reading_xcomp + A0reading_xcomp;
    double y_sum_A0_A5 = A0reading_ycomp + A5reading_ycomp;                     //calculate y-sum between two channels
    double y_sum_A5_A4 = A5reading_ycomp + A4reading_ycomp;
    double y_sum_A4_A3 = A4reading_ycomp + A3reading_ycomp;
    double y_sum_A3_A2 = A3reading_ycomp + A2reading_ycomp;
    double y_sum_A2_A1 = A2reading_ycomp + A1reading_ycomp;
    double y_sum_A1_A0 = A1reading_ycomp + A0reading_ycomp;
    double vector_A0_A5 = (atan2(x_sum_A0_A5, y_sum_A0_A5))*180 / 3.1415;       //calculate vector between two channels
    double vector_A5_A4 = (atan2(x_sum_A5_A4, y_sum_A5_A4))*180 / 3.1415;
    double vector_A4_A3 = (atan2(x_sum_A4_A3, y_sum_A4_A3))*180 / 3.1415;
    double vector_A3_A2 = (6.283 - abs(atan2(x_sum_A3_A2, y_sum_A3_A2)))*180 / 3.1415;
    double vector_A2_A1 = (6.283 - abs(atan2(x_sum_A2_A1, y_sum_A2_A1)))*180 / 3.1415;
    double vector_A1_A0 = (6.283 - abs(atan2(x_sum_A1_A0, y_sum_A1_A0)))*180 / 3.1415;
    double pythag_A0_A5 = sqrt(x_sum_A0_A5*x_sum_A0_A5 + y_sum_A0_A5*y_sum_A0_A5);                          //calculate pythagorean theorem based on x and y components
    double pythag_A5_A4 = sqrt(x_sum_A5_A4*x_sum_A5_A4 + y_sum_A5_A4*y_sum_A5_A4);                          //calculate pythagorean theorem based on x and y components
    double pythag_A4_A3 = sqrt(x_sum_A4_A3*x_sum_A4_A3 + y_sum_A4_A3*y_sum_A4_A3);                          //calculate pythagorean theorem based on x and y components
    double pythag_A3_A2 = sqrt(x_sum_A3_A2*x_sum_A3_A2 + y_sum_A3_A2*y_sum_A3_A2);                          //calculate pythagorean theorem based on x and y components
    double pythag_A2_A1 = sqrt(x_sum_A2_A1*x_sum_A2_A1 + y_sum_A2_A1*y_sum_A2_A1);                          //calculate pythagorean theorem based on x and y components
    double pythag_A1_A0 = sqrt(x_sum_A1_A0*x_sum_A1_A0 + y_sum_A1_A0*y_sum_A1_A0);                          //calculate pythagorean theorem based on x and y components
    switch(TX_on1_off0){
      case 0:
        x_sum = A0reading_xcomp + A5reading_xcomp + A4reading_xcomp + A3reading_xcomp + A2reading_xcomp + A1reading_xcomp;
        y_sum = A0reading_ycomp + A5reading_ycomp + A4reading_ycomp + A3reading_ycomp + A2reading_ycomp + A1reading_ycomp;
        if (x_sum > 0){
          vector = atan2(x_sum, y_sum);                                           //calculate angle from 0 - 360
        }
        if (x_sum < 0){
          vector = 6.283 - abs(atan2(x_sum, y_sum));
        }
        if (abs(x_sum) < 5 && abs(y_sum) < 5) vector = 0.0;
        vector_degrees = vector * 180 / 3.1415;                                   //convert from radians to degrees
        vector_degrees_int = (int)vector_degrees;
        sprintf(vector_array, "%i", vector_degrees_int);                          //display vector angle as an int  
        if (vector_A0_A5 <= 20){
          VisibleLED_Receive[1] = pythag_A0_A5 - ((abs(vector_A0_A5 - 20) / abs(vector_A0_A5 - 60)) * abs(pythag_A0_A5 - VisibleLED_Receive[3]));   //calculate linearly
          VisibleLED_Receive[2] = pythag_A0_A5 - ((abs(vector_A0_A5 - 40) / abs(vector_A0_A5 - 60)) * abs(pythag_A0_A5 - VisibleLED_Receive[3]));
        }
        if (vector_A0_A5 > 20 && vector_A0_A5 <= 40){
          VisibleLED_Receive[1] = pythag_A0_A5 - ((abs(vector_A0_A5 - 20) / abs(vector_A0_A5 - 0)) * abs(pythag_A0_A5 - VisibleLED_Receive[0]));
          VisibleLED_Receive[2] = pythag_A0_A5 - ((abs(vector_A0_A5 - 40) / abs(vector_A0_A5 - 60)) * abs(pythag_A0_A5 - VisibleLED_Receive[3]));
        }
        if (vector_A0_A5 > 40 && vector_A0_A5 <= 61){                             //overlap is necessary due to rounding errors
          VisibleLED_Receive[1] = pythag_A0_A5 - ((abs(vector_A0_A5 - 20) / abs(vector_A0_A5 - 0)) * abs(pythag_A0_A5 - VisibleLED_Receive[0]));
          VisibleLED_Receive[2] = pythag_A0_A5 - ((abs(vector_A0_A5 - 40) / abs(vector_A0_A5 - 0)) * abs(pythag_A0_A5 - VisibleLED_Receive[0]));
        } 
        if (vector_A5_A4 > 59 && vector_A5_A4 <= 80){
          VisibleLED_Receive[4] = pythag_A5_A4 - ((abs(vector_A5_A4 - 80) / abs(vector_A5_A4 - 120)) * abs(pythag_A5_A4 - VisibleLED_Receive[6]));
          VisibleLED_Receive[5] = pythag_A5_A4 - ((abs(vector_A5_A4 - 100) / abs(vector_A5_A4 - 120)) * abs(pythag_A5_A4 - VisibleLED_Receive[6]));
        }
        if (vector_A5_A4 > 80 && vector_A5_A4 <= 100){
          VisibleLED_Receive[4] = pythag_A5_A4 - ((abs(vector_A5_A4 - 80) / abs(vector_A5_A4 - 60)) * abs(pythag_A5_A4 - VisibleLED_Receive[3]));
          VisibleLED_Receive[5] = pythag_A5_A4 - ((abs(vector_A5_A4 - 100) / abs(vector_A5_A4 - 120)) * abs(pythag_A5_A4 - VisibleLED_Receive[6]));
        }
        if (vector_A5_A4 > 100 && vector_A5_A4 <= 121){
          VisibleLED_Receive[4] = pythag_A5_A4 - ((abs(vector_A5_A4 - 80) / abs(vector_A5_A4 - 60)) * abs(pythag_A5_A4 - VisibleLED_Receive[3]));
          VisibleLED_Receive[5] = pythag_A5_A4 - ((abs(vector_A5_A4 - 100) / abs(vector_A5_A4 - 60)) * abs(pythag_A5_A4 - VisibleLED_Receive[3]));
        } 
        if (vector_A4_A3 > 119 && vector_A4_A3 <= 140){
          VisibleLED_Receive[7] = pythag_A4_A3 - ((abs(vector_A4_A3 - 140) / abs(vector_A4_A3 - 180)) * abs(pythag_A4_A3 - VisibleLED_Receive[9]));
          VisibleLED_Receive[8] = pythag_A4_A3 - ((abs(vector_A4_A3 - 160) / abs(vector_A4_A3 - 180)) * abs(pythag_A4_A3 - VisibleLED_Receive[9]));
        }
        if (vector_A4_A3 > 140 && vector_A4_A3 <= 160){
          VisibleLED_Receive[7] = pythag_A4_A3 - ((abs(vector_A4_A3 - 140) / abs(vector_A4_A3 - 120)) * abs(pythag_A4_A3 - VisibleLED_Receive[6]));
          VisibleLED_Receive[8] = pythag_A4_A3 - ((abs(vector_A4_A3 - 160) / abs(vector_A4_A3 - 180)) * abs(pythag_A4_A3 - VisibleLED_Receive[9]));
        }
        if (vector_A4_A3 > 160 && vector_A4_A3 <= 181){
          VisibleLED_Receive[7] = pythag_A4_A3 - ((abs(vector_A4_A3 - 140) / abs(vector_A4_A3 - 120)) * abs(pythag_A4_A3 - VisibleLED_Receive[6]));
          VisibleLED_Receive[8] = pythag_A4_A3 - ((abs(vector_A4_A3 - 160) / abs(vector_A4_A3 - 120)) * abs(pythag_A4_A3 - VisibleLED_Receive[6]));
        }
        if (vector_A3_A2 > 179 && vector_A3_A2 <= 200){
          VisibleLED_Receive[10] = pythag_A3_A2 - ((abs(vector_A3_A2 - 200) / abs(vector_A3_A2 - 240)) * abs(pythag_A3_A2 - VisibleLED_Receive[12]));
          VisibleLED_Receive[11] = pythag_A3_A2 - ((abs(vector_A3_A2 - 220) / abs(vector_A3_A2 - 240)) * abs(pythag_A3_A2 - VisibleLED_Receive[12]));
        }
        if (vector_A3_A2 > 200 && vector_A3_A2 <= 220){
          VisibleLED_Receive[10] = pythag_A3_A2 - ((abs(vector_A3_A2 - 200) / abs(vector_A3_A2 - 180)) * abs(pythag_A3_A2 - VisibleLED_Receive[9]));
          VisibleLED_Receive[11] = pythag_A3_A2 - ((abs(vector_A3_A2 - 220) / abs(vector_A3_A2 - 240)) * abs(pythag_A3_A2 - VisibleLED_Receive[12]));
        }
        if (vector_A3_A2 > 220 && vector_A3_A2 <= 241){
          VisibleLED_Receive[10] = pythag_A3_A2 - ((abs(vector_A3_A2 - 200) / abs(vector_A3_A2 - 180)) * abs(pythag_A3_A2 - VisibleLED_Receive[9]));
          VisibleLED_Receive[11] = pythag_A3_A2 - ((abs(vector_A3_A2 - 220) / abs(vector_A3_A2 - 180)) * abs(pythag_A3_A2 - VisibleLED_Receive[9]));
        }
        if (vector_A2_A1 > 239 && vector_A2_A1 <= 260){
          VisibleLED_Receive[13] = pythag_A2_A1 - ((abs(vector_A2_A1 - 260) / abs(vector_A2_A1 - 300)) * abs(pythag_A2_A1 - VisibleLED_Receive[15]));
          VisibleLED_Receive[14] = pythag_A2_A1 - ((abs(vector_A2_A1 - 280) / abs(vector_A2_A1 - 300)) * abs(pythag_A2_A1 - VisibleLED_Receive[15]));
        }
        if (vector_A2_A1 > 260 && vector_A2_A1 <= 280){
          VisibleLED_Receive[13] = pythag_A2_A1 - ((abs(vector_A2_A1 - 260) / abs(vector_A2_A1 - 240)) * abs(pythag_A2_A1 - VisibleLED_Receive[12]));
          VisibleLED_Receive[14] = pythag_A2_A1 - ((abs(vector_A2_A1 - 280) / abs(vector_A2_A1 - 300)) * abs(pythag_A2_A1 - VisibleLED_Receive[15]));
        }
        if (vector_A2_A1 > 280 && vector_A2_A1 <= 301){
          VisibleLED_Receive[13] = pythag_A2_A1 - ((abs(vector_A2_A1 - 260) / abs(vector_A2_A1 - 240)) * abs(pythag_A2_A1 - VisibleLED_Receive[12]));
          VisibleLED_Receive[14] = pythag_A2_A1 - ((abs(vector_A2_A1 - 280) / abs(vector_A2_A1 - 240)) * abs(pythag_A2_A1 - VisibleLED_Receive[12]));
        }
        if (vector_A1_A0 > 299 && vector_A1_A0 <= 320){
          VisibleLED_Receive[16] = pythag_A1_A0 - ((abs(vector_A1_A0 - 320) / abs(vector_A1_A0 - 360)) * abs(pythag_A1_A0 - VisibleLED_Receive[0]));
          VisibleLED_Receive[17] = pythag_A1_A0 - ((abs(vector_A1_A0 - 340) / abs(vector_A1_A0 - 360)) * abs(pythag_A1_A0 - VisibleLED_Receive[0]));
        }
        if (vector_A1_A0 > 320 && vector_A1_A0 <= 340){
          VisibleLED_Receive[16] = pythag_A1_A0 - ((abs(vector_A1_A0 - 320) / abs(vector_A1_A0 - 300)) * abs(pythag_A1_A0 - VisibleLED_Receive[15]));
          VisibleLED_Receive[17] = pythag_A1_A0 - ((abs(vector_A1_A0 - 340) / abs(vector_A1_A0 - 360)) * abs(pythag_A1_A0 - VisibleLED_Receive[0]));
        }
        if (vector_A1_A0 > 340 && vector_A1_A0 <= 361){
          VisibleLED_Receive[16] = pythag_A1_A0 - ((abs(vector_A1_A0 - 320) / abs(vector_A1_A0 - 300)) * abs(pythag_A1_A0 - VisibleLED_Receive[15]));
          VisibleLED_Receive[17] = pythag_A1_A0 - ((abs(vector_A1_A0 - 340) / abs(vector_A1_A0 - 300)) * abs(pythag_A1_A0 - VisibleLED_Receive[15]));
        }
      break;
      case 1:
        if (vector_A0_A5 <= 20){
          VisibleLED[1] = pythag_A0_A5 - ((abs(vector_A0_A5 - 20) / abs(vector_A0_A5 - 60)) * abs(pythag_A0_A5 - VisibleLED[3]));
          VisibleLED[2] = pythag_A0_A5 - ((abs(vector_A0_A5 - 40) / abs(vector_A0_A5 - 60)) * abs(pythag_A0_A5 - VisibleLED[3]));
        }
        if (vector_A0_A5 > 20 && vector_A0_A5 <= 40){
          VisibleLED[1] = pythag_A0_A5 - ((abs(vector_A0_A5 - 20) / abs(vector_A0_A5 - 0)) * abs(pythag_A0_A5 - VisibleLED[0]));
          VisibleLED[2] = pythag_A0_A5 - ((abs(vector_A0_A5 - 40) / abs(vector_A0_A5 - 60)) * abs(pythag_A0_A5 - VisibleLED[3]));
        }
        if (vector_A0_A5 > 40 && vector_A0_A5 <= 61){
          VisibleLED[1] = pythag_A0_A5 - ((abs(vector_A0_A5 - 20) / abs(vector_A0_A5 - 0)) * abs(pythag_A0_A5 - VisibleLED[0]));
          VisibleLED[2] = pythag_A0_A5 - ((abs(vector_A0_A5 - 40) / abs(vector_A0_A5 - 0)) * abs(pythag_A0_A5 - VisibleLED[0]));
        } 
        if (vector_A5_A4 > 59 && vector_A5_A4 <= 80){
          VisibleLED[4] = pythag_A5_A4 - ((abs(vector_A5_A4 - 80) / abs(vector_A5_A4 - 120)) * abs(pythag_A5_A4 - VisibleLED[6]));
          VisibleLED[5] = pythag_A5_A4 - ((abs(vector_A5_A4 - 100) / abs(vector_A5_A4 - 120)) * abs(pythag_A5_A4 - VisibleLED[6]));
        }
        if (vector_A5_A4 > 80 && vector_A5_A4 <= 100){
          VisibleLED[4] = pythag_A5_A4 - ((abs(vector_A5_A4 - 80) / abs(vector_A5_A4 - 60)) * abs(pythag_A5_A4 - VisibleLED[3]));
          VisibleLED[5] = pythag_A5_A4 - ((abs(vector_A5_A4 - 100) / abs(vector_A5_A4 - 120)) * abs(pythag_A5_A4 - VisibleLED[6]));
        }
        if (vector_A5_A4 > 100 && vector_A5_A4 <= 121){
          VisibleLED[4] = pythag_A5_A4 - ((abs(vector_A5_A4 - 80) / abs(vector_A5_A4 - 60)) * abs(pythag_A5_A4 - VisibleLED[3]));
          VisibleLED[5] = pythag_A5_A4 - ((abs(vector_A5_A4 - 100) / abs(vector_A5_A4 - 60)) * abs(pythag_A5_A4 - VisibleLED[3]));
        } 
        if (vector_A4_A3 > 119 && vector_A4_A3 <= 140){
          VisibleLED[7] = pythag_A4_A3 - ((abs(vector_A4_A3 - 140) / abs(vector_A4_A3 - 180)) * abs(pythag_A4_A3 - VisibleLED[9]));
          VisibleLED[8] = pythag_A4_A3 - ((abs(vector_A4_A3 - 160) / abs(vector_A4_A3 - 180)) * abs(pythag_A4_A3 - VisibleLED[9]));
        }
        if (vector_A4_A3 > 140 && vector_A4_A3 <= 160){
          VisibleLED[7] = pythag_A4_A3 - ((abs(vector_A4_A3 - 140) / abs(vector_A4_A3 - 120)) * abs(pythag_A4_A3 - VisibleLED[6]));
          VisibleLED[8] = pythag_A4_A3 - ((abs(vector_A4_A3 - 160) / abs(vector_A4_A3 - 180)) * abs(pythag_A4_A3 - VisibleLED[9]));
        }
        if (vector_A4_A3 > 160 && vector_A4_A3 <= 181){
          VisibleLED[7] = pythag_A4_A3 - ((abs(vector_A4_A3 - 140) / abs(vector_A4_A3 - 120)) * abs(pythag_A4_A3 - VisibleLED[6]));
          VisibleLED[8] = pythag_A4_A3 - ((abs(vector_A4_A3 - 160) / abs(vector_A4_A3 - 120)) * abs(pythag_A4_A3 - VisibleLED[6]));
        }
        if (vector_A3_A2 > 179 && vector_A3_A2 <= 200){
          VisibleLED[10] = pythag_A3_A2 - ((abs(vector_A3_A2 - 200) / abs(vector_A3_A2 - 240)) * abs(pythag_A3_A2 - VisibleLED[12]));
          VisibleLED[11] = pythag_A3_A2 - ((abs(vector_A3_A2 - 220) / abs(vector_A3_A2 - 240)) * abs(pythag_A3_A2 - VisibleLED[12]));
        }
        if (vector_A3_A2 > 200 && vector_A3_A2 <= 220){
          VisibleLED[10] = pythag_A3_A2 - ((abs(vector_A3_A2 - 200) / abs(vector_A3_A2 - 180)) * abs(pythag_A3_A2 - VisibleLED[9]));
          VisibleLED[11] = pythag_A3_A2 - ((abs(vector_A3_A2 - 220) / abs(vector_A3_A2 - 240)) * abs(pythag_A3_A2 - VisibleLED[12]));
        }
        if (vector_A3_A2 > 220 && vector_A3_A2 <= 241){
          VisibleLED[10] = pythag_A3_A2 - ((abs(vector_A3_A2 - 200) / abs(vector_A3_A2 - 180)) * abs(pythag_A3_A2 - VisibleLED[9]));
          VisibleLED[11] = pythag_A3_A2 - ((abs(vector_A3_A2 - 220) / abs(vector_A3_A2 - 180)) * abs(pythag_A3_A2 - VisibleLED[9]));
        }
        if (vector_A2_A1 > 239 && vector_A2_A1 <= 260){
          VisibleLED[13] = pythag_A2_A1 - ((abs(vector_A2_A1 - 260) / abs(vector_A2_A1 - 300)) * abs(pythag_A2_A1 - VisibleLED[15]));
          VisibleLED[14] = pythag_A2_A1 - ((abs(vector_A2_A1 - 280) / abs(vector_A2_A1 - 300)) * abs(pythag_A2_A1 - VisibleLED[15]));
        }
        if (vector_A2_A1 > 260 && vector_A2_A1 <= 280){
          VisibleLED[13] = pythag_A2_A1 - ((abs(vector_A2_A1 - 260) / abs(vector_A2_A1 - 240)) * abs(pythag_A2_A1 - VisibleLED[12]));
          VisibleLED[14] = pythag_A2_A1 - ((abs(vector_A2_A1 - 280) / abs(vector_A2_A1 - 300)) * abs(pythag_A2_A1 - VisibleLED[15]));
        }
        if (vector_A2_A1 > 280 && vector_A2_A1 <= 301){
          VisibleLED[13] = pythag_A2_A1 - ((abs(vector_A2_A1 - 260) / abs(vector_A2_A1 - 240)) * abs(pythag_A2_A1 - VisibleLED[12]));
          VisibleLED[14] = pythag_A2_A1 - ((abs(vector_A2_A1 - 280) / abs(vector_A2_A1 - 240)) * abs(pythag_A2_A1 - VisibleLED[12]));
        }
        if (vector_A1_A0 > 299 && vector_A1_A0 <= 320){
          VisibleLED[16] = pythag_A1_A0 - ((abs(vector_A1_A0 - 320) / abs(vector_A1_A0 - 360)) * abs(pythag_A1_A0 - VisibleLED[0]));
          VisibleLED[17] = pythag_A1_A0 - ((abs(vector_A1_A0 - 340) / abs(vector_A1_A0 - 360)) * abs(pythag_A1_A0 - VisibleLED[0]));
        }
        if (vector_A1_A0 > 320 && vector_A1_A0 <= 340){
          VisibleLED[16] = pythag_A1_A0 - ((abs(vector_A1_A0 - 320) / abs(vector_A1_A0 - 300)) * abs(pythag_A1_A0 - VisibleLED[15]));
          VisibleLED[17] = pythag_A1_A0 - ((abs(vector_A1_A0 - 340) / abs(vector_A1_A0 - 360)) * abs(pythag_A1_A0 - VisibleLED[0]));
        }
        if (vector_A1_A0 > 340 && vector_A1_A0 <= 361){
          VisibleLED[16] = pythag_A1_A0 - ((abs(vector_A1_A0 - 320) / abs(vector_A1_A0 - 300)) * abs(pythag_A1_A0 - VisibleLED[15]));
          VisibleLED[17] = pythag_A1_A0 - ((abs(vector_A1_A0 - 340) / abs(vector_A1_A0 - 300)) * abs(pythag_A1_A0 - VisibleLED[15]));
        } 
      break;
    }
}

void Light_Max_LED_18(void){                                    //sets PWM value of one maximum LED only
  int maxvalue = 0;
  int maxindex = 0;
  int i_VisibleLED = 0;
  for(i_VisibleLED = 0; i_VisibleLED < 18; i_VisibleLED++){
    VisibleLED_PWM[i_VisibleLED] = 0;  
    switch(obstacledetect1_vector0){
      case 0:
        if (VisibleLED_Receive[i_VisibleLED] > maxvalue){
          maxvalue = VisibleLED_Receive[i_VisibleLED];
          maxindex = i_VisibleLED;
        }
      break;
      case 1:
        if (VisibleLED[i_VisibleLED] > maxvalue){
          maxvalue = VisibleLED[i_VisibleLED];
          maxindex = i_VisibleLED;
        }
      break;
    }
  }
  if (maxvalue > 3){               
    VisibleLED_PWM[maxindex] = 3600;
  }
}

void Set_Analog_CHs_HWPWM(void){              //this is the main function that sets PWM values. 
  int i_PWM = 0;  
  for(i_PWM=0; i_PWM<18; i_PWM++) {  
    VisibleLED_Previous_PWM[i_PWM] = VisibleLED_PWM[i_PWM];         //store previous PWM values used for comparison        
    switch(obstacledetect1_vector0){
      case 0:
        if(i_PWM == 1 || i_PWM == 2 || i_PWM == 4 || i_PWM == 5 || i_PWM == 7 || i_PWM == 8 || i_PWM == 10 || i_PWM == 11 || i_PWM == 13 || i_PWM == 14 || i_PWM == 16 || i_PWM == 17){
          VisibleLED_Receive[i_PWM] = VisibleLED_Receive[i_PWM] * 100 / 149;                                      //normalize to 100
        }
        if (VisibleLED_Receive[i_PWM] < 6) VisibleLED_PWM[i_PWM] = 0;                                             //0%
        else if (6 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 12) VisibleLED_PWM[i_PWM] = 21;    //0.51%
        else if (12 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 19) VisibleLED_PWM[i_PWM] = 41;   //1%
        else if (19 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 25) VisibleLED_PWM[i_PWM] = 62;   //1.51%
        else if (25 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 31) VisibleLED_PWM[i_PWM] = 103;  //2.51%
        else if (31 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 37) VisibleLED_PWM[i_PWM] = 154;  //3.76%
        else if (37 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 44) VisibleLED_PWM[i_PWM] = 210;  //5.13%
        else if (44 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 50) VisibleLED_PWM[i_PWM] = 280;  //6.84%
        else if (50 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 56) VisibleLED_PWM[i_PWM] = 380;  //9.28%
        else if (56 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 62) VisibleLED_PWM[i_PWM] = 500;  //12.21%
        else if (62 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 69) VisibleLED_PWM[i_PWM] = 800;  //19.53%
        else if (69 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 75) VisibleLED_PWM[i_PWM] = 1150; //28.08%
        else if (75 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 81) VisibleLED_PWM[i_PWM] = 1550; //37.84%
        else if (81 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 87) VisibleLED_PWM[i_PWM] = 2050; //50.05%
        else if (87 <= VisibleLED_Receive[i_PWM] && VisibleLED_Receive[i_PWM] < 94) VisibleLED_PWM[i_PWM] = 2750; //67.14%       
        else if (VisibleLED_Receive[i_PWM] >= 94) VisibleLED_PWM[i_PWM] = 3600;                                   //87.89%
      break;
      case 1:
        if(i_PWM == 1 || i_PWM == 2 || i_PWM == 4 || i_PWM == 5 || i_PWM == 7 || i_PWM == 8 || i_PWM == 10 || i_PWM == 11 || i_PWM == 13 || i_PWM == 14 || i_PWM == 16 || i_PWM == 17){
          VisibleLED[i_PWM] = VisibleLED[i_PWM] * 100 / 149;                                                      //normalize to 100
        }
        if (VisibleLED[i_PWM] < 6) VisibleLED_PWM[i_PWM] = 0;                                      
        else if (6 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 12) VisibleLED_PWM[i_PWM] = 21;  
        else if (12 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 19) VisibleLED_PWM[i_PWM] = 41;  
        else if (19 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 25) VisibleLED_PWM[i_PWM] = 62; 
        else if (25 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 31) VisibleLED_PWM[i_PWM] = 103; 
        else if (31 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 37) VisibleLED_PWM[i_PWM] = 154; 
        else if (37 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 44) VisibleLED_PWM[i_PWM] = 210; 
        else if (44 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 50) VisibleLED_PWM[i_PWM] = 280;
        else if (50 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 56) VisibleLED_PWM[i_PWM] = 380;
        else if (56 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 62) VisibleLED_PWM[i_PWM] = 500;
        else if (62 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 69) VisibleLED_PWM[i_PWM] = 800;
        else if (69 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 75) VisibleLED_PWM[i_PWM] = 1150;
        else if (75 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 81) VisibleLED_PWM[i_PWM] = 1550;
        else if (81 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 87) VisibleLED_PWM[i_PWM] = 2050;
        else if (87 <= VisibleLED[i_PWM] && VisibleLED[i_PWM] < 94) VisibleLED_PWM[i_PWM] = 2750;        
        else if (VisibleLED[i_PWM] >= 94) VisibleLED_PWM[i_PWM] = 3600;   
      break;
    }
  }
}

void Set_Visible_LEDs(void){                     //This is the main function that lights the visible LEDs
  i_VLED2 = 0;
  for(i_VLED2=0; i_VLED2<18; i_VLED2++) {
    if((VisibleLED_Previous_PWM[i_VLED2] != VisibleLED_PWM[i_VLED2]) || (VisibleLED_PWM[i_VLED2] == 0)){        //if current values are not equal to previous values, or are equal to zero
      if(i_VLED2 == 16){
        if(VisibleLED_PWM[i_VLED2] == 0) OC2RS = 0x00000000; 
        else if(VisibleLED_PWM[i_VLED2] == 21) OC2RS = 0x000000FF;    //these duty cycle values correspond to the same duty cycle percentages above
        else if(VisibleLED_PWM[i_VLED2] == 41) OC2RS = 0x000001F4;
        else if(VisibleLED_PWM[i_VLED2] == 62) OC2RS = 0x000002F3;
        else if(VisibleLED_PWM[i_VLED2] == 103) OC2RS = 0x000004E7;
        else if(VisibleLED_PWM[i_VLED2] == 154) OC2RS = 0x00000758;
        else if(VisibleLED_PWM[i_VLED2] == 210) OC2RS = 0x00000A05;
        else if(VisibleLED_PWM[i_VLED2] == 280) OC2RS = 0x00000D5C;
        else if(VisibleLED_PWM[i_VLED2] == 380) OC2RS = 0x00001220;
        else if(VisibleLED_PWM[i_VLED2] == 500) OC2RS = 0x000017D9;
        else if(VisibleLED_PWM[i_VLED2] == 800) OC2RS = 0x00002625;
        else if(VisibleLED_PWM[i_VLED2] == 1150) OC2RS = 0x000036D8;
        else if(VisibleLED_PWM[i_VLED2] == 1550) OC2RS = 0x000049E8;
        else if(VisibleLED_PWM[i_VLED2] == 2050) OC2RS = 0x000061C1;
        else if(VisibleLED_PWM[i_VLED2] == 2750) OC2RS = 0x00008322;
        else if(VisibleLED_PWM[i_VLED2] == 3600) OC2RS = 0x0000ABA9;
      }
      else if(i_VLED2 == 17){
        if(VisibleLED_PWM[i_VLED2] == 0) OC3RS = 0x00000000; 
        else if(VisibleLED_PWM[i_VLED2] == 21) OC3RS = 0x000000FF;
        else if(VisibleLED_PWM[i_VLED2] == 41) OC3RS = 0x000001F4;
        else if(VisibleLED_PWM[i_VLED2] == 62) OC3RS = 0x000002F3;
        else if(VisibleLED_PWM[i_VLED2] == 103) OC3RS = 0x000004E7;
        else if(VisibleLED_PWM[i_VLED2] == 154) OC3RS = 0x00000758;
        else if(VisibleLED_PWM[i_VLED2] == 210) OC3RS = 0x00000A05;
        else if(VisibleLED_PWM[i_VLED2] == 280) OC3RS = 0x00000D5C;
        else if(VisibleLED_PWM[i_VLED2] == 380) OC3RS = 0x00001220;
        else if(VisibleLED_PWM[i_VLED2] == 500) OC3RS = 0x000017D9;
        else if(VisibleLED_PWM[i_VLED2] == 800) OC3RS = 0x00002625;
        else if(VisibleLED_PWM[i_VLED2] == 1150) OC3RS = 0x000036D8;
        else if(VisibleLED_PWM[i_VLED2] == 1550) OC3RS = 0x000049E8;
        else if(VisibleLED_PWM[i_VLED2] == 2050) OC3RS = 0x000061C1;
        else if(VisibleLED_PWM[i_VLED2] == 2750) OC3RS = 0x00008322;
        else if(VisibleLED_PWM[i_VLED2] == 3600) OC3RS = 0x0000ABA9;
      }
      else{
        pwm.setPWM(i_VLED2, 0, VisibleLED_PWM[i_VLED2]);                
      } 
    }
  }
} 

int digital_filter(int ADCpinnumber){                               //This is a digital filter. Input is analog pin number to read
  int smoothedindex;
  if (ADCpinnumber == A0) smoothedindex = 0;  
  else if (ADCpinnumber == A1) smoothedindex = 1;
  else if (ADCpinnumber == A2) smoothedindex = 2;
  else if (ADCpinnumber == A3) smoothedindex = 3;
  else if (ADCpinnumber == A4) smoothedindex = 4;
  else if (ADCpinnumber == A5) smoothedindex = 5; 
  int ADCvalue = analogRead(ADCpinnumber);
  switch(TX_on1_off0){
    case 0:
      smoothedVal_Receive[smoothedindex] = (ADCvalue * (1 - filterVal)) + (smoothedVal_Receive[smoothedindex] * filterVal);
      return (int)smoothedVal_Receive[smoothedindex];
    break;
    case 1:
      smoothedVal[smoothedindex] = (ADCvalue * (1 - filterVal)) + (smoothedVal[smoothedindex] * filterVal);
      return (int)smoothedVal[smoothedindex];
    break;
  }
}

void Optical_Transmit_Packet(int cmdbyte, int databyte1, int databyte2){      //this is used to send a packet of 4 bytes of data. The fourth byte is a calculated checksum byte
  int checksum = ((cmdbyte + databyte1 + databyte2) ^ 0xff) & 0xff;           //this is the checksum equation
  unsigned long TransmitData = (cmdbyte << 24) | (databyte1 << 16) | (databyte2 << 8) | (checksum);         //the transmitted packet is 4 bytes long
  int index_i = 0;
  Optical_Transmit_Start_Byte();                                              //Transmit unique start byte
  for(index_i = 31; index_i >= 0; index_i--){
    unsigned long transmitbyte2 = TransmitData & (1 << index_i);
    unsigned long transmitbyte3 = TransmitData & (1 << (index_i - 1));
    if(transmitbyte2 > 0){
      if(transmitbyte3 > 0){
        timer_hold = counter;
        OC1RS = 0x00000FA0;                     //duty cycle = 50% in 16 bit mode
        while(counter < timer_hold + 10){
        }
      }
      else{
        timer_hold = counter;
        OC1RS = 0x00000FA0;                     //duty cycle = 50% in 16 bit mode
        while(counter < timer_hold + 5){
        }
        timer_hold = counter;
        OC1RS = 0x00000000;                     //duty cycle = 0% in 16 bit mode
        while(counter < timer_hold + 5){
        }
      }   
    }
    else {
      timer_hold = counter;
      OC1RS = 0x00000000;                       //duty cycle = 0% in 16 bit mode
      while(counter < timer_hold + 10){
      }
    }
  }                                             
  digitalWrite(PWM0_EN, LOW);                   //turn off GPIOs                           
  digitalWrite(PWM1_EN, LOW);   
  digitalWrite(PWM2_EN, LOW);   
  digitalWrite(PWM3_EN, LOW);   
  digitalWrite(PWM4_EN, LOW);   
  digitalWrite(PWM5_EN, LOW);  
  OC1RS = 0x00000FA0;                           //duty cycle = 50% in 16 bit mode  
}

void Optical_Transmit_Start_Byte(void){   
  OC1RS = 0x00000000;                     //duty cycle = 0% in 16 bit mode
  digitalWrite(PWM0_EN, HIGH);            //Turns on GPIO pins to enable transmission
  digitalWrite(PWM1_EN, HIGH);   
  digitalWrite(PWM2_EN, HIGH);   
  digitalWrite(PWM3_EN, HIGH);   
  digitalWrite(PWM4_EN, HIGH);   
  digitalWrite(PWM5_EN, HIGH);     
  timer_hold = counter;
  OC1RS = 0x00000FA0;                     //duty cycle = 50% in 16 bit mode
  while(counter < timer_hold + 13){       //1.3ms or 1.3 bits
  }
  timer_hold = counter;
  OC1RS = 0x00000000;                     //duty cycle = 0% in 16 bit mode
  while(counter < timer_hold + 17){       //1.7ms or 1.7 bits
  }  
}

void start_timer_1(uint32_t frequency_12hz) {      //Starts Timer 1 used for visual updates
  uint32_t period_12hz;  
  period_12hz = TICKS_PER_SECOND / frequency_12hz / 256;
  T1CONCLR = T3_ON;         /* Turn the timer off */
  T1CON = 0x30;             /* Set the prescaler to 256, timer 1 is always 16 bit */
  TMR1 = 0;                 /* Clear the counter  */
  PR1 = period_12hz;             /* Set the period     */
  T1CONSET = T3_ON;         /* Turn the timer on  */
} 

void start_timer_2(uint32_t frequency_VLED) {      //Starts Timer 2 used for visible LEDS 16 & 17
  uint32_t period_VLED;  
  period_VLED = TICKS_PER_SECOND / frequency_VLED / 16;
  T2CONCLR = T3_ON;         /* Turn the timer off */
  T2CON = 0x40;             /* Set the prescaler to 16, 16 bit timer */
  TMR2 = 0;                 /* Clear the counter  */
  PR2 = period_VLED;             /* Set the period     */
  T2CONSET = T3_ON;         /* Turn the timer on  */
} 

void start_timer_3(uint32_t frequency) {          //Starts Timer 3 used in accurate counter
  uint32_t period;  
  period = TICKS_PER_SECOND / frequency;
  T3CONCLR = T3_ON;         /* Turn the timer off */
  T3CON = 0x00;             /* Set the prescaler to 0 */
  TMR3 = 0;                 /* Clear the counter  */
  PR3 = period;             /* Set the period     */
  T3CONSET = T3_ON;         /* Turn the timer on  */
} 

void start_timer_4(uint32_t frequency_10hz) {      //Starts Timer 2 used in visual updates
  uint32_t period_10hz;  
  period_10hz = TICKS_PER_SECOND / frequency_10hz / 256;
  T4CONCLR = T3_ON;         /* Turn the timer off */
  T4CON = 0x70;             /* Set the prescaler to 256, 16 bit timer */
  TMR4 = 0;                 /* Clear the counter  */
  PR4 = period_10hz;             /* Set the period     */
  T4CONSET = T3_ON;         /* Turn the timer on  */
} 

void start_timer_5(uint32_t frequency_1khz) {     //Starts Timer 5 used in external interrupts
  uint32_t period_1khz;  
  period_1khz = TICKS_PER_SECOND / 2 / frequency_1khz;
  T5CONCLR = T3_ON;         /* Turn the timer off */
  T5CON = 0x10;             /* Set the prescaler to 2 */
  TMR5 = 0;                 /* Clear the counter  */
  PR5 = period_1khz;        /* Set the period     */
  T5CONSET = T3_ON;         /* Turn the timer on  */
} 

void CCW_Visible_LEDs_index(int index){                   //CCW visible LED show
  OC2RS = 0x00000000;                                     //turn everything off
  OC3RS = 0x00000000;
  for(int i=0; i<16; i++) pwm.setPWM(i, 0, 0);
  if (index == 5 || index == 4 || index ==2 || index == 1){
    index = abs(index - 6) * 3;
  }
  else if (index == 0 || index == 3){
    index = index * 3;
  }
  int index_hold = index;
  while(index >= 0){
    pwm.setPWM(index, 0, 3500);
    delay(VLED_on_time);
    pwm.setPWM(index, 0, 0);
    delay(VLED_off_time);
    index--;
  }
  OC3RS = 0x0000ABA9;
  delay(VLED_on_time);
  OC3RS = 0x00000000;
  delay(VLED_off_time);
  OC2RS = 0x0000ABA9;
  delay(VLED_on_time);
  OC2RS = 0x00000000;
  delay(VLED_off_time);
  index = 15;
  while(index > index_hold){
    pwm.setPWM(index, 0, 3500);
    delay(VLED_on_time);
    pwm.setPWM(index, 0, 0);
    delay(VLED_off_time);
    index--;
  }
}

void CW_Visible_LEDs_index(int index){                      //CW visible LED show  
  OC2RS = 0x00000000;
  OC3RS = 0x00000000;
  for(int i=0; i<16; i++) pwm.setPWM(i, 0, 0);
  if (index == 5 || index == 4 || index == 2 || index == 1){
    index = abs(index - 6) * 3;
  }
  else if (index == 0 || index == 3){
    index = index * 3;
  }
  int index_hold = index;
  while(index <= 15){
    pwm.setPWM(index, 0, 3500);
    delay(VLED_on_time);
    pwm.setPWM(index, 0, 0);
    delay(VLED_off_time);
    index++;
  }
  OC2RS = 0x0000ABA9;
  delay(VLED_on_time);
  OC2RS = 0x00000000;
  delay(VLED_off_time); 
  OC3RS = 0x0000ABA9;
  delay(VLED_on_time);
  OC3RS = 0x00000000;
  delay(VLED_off_time);
  index = 0;
  while(index < index_hold){
    pwm.setPWM(index, 0, 3500);
    delay(VLED_on_time);
    pwm.setPWM(index, 0, 0);
    delay(VLED_off_time);
    index++;
  }
}

void Test_Board(void){                  //Program to test OLED display, visible LEDs              
  IOShieldOled.begin();                                         
  IOShieldOled.clearBuffer();
  IOShieldOled.setCursor(0, 0);
  IOShieldOled.putString("Board Test");
  int i_test = 0;
  for(i_test = 0; i_test < 16; i_test++){
    pwm.setPWM(i_test, 0, 3500);
    delay(750);
    pwm.setPWM(i_test, 0, 0);
    delay(50);
  }
  OC2RS = 0x0000ABA9;
  delay(750);
  OC2RS = 0x00000000;
  delay(50);
  OC3RS = 0x0000ABA9;
  delay(750);
  OC3RS = 0x00000000;
  delay(50);
}


  
