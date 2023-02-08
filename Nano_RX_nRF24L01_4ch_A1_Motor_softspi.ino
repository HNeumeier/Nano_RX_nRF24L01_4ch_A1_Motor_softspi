//************************************************************************************************************************************************************************
//Communication nRF24L01P. Fixed RF channel, fixed address.
//Support for TX Telemetry LCD transmitter https://github.com/stanekTM/TX_nRF24L01_4ch_Telemetry_LCD
//and for the TX Telemetry LED transmitter https://github.com/stanekTM/TX_nRF24L01_5ch_Telemetry_LED
//************************************************************************************************************************************************************************
__attribute__((section(".noinit"))) volatile bool reboot;

void(*resetFunc) (void) = 0; 

//#define DEBUG_SERIAL  // uncomment for output on Serial

#include "My_RF24.h"         //https://github.com/nRF24/RF24  // modified for soft_SPI
#ifdef DEBUG_SERIAL
  #include <printf.h>       //print the radio debug info
#endif
#include <DigitalIO.h>    //https://github.com/greiman/DigitalIO
#include "PWMFrequency.h" //used locally https://github.com/TheDIYGuy999/PWMFrequency

//setting RF channels address (5 bytes number or character)
const byte address[] = "jirka";

//RF communication channel settings (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RADIO_CHANNEL  76

//setting the reaction of the motor to be rotated after the lever has been moved. Settings (0-255))
#define ACCELERATE_MOTOR_A  0
#define ACCELERATE_MOTOR_B  0
#define ACCELERATE_MOTOR_TOWER  0

//setting the maximum motor power. Suitable for TX transmitters without endpoint setting. Settings (0-255)
#define MAX_MOTOR_A  255
#define MAX_MOTOR_B  255
#define MAX_MOTOR_TOWER  255

//brake setting, no brake 0, max brake 255. Settings (0-255)
#define BRAKE_MOTOR_A  0
#define BRAKE_MOTOR_B  0
#define BRAKE_MOTOR_TOWER  0

//LED alarm battery voltage setting
#define BATTERY_VOLTAGE    3.7 // not used
#define MONITORED_VOLTAGE  2.0 // not used

//setting the dead zone of poor quality joysticks TX for the motor controller
#define DEAD_ZONE  15

//setting the control range value
#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

//settings PWM (pin D5 or D6 are paired on timer0/8-bit, functions delay, millis, micros and delayMicroseconds)
//1024 = 61Hz, 256 = 244Hz, 64 = 976Hz(default), 8 = 7812Hz
//#define PWM_MOTOR_A  64

//settings PWM (pin D9 or D10 are paired on timer1/16-bit, Servo library)
//1024 = 30Hz, 256 = 122Hz, 64 = 488Hz(default), 8 = 3906Hz
#define PWM_MOTOR_A  256

//settings PWM (pin D3 or D11 are paired on timer2/8-bit, ServoTimer2 library)
//1024 = 30Hz, 256 = 122Hz, 128 = 244Hz, 64 = 488Hz(default), 32 = 976Hz, 8 = 3906Hz
#define PWM_MOTOR_B  256

//free pins
//pin                      0
//pin                      1
//pin                      4
//pin                      5
//pin                      6
//pin                      7
//pin                      8
//pin                      12 //MISO
//pin                      13 //SCK
//pin                      A5
//pin                      A6
 
//pwm pins for motor
#define PIN_PWM_1_MOTOR_A  9
#define PIN_PWM_2_MOTOR_A  10
#define PIN_PWM_3_MOTOR_B  3
#define PIN_PWM_4_MOTOR_B  11 //MOSI
#define PIN_1_MOTOR_TOWER  5
#define PIN_2_MOTOR_TOWER  6
#define PIN_1_MOTOR_SHOOT  7

//LED RX battery and RF on/off
#define PIN_LED            LED_BUILTIN // Nano/UNO D13 / ESP32 D02 / NodeMCU LED D0/GPIO 16 / ESP8266 D4/GPIO 02

//input RX battery
#define PIN_RX_BATTERY     A7

// SPI Pins are defined in My_RF24_config.h

//pins for nRF24L01
#define PIN_CE                 14     // A0
#define PIN_CSN                15     // A1

//software SPI https://nrf24.github.io/RF24/md_docs_arduino.html
//----- SCK                    16 - A2
//----- MOSI                   17 - A3
//----- MISO                   18 - A4

//setting of CE and CSN pins
My_RF24 radio(PIN_CE, PIN_CSN);

//************************************************************************************************************************************************************************
//this structure defines the received data in bytes (structure size max. 32 bytes) ***************************************************************************************
//************************************************************************************************************************************************************************
struct rc_packet_size
{
  unsigned int ch_motorA = MID_CONTROL_VAL;
  unsigned int ch_motorB = MID_CONTROL_VAL;
  unsigned int ch_motorTower = MID_CONTROL_VAL; // Motor for the Tower
  unsigned int ch_motorShoot = MID_CONTROL_VAL; // Motor to shoot
  unsigned int ch_servo3 = MID_CONTROL_VAL; //unused channel, only adding byte array TX 5ch
};
rc_packet_size rc_packet; //create a variable with the above structure

//************************************************************************************************************************************************************************
//this struct defines data, which are embedded inside the ACK payload ****************************************************************************************************
//************************************************************************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;        //not used yet
  byte RX_batt_A1;
  byte RX_batt_A2; //not used yet
};
telemetry_packet_size telemetry_packet;

//************************************************************************************************************************************************************************
//fail safe, settings 1000-2000 ​​(MIN_CONTROL_VAL = 1000, MID_CONTROL_VAL = 1500, MAX_CONTROL_VAL = 2000) *****************************************************************
//************************************************************************************************************************************************************************
void fail_safe()
{
  rc_packet.ch_motorA = MID_CONTROL_VAL;
  rc_packet.ch_motorB = MID_CONTROL_VAL;
  rc_packet.ch_motorTower = MID_CONTROL_VAL;
  rc_packet.ch_motorShoot = MID_CONTROL_VAL;
}

//************************************************************************************************************************************************************************
//setup frequencies and motors control ***********************************************************************************************************************************
//************************************************************************************************************************************************************************
int value_motorA = 0, value_motorB = 0, value_motorTower = 0;

void output_PWM()
{
  #ifdef DEBUG_SERIAL
    //Serial.print("rc_packet.ch_motorA: "); //print value ​​on a serial monitor
    //Serial.print(rc_packet.ch_motorA); //print value ​​on a serial monitor
    //Serial.print(" rc_packet.ch_motorB: "); //print value ​​on a serial monitor
    //Serial.print(rc_packet.ch_motorB); //print value ​​on a serial monitor
    //Serial.print(" rc_packet.ch_motorTower: "); //print value ​​on a serial monitor
    //Serial.print(rc_packet.ch_motorTower); //print value ​​on a serial monitor
    //Serial.print(" rc_packet.ch_motorShoot: "); //print value ​​on a serial monitor
    //Serial.print(rc_packet.ch_motorShoot); //print value ​​on a serial monitor
  #endif
  setPWMPrescaler(PIN_PWM_1_MOTOR_A, PWM_MOTOR_A);
  setPWMPrescaler(PIN_PWM_3_MOTOR_B, PWM_MOTOR_B);
  
  //forward motorA
  if (rc_packet.ch_motorA > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorA = map(rc_packet.ch_motorA, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR_A, MAX_MOTOR_A);
    value_motorA = constrain(value_motorA, ACCELERATE_MOTOR_A, MAX_MOTOR_A);
    analogWrite(PIN_PWM_2_MOTOR_A, value_motorA); 
    digitalWrite(PIN_PWM_1_MOTOR_A, LOW);
  }
  //back motorA
  else if (rc_packet.ch_motorA < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorA = map(rc_packet.ch_motorA, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR_A, MAX_MOTOR_A);
    value_motorA = constrain(value_motorA, ACCELERATE_MOTOR_A, MAX_MOTOR_A);
    analogWrite(PIN_PWM_1_MOTOR_A, value_motorA);
    digitalWrite(PIN_PWM_2_MOTOR_A, LOW);
  }
  else
  {
    analogWrite(PIN_PWM_1_MOTOR_A, BRAKE_MOTOR_A);
    analogWrite(PIN_PWM_2_MOTOR_A, BRAKE_MOTOR_A);
  }
  #ifdef DEBUG_SERIAL
    //Serial.print(" value_motorA: ");
    //Serial.print(value_motorA);
  #endif
  //forward motorB
  if (rc_packet.ch_motorB > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorB = map(rc_packet.ch_motorB, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR_B, MAX_MOTOR_B);
    value_motorB = constrain(value_motorB, ACCELERATE_MOTOR_B, MAX_MOTOR_B);
    analogWrite(PIN_PWM_4_MOTOR_B, value_motorB);
    digitalWrite(PIN_PWM_3_MOTOR_B, LOW);
  }
  //back motorB
  else if (rc_packet.ch_motorB < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorB = map(rc_packet.ch_motorB, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR_B, MAX_MOTOR_B);
    value_motorB = constrain(value_motorB, ACCELERATE_MOTOR_B, MAX_MOTOR_B);
    analogWrite(PIN_PWM_3_MOTOR_B, value_motorB);
    digitalWrite(PIN_PWM_4_MOTOR_B, LOW);
  }
  else
  {
    analogWrite(PIN_PWM_3_MOTOR_B, BRAKE_MOTOR_B);
    analogWrite(PIN_PWM_4_MOTOR_B, BRAKE_MOTOR_B);
  }
  #ifdef DEBUG_SERIAL
    //Serial.print(" value_motorB: ");
    //Serial.print(value_motorB);
  #endif
  //forward motorTower
  if (rc_packet.ch_motorTower > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorTower = map(rc_packet.ch_motorTower, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR_TOWER, MAX_MOTOR_TOWER);
    value_motorTower = constrain(value_motorTower, ACCELERATE_MOTOR_TOWER, MAX_MOTOR_TOWER);
    analogWrite(PIN_2_MOTOR_TOWER, value_motorTower); 
    digitalWrite(PIN_1_MOTOR_TOWER, LOW);
  }
  //back motorTower
  else if (rc_packet.ch_motorTower < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorTower = map(rc_packet.ch_motorTower, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR_TOWER, MAX_MOTOR_TOWER);
    value_motorTower = constrain(value_motorTower, ACCELERATE_MOTOR_TOWER, MAX_MOTOR_TOWER);
    analogWrite(PIN_1_MOTOR_TOWER, value_motorTower);
    digitalWrite(PIN_2_MOTOR_TOWER, LOW);
  }
  else
  {
    analogWrite(PIN_1_MOTOR_TOWER, BRAKE_MOTOR_TOWER);
    analogWrite(PIN_2_MOTOR_TOWER, BRAKE_MOTOR_TOWER);
  }
  #ifdef DEBUG_SERIAL
    //Serial.print(" value_motorTower: ");
    //Serial.println(value_motorTower);
  #endif
  
  //forward motorShoot
  if (rc_packet.ch_motorShoot > MID_CONTROL_VAL + DEAD_ZONE)
  {
    digitalWrite(PIN_1_MOTOR_SHOOT, HIGH);
    #ifdef DEBUG_SERIAL
      //Serial.print(" PIN_1_MOTOR_SHOOT: ");
      //Serial.println(PIN_1_MOTOR_SHOOT);
    #endif
  }
  //back motorShoot
  else if (rc_packet.ch_motorShoot < MID_CONTROL_VAL - DEAD_ZONE)
  {
    digitalWrite(PIN_1_MOTOR_SHOOT, LOW);
    #ifdef DEBUG_SERIAL
      //Serial.print(" PIN_1_MOTOR_SHOOT: ");
      //Serial.println(PIN_1_MOTOR_SHOOT);
    #endif
  }
  else
  {
    digitalWrite(PIN_1_MOTOR_SHOOT, LOW);
  }
  #ifdef DEBUG_SERIAL
    //Serial.print(" PIN_1_MOTOR_SHOOT: ");
    //Serial.println(PIN_1_MOTOR_SHOOT);
  #endif
}

//************************************************************************************************************************************************************************
//initial main settings **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void setup()
{
#ifdef DEBUG_SERIAL  
  Serial.begin(19200);
  printf_begin();     //print the radio debug info
#endif
  delay(500);
  pinMode(PIN_PWM_1_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_2_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_3_MOTOR_B, OUTPUT);
  pinMode(PIN_PWM_4_MOTOR_B, OUTPUT);
  pinMode(PIN_1_MOTOR_TOWER, OUTPUT);
  pinMode(PIN_2_MOTOR_TOWER, OUTPUT);
  pinMode(PIN_1_MOTOR_SHOOT, OUTPUT);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RX_BATTERY, INPUT);
  
  fail_safe();
  
  //define the radio communication
  radio.begin();
  radio.setAutoAck(true);          //ensure autoACK is enabled (default true)
  radio.enableAckPayload();        //enable Ack dynamic payloads. This only works on pipes 0&1 by default
  radio.enableDynamicPayloads();   //enable dynamic payloads on all pipes
  radio.setRetries(4, 4);        //// delay, count set the number and delay of retries on failed submit (max. 15 x 250us delay (blocking !), max. 15 retries)
  
  radio.setChannel(RADIO_CHANNEL); //which RF channel to communicate on (0-125, 2.4Ghz + 76 = 2.476Ghz)
  radio.setDataRate(RF24_250KBPS); //RF24_250KBPS (fails for units without +), RF24_1MBPS, RF24_2MBPS
  radio.setPALevel(RF24_PA_MIN);   //RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)
  
  radio.openReadingPipe(1, address);     //open the reading pipe1 (RX_ADDR_P1) and then call "startListening"
  
  radio.startListening(); //set the module as receiver. Start listening on the pipes opened for reading

  Serial.println("Radio details *****************");
  //radio.printDetails(); //print the radio debug info Basic
  radio.printPrettyDetails(); //print the radio debug info Detail
  
// do a one Time reboot because with only power on sometimes the Radio is not correct initialized
  if (!reboot) {
    reboot = true;
    #ifdef DEBUG_SERIAL 
      Serial.println("Reboot");
    #endif
    resetFunc();
  }
}

//************************************************************************************************************************************************************************
//program loop ***********************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void loop()
{
  last_rx_time();
  send_and_receive_data();
  output_PWM();
}

//************************************************************************************************************************************************************************
//get time after losing RF data or turning off the TX, reset data and the LED flashing ***********************************************************************************
//************************************************************************************************************************************************************************
unsigned long rx_time = 0;
unsigned long reception_time = 0;

void last_rx_time()
{
  // set a Basic rssi Value to send back to the transmitter
  reception_time = millis() - rx_time;
  if (reception_time < 10)   telemetry_packet.rssi = 100;
  if (reception_time > 10 and reception_time < 70) telemetry_packet.rssi = 90;
  if (reception_time > 70 and reception_time < 100) telemetry_packet.rssi = 50;
  if (reception_time > 100) telemetry_packet.rssi = 0;
  #ifdef DEBUG_SERIAL
    //Serial.println("reception_time:");
    //Serial.println(reception_time);
  #endif    

  if(millis() - rx_time > 1000) //1s
  {
    #ifdef DEBUG_SERIAL
      Serial.println("Fail Safe");
    #endif    
    fail_safe();
    RF_off_check();
  }
}

//************************************************************************************************************************************************************************
//send and receive data **************************************************************************************************************************************************
//************************************************************************************************************************************************************************
void send_and_receive_data()
{
  if (radio.available())
  {
    radio.read(&rc_packet, sizeof(rc_packet_size));
      #ifdef DEBUG_SERIAL
      //Serial.print("RX_batt_A1: ");
      //Serial.print(telemetry_packet.RX_batt_A1);
      //Serial.print(" RX_batt_A2: ");
      //Serial.println(telemetry_packet.RX_batt_A2);
      //Serial.println(telemetry_packet.rssi);
      //Serial.println(reception_time);
    #endif      
    RX_batt_check();
    radio.writeAckPayload(1, &telemetry_packet, sizeof(telemetry_packet_size));
    rx_time = millis(); //at this moment we have received the data
  }
}

//************************************************************************************************************************************************************************
//reading adc RX battery. After receiving RF data, the monitored RX battery is activated *********************************************************************************
//when RX BATTERY_VOLTAGE < MONITORED_VOLTAGE = LED alarm RX flash at a interval of 0.5s. Battery OK = LED RX is lit *****************************************************
//************************************************************************************************************************************************************************
unsigned long adc_time = 0, led_time = 0;
int adcval;
bool batt_detect, led_state;

void RX_batt_check()
{
  if (millis() - adc_time > 1000) //delay adc reading RX battery
  {
    int val;
    adc_time = millis();
    adcval = analogRead(PIN_RX_BATTERY);
    val = map(adcval, 0, 1023, 0, 255); // reduce to 8bit to send to the Transmitter
    telemetry_packet.RX_batt_A1 = val;  // for 4in1 Multi and openTX
  }

  batt_detect = telemetry_packet.RX_batt_A1 <= MONITORED_VOLTAGE; // not used
  
  if (millis() - led_time > 500)
  {
    led_time = millis();
    
    if (led_state >= !batt_detect + HIGH)
    {
      led_state = LOW;
    }
    else
    {
      led_state = HIGH;
    }
    digitalWrite(PIN_LED, led_state);
  }
}

//************************************************************************************************************************************************************************
//when RX is switched on and TX is switched off, or after the loss of RF data = LED RX flash at a interval of 0.1s. Normal mode = LED RX is lit **************************
//************************************************************************************************************************************************************************
void RF_off_check()
{
  if (millis() - led_time > 100)
  {
    led_time = millis();
    
    if (led_state)
    {
      led_state = LOW;
    }
    else
    {
      led_state = HIGH;
    }
    digitalWrite(PIN_LED, led_state);
  }
}
 
