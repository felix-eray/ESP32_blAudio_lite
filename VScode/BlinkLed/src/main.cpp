#include <Arduino.h>
#include <ESP32Encoder.h>
#include <driver/ledc.h>
#include "Wire.h"
#include "TFA9879.h"

uint8_t writeBassTreble(int , int , int );

ESP32Encoder encoder;
TFA9879 TFA[2];

const int PWMPin = 1; 
const int PWMChannel = 0;
const int freq = 48000*256;
const int resolution = 1; //resolution in bits
const int dutyCycle = 1; //value when using bits from resolution

const int anaPot = 26;
const int encSW = 32;
const int encA = 32;
const int encB = 25;
const int adcEN = 21;
const int ledPin1 = 16;
const int ledPin2 = 17;
const int pushButton1 = 4;
const int pushButton2 = 2;
const int pushButton3 = 15;
const int I2C_SDA = 14;
const int I2C_SCL = 27;
const int pwrEN = 12;
const int en5V = 39;
const int sw3_5mm = 13;
const int LRCK1 = 19;
const int LRCK2 = 22;
const int BCLK1 = 18;
const int BCLK2 = 3;
const int SDI1 = 5;
const int SDI2 = 23;
const int MCLK = 1;

//TFA9879, I2C-address 0b11011YX, solder bridge open = 1, closed = 0, (JP4/JP2) = X, (JP5/JP3) = Y
//both open: 0b1101111
//both closed: 0b1101100
//(JP4/JP2) closed, (JP5/JP3) open: 0b1101110
//(JP4/JP2) open, (JP5/JP3) closed: 0b1101101

bool initialized = 0;

//PCF8574A, I2C-address 0b0111ZYX, solder bridge open = 1, closed = 0, (JP6) = X, (JP7) = Y, (JP8) = Z 

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin ledPin as an output.
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(adcEN, OUTPUT);
  pinMode(pwrEN, OUTPUT);
  pinMode(en5V, OUTPUT);

  digitalWrite(adcEN, LOW);
  digitalWrite(pwrEN, HIGH);
  digitalWrite(en5V, LOW);


  Wire.setPins((int)I2C_SDA, (int)I2C_SCL);
  initialized = Wire.begin(I2C_SDA, I2C_SCL, (uint32_t)100000);

  TFA_init(&TFA[0]);
  TFA_init(&TFA[1]);

  TFA_setAddress(&TFA[0], 0b1101110); //upper SB closed
  TFA_setAddress(&TFA[1], 0b1101111); //both SBs open

  delay(10);

  ESP32Encoder::useInternalWeakPullResistors=UP;

	// use pin 19 and 18 for the first encoder
	encoder.attachSingleEdge(encA, encB);

  // configure PWM 
  ledcSetup(PWMChannel, freq, resolution);
  
  // attach pin to be controlled
  ledcAttachPin(PWMPin, PWMChannel);

  //write PWM to output pin.
  ledcWrite(PWMChannel, dutyCycle);

  *((volatile uint32_t *) (IO_MUX_GPIO1_REG)) = ((*(volatile uint32_t *) (IO_MUX_GPIO1_REG)) & ~(3 << 10));// | (1 << 10);

  delay(100);
  digitalWrite(adcEN, HIGH);
  delay(100);

  //note, the ADC for the 3,5mm jack input needs the 12.288 MHz master clock and serial port will use that pin
  //Serial.begin(115200);

  /*
  Wire.beginTransmission(0b1101110);
  Wire.write(0x13); //volume control 0x13
  Wire.write(0x10);
  Wire.write(0x30); //-0 db
  Wire.endTransmission(true);
  */


  TFA_setVolume(&TFA[0], 0);
  TFA_setBassTreble(&TFA[0], 5, 5);

  delay(100);

  Wire.beginTransmission(0b1101110);
  Wire.write(0x00); //device control 0x00
  Wire.write(0x00);
  Wire.write(0x19); //input 2, amp output on, TF9789 powered
  Wire.endTransmission(true);
}

// the loop function runs over and over again forever
void loop() {
  static int loops = 0;

  digitalWrite(ledPin1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(ledPin2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                  // wait for a second
  digitalWrite(ledPin1, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(ledPin2, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                  // wait for a second

  
  
  
  /*
  uint16_t data = 0;
  while(Wire.available() != 2);
  data = (Wire.read() << 8);
  data |= Wire.read();
  */
  
  
  //Serial.println(encoder.getCount());
  //Serial.println(initialized);
  //Serial.println(data, HEX);
  //Serial.println(*(uint32_t*)GPIO_FUNC1_OUT_SEL_CFG_REG, HEX);
  //Serial.println(*(uint32_t*)IO_MUX_GPIO1_REG, HEX);
  if( loops == 30 );
    //digitalWrite(pwrEN, LOW);

  loops++;
}