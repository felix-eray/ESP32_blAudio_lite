#include <Arduino.h>
#include <ESP32Encoder.h>
#include <driver/ledc.h>
#include "Wire.h"
#include "TFA9879.h"
#include "BluetoothA2DPSink.h"
#include "LiquidCrystal_I2C.h"

#define COLUMS 20
#define ROWS   4

#define PAGE   ((COLUMS) * (ROWS))

LiquidCrystal_I2C lcd((PCF8574_address)0b0111111, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

void buttonPress(void);
void toggleAudioSource(void);

BluetoothA2DPSink a2dp_sink;

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
const int pushButton3 = 4;
const int pushButton2 = 2;
const int pushButton1 = 15;
const int I2C_SDA = 14;
const int I2C_SCL = 27;
const int pwrEN = 12;
const int en5V = 13;
const int sw3_5mm = 35;
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

//PCF8574A, I2C-address 0b0111ZYX, solder bridge open = 1, closed = 0, (JP6) = X, (JP7) = Y, (JP8) = Z 

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin ledPin as an output.
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(adcEN, OUTPUT);
  pinMode(pwrEN, OUTPUT);
  pinMode(en5V, OUTPUT);
  pinMode(anaPot, INPUT);
  pinMode(pushButton1, INPUT_PULLUP);
  pinMode(pushButton2, INPUT_PULLUP);
  pinMode(pushButton3, INPUT_PULLUP);

  digitalWrite(adcEN, LOW);
  digitalWrite(pwrEN, HIGH);
  digitalWrite(en5V, HIGH);

  Wire.setPins((int)I2C_SDA, (int)I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL, (uint32_t)100000);

  TFA_init(&TFA[0]);
  TFA_init(&TFA[1]);

  TFA_setAddress(&TFA[0], 0b1101110); //upper SB closed
  TFA_setAddress(&TFA[1], 0b1101111); //both SBs open

  i2s_pin_config_t my_pin_config = {
      .bck_io_num = BCLK1,
      .ws_io_num = LRCK1,
      .data_out_num = SDI1,
      .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(my_pin_config);
  //a2dp_sink.start("testBL");

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
  //note, the ADC for the 3,5mm jack input needs the 12.288 MHz master clock and serial port will use that pin
  Serial.begin(115200);

  delay(100);
  digitalWrite(adcEN, HIGH);
  delay(100);

  TFA_setVolume(&TFA[0], 0);
  TFA_setBassTreble(&TFA[0], 5, 5);
  TFA_setBassTrebleCfrequency(&TFA[0], 0, 2);
  TFA_setI2SsampleFreq(&TFA[0], 7, 0);  //set sample frequency for I2S-channel 1 to 44,1kHz for bluetooth

  TFA_setVolume(&TFA[1], 0);
  TFA_setBassTreble(&TFA[1], 5, 5);
  TFA_setBassTrebleCfrequency(&TFA[1], 0, 2);
  TFA_setI2SsampleFreq(&TFA[1], 7, 0);  //set sample frequency for I2S-channel 1 to 44,1kHz for bluetooth

  TFA_setLRchannel(&TFA[0], 2, 1);  //set TFA #1 amp's I2S-channel 2, to both channel only
  TFA_setLRchannel(&TFA[1], 2, 1);  //set TFA #2 amp's I2S-channel 2, to both channel only

  //TFA_parametricEQband(&TFA[0], 0, 100, -10, 1.0);
  //TFA_parametricEQband(&TFA[1], 0, 100, 5, 1.0);

  //TFA_setBypassEQ(&TFA[0], 0);
  //TFA_setBypassEQ(&TFA[1], 0);

  delay(100);

  TFA_setDeviceControl(&TFA[0], 1, 1);  //TFA #1 amp, use I2S channel #2, power on and amp on
  TFA_setDeviceControl(&TFA[1], 1, 1);  //TFA #2 amp, use I2S channel #2, power on and amp on

  uint8_t test = lcd.begin(COLUMS, ROWS, LCD_5x8DOTS);

  Serial.println(test);

  lcd.print(F("PCF8574 is OK...")); //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  lcd.clear();

  /* prints static text */
  lcd.setCursor(0, 1);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
  lcd.print(F("Hello world!"));
  lcd.setCursor(0, 2);            //set 1-st colum & 3-rd row, 1-st colum & row started at zero
  lcd.print(F("Random number:"));

}

// the loop function runs over and over again forever
void loop() {
  static uint32_t tick_volumeAnaRead = millis();
  static uint32_t tick_blinkLed = millis();
  static uint32_t tick_buttonChecks = millis();
  static uint32_t button1debounce = 0;

  //non-blocking led blink
  if( tick_blinkLed < millis() ){
    static int ledState = 0;
    tick_blinkLed = tick_blinkLed + 1000;

    if( ledState == 0 ){
      //digitalWrite(ledPin1, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(ledPin2, HIGH);   // turn the LED on (HIGH is the voltage level)
      ledState = 1;
    }
    else{
      //digitalWrite(ledPin1, LOW);    // turn the LED off by making the voltage LOW
      digitalWrite(ledPin2, LOW);    // turn the LED off by making the voltage LOW
      ledState = 0;
    }
  }

  //read analog pot reading roughly every 100ms and update volume if reading has changed enough to indicate that the knob has been turned
  if( tick_volumeAnaRead < millis() ){
    static int volumeOld = 0;
    tick_volumeAnaRead = tick_volumeAnaRead + 100;

    int volume = analogRead(anaPot);
    int difference = volume - volumeOld;

    //check for big enough difference between new and old reading
    if( difference > 55 || difference < -55 ){
      volumeOld = volume;

      if(volume == 4095){
        TFA_setVolume(&TFA[0], 0);
        TFA_setVolume(&TFA[1], 0);
      }
      else if(volume <= 100){
        TFA_setVolume(&TFA[0], -71);
        TFA_setVolume(&TFA[1], -71);
      }
      else{
        int setVolume = (volume / 58) - 70;
        TFA_setVolume(&TFA[0], setVolume);
        TFA_setVolume(&TFA[1], setVolume);
      }
    }
  }

  if( tick_buttonChecks < millis() ){
    tick_buttonChecks = tick_buttonChecks + 1;
    buttonPress();
  }

  
}

void buttonPress(){
  static uint16_t button1Debounce = 0, button1press = 0;
  static uint16_t button2Debounce = 0, button2press = 0;
  static uint16_t button3Debounce = 0, button3press = 0;


  if( digitalRead(pushButton1) == 0 && button1Debounce == 0 ){  //increase counter for how long button has been pressed
    button1press++;
  }
  else if( digitalRead(pushButton1) == 1 && button1Debounce != 0 ){ //start decreasing debounce
    button1Debounce--;
  }

  if( digitalRead(pushButton1) == 0 && button1press == 1500 ){  //long hold >1,5seconds, executed only once
    toggleAudioSource();

    button1press = 0;
    button1Debounce = 20;
  }
  else if( digitalRead(pushButton1) == 1 && button1press != 0 ){  //short press <1,5 seconds

    button1press = 0;
    button1Debounce = 20;
  }

  if( digitalRead(pushButton2) == 0 && button2Debounce == 0 ){  //increase counter for how long button has been pressed
    button2press++;
  }
  else if( digitalRead(pushButton2) == 1 && button2Debounce != 0 ){ //start decreasing debounce
    button2Debounce--;
  }

  if( digitalRead(pushButton2) == 0 && button2press == 1500 ){  //long hold >1,5seconds, executed only once

    button2press = 0;
    button2Debounce = 20;
  }
  else if( digitalRead(pushButton2) == 1 && button2press != 0 ){  //short press <1,5 seconds

    button2press = 0;
    button2Debounce = 20;
  }

  if( digitalRead(pushButton3) == 0 && button3Debounce == 0 ){  //increase counter for how long button has been pressed
    button3press++;
  }
  else if( digitalRead(pushButton3) == 1 && button3Debounce != 0 ){ //start decreasing debounce
    button3Debounce--;
  }

  if( digitalRead(pushButton3) == 0 && button3press == 1500 ){  //long hold >1,5seconds, executed only once
    digitalWrite(pwrEN, LOW);

    button3press = 0;
    button3Debounce = 20;
  }
  else if( digitalRead(pushButton3) == 1 && button3press != 0 ){  //short press <1,5 seconds

    button3press = 0;
    button3Debounce = 20;
  }

}

void toggleAudioSource(){
  static uint8_t source = 0;

  if(source == 0){
    a2dp_sink.start("testBL", 1);
    digitalWrite(ledPin1, HIGH);
    delay(100);

    TFA_setDeviceControl(&TFA[0], 1, 0);  //TFA #1 amp, use I2S channel #1, power on and amp on
    TFA_setDeviceControl(&TFA[1], 1, 0);  //TFA #2 amp, use I2S channel #1, power on and amp on

    source = 1;

  }
  else{
    a2dp_sink.end();
    digitalWrite(ledPin1, LOW);
    delay(100);

    TFA_setDeviceControl(&TFA[0], 1, 1);  //TFA #1 amp, use I2S channel #2, power on and amp on
    TFA_setDeviceControl(&TFA[1], 1, 1);  //TFA #2 amp, use I2S channel #2, power on and amp on

    source = 0;
  }
}