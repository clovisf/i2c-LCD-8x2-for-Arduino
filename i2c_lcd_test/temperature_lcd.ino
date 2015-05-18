/*
 * Sketch for testing sleep mode with wake up on WDT.
 * Donal Morrissey - 2011.
 *
 */
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define LED_PIN (13)

volatile int f_wdt=1;
#include <Wire.h>
 
#define vddPin 16    // ArduinoA2
#define gndPin 17    // ArduinoA3
#define sdaPin 18    // ArduinoA4
#define sclPin 19    // ArduinoA5
#define I2Cadr 0x3e  // Fixed
byte contrast = 35;  // Contrast (0～63)
boolean contrastFlag = false;
const int led = 13;
const int sensorPin = A0; 
word temperature = 0;



/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
  }
  else
  {
    Serial.println("WDT Overrun!!!");
  }
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_SAVE for lowest power consumption. */
  
  
    power_adc_disable();
power_spi_disable();
power_timer0_disable();
power_timer1_disable();
power_timer2_disable();
power_twi_disable();


  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}



/***************************************************
 *  Name:        setup
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Setup for the serial comms and the
 *                Watch dog timeout. 
 *
 ***************************************************/
void setup()
{
  

  /*** Setup the WDT ***/
  
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  

  /* set new watchdog timeout prescaler value */
  //WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  //WDTCSR = 1<<WDP1 | 1<<WDP2; /* 1.0 second */
  WDTCSR = 1<<WDP1 | 1<<WDP2 | 1<<WDP3; /* 2.0 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);
  pinMode(vddPin, OUTPUT);
  digitalWrite(vddPin, HIGH);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  String str;
  delay(500);
  Wire.begin();
  lcd_cmd(0b00111000); // function set
  lcd_cmd(0b00111001); // function set
  lcd_cmd(0b00000100); // EntryModeSet
  lcd_cmd(0b00010100); // interval osc
  lcd_cmd(0b01110000 | (contrast & 0xF)); // contrast Low
  lcd_cmd(0b01011100 | ((contrast >> 4) & 0x3)); // contast High/icon/power
  lcd_cmd(0b01101100); // follower control
  delay(200);
  lcd_cmd(0b00111000); // function set
  lcd_cmd(0b00001100); // Display On
  lcd_cmd(0b00000001); // Clear Display
  delay(2);
  
  }



/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Main application loop.
 *
 ***************************************************/
void loop()
{
  if(f_wdt == 1)
  {
     
    //digitalWrite(led, !digitalRead(led));
    digitalWrite(led, HIGH);
    temperature= analogRead(sensorPin);
    temperature= temperature/20.48;
    //char __temperature[sizeof(temperature)]; // not working
    
    int i = 42;
    char buf[12];
    //sprintf(buffer, "   ", i);
    
    
        
    
    /* Toggle the LED */
      lcd_setCursor(0, 0); // Coluna, linha
  lcd_printStr("temp(C):");
  lcd_setCursor(3, 1); // Coluna, linha
  //lcd_printStr(__temperature); // not working
  lcd_printStr(itoa(temperature, buf, 10)); // not working
  /*
  if (contrastFlag == false) {
    if (++contrast >= 63) {
      contrastFlag = true;
    }
  } else {
    if (--contrast <= 0) {
      contrastFlag = false;
    }
  }
  */
  lcd_setContrast(contrast);
  delay(100);
    
    /* Don't forget to clear the flag. */
    f_wdt = 0;
    
    /* Re-enter sleep mode. */
    digitalWrite(led, LOW);
    enterSleep();
  }
  else
  {
    /* Do nothing. */
  }
  
}

void lcd_cmd(byte x) {
  Wire.beginTransmission(I2Cadr);
  Wire.write(0b00000000); // CO = 0,RS = 0
  Wire.write(x);
  Wire.endTransmission();
}
 
void lcd_contdata(byte x) {
  Wire.write(0b11000000); // CO = 1, RS = 1
  Wire.write(x);
}
 
void lcd_lastdata(byte x) {
  Wire.write(0b01000000); // CO = 0, RS = 1
  Wire.write(x);
}
 
// Show the string.
void lcd_printStr(const char *s) {
  Wire.beginTransmission(I2Cadr);
  while (*s) {
    if (*(s + 1)) {
      lcd_contdata(*s);
    } else {
      lcd_lastdata(*s);
    }
    s++;
  }
  Wire.endTransmission();
}
 
// Set the character location.
void lcd_setCursor(byte x, byte y) {
  lcd_cmd(0x80 | (y * 0x40 + x));
}
void lcd_setContrast(byte c) {
  lcd_cmd(0x39);
  lcd_cmd(0b01110000 | (c & 0x0f)); // contrast Low
  lcd_cmd(0b01011100 | ((c >> 4) & 0x03)); // contast High/icon/power
  lcd_cmd(0x38);
}
