
// LIBRARIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <LiquidCrystal.h> // LCD Display
#include <Stepper.h> // Stepper Motor

// GLOBAL VARIABLES & MACROS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* NOTE -> each state has an assigned integer for ease of programming.
 *  see below table for guide. */
#define DISABLED 0
#define IDLING 1
#define ERRORSTATE 2
#define RUNNING 3

int state;
int currentTime; // 4-digit 24-hour format, 0000 = 12:00AM, 1400 = 2:00PM
const char* currentDate; // const string, e.g. "date/date/date"
int runCount;

// Water Sensor
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0; // USART 0A Control and Status Register
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1; // USART 0B Control and Status Register
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2; // USART 0C Control and Status Register
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4; // USART 0 Baud Rate Register Low Byte
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6; // USART I/O Data Register
 
volatile unsigned char *my_ADMUX    = (unsigned char*) 0x7C; // ADC Multiplexer Selection Register
volatile unsigned char *my_ADCSRB   = (unsigned char*) 0x7B; // ADC Control and Status Register B
volatile unsigned char *my_ADCSRA   = (unsigned char*) 0x7A; // ADC Control and Status Register A
volatile unsigned int  *my_ADC_DATA = (unsigned int*)  0x78; // ADC Data Register

volatile unsigned char* port_a = (unsigned char*) 0x22; // Port A Data Register
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; // Port A Direction Register

const float maxWtrVoltage = 5.00; // max voltage expected from water sensor

// LED # Codes
#define RED 3
#define YLW 2
#define GRN 1
#define BLU 0

// NOTE - > all LEDs on Port C, 0 thru 3 (pins D34 - 37)
volatile unsigned char* port_c = (unsigned char*) 0x28; // LED Port
volatile unsigned char* ddr_c = (unsigned char*) 0x27;

// Stepper Motor
#define IN1 22 // ULN2003 Stepper Motor Driver Pins
#define IN2 24
#define IN3 26
#define IN4 28

const int stepsPerRevolution = (2048 / 2);  // change this to fit the number of steps per revolution, or set a max range of motion (2048 = 1 revolution for the 28BYJ-48 Stepper Motor)
const float maxPotentVoltage = 5.00; // max voltage expected from potentiometer
int motorPosition;

// LCD and Temperature/Humidity Sensor
float humid = 25.50; // test
float temp = 12.87; //test
// Fan Motor

// FUNCTION INITIALIZATIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void reportStateChange(char);

float checkWtr(void); // check voltage readout of water sensor
// NOTE -> Sensor not-submerged: <0.7V, partially submerged ~0.7V, fully submerged ~0.9V, threshold ~0.75V

void LEDon(int);
void LEDoff(int);
void allLEDoff(void);

void lcdDisplayError(void);
void lcdDisplayTempHumid(float, float);
void wipeLCD(void);

int updateVentPosition(void); // change vent position according to potentiometer setting and return steps changed
int getVentPosition(void); // return stepper motor position
void reportVentChange(int); // print changes to vent steps

// SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  // setup the UART and Serial Monitor
  U0init(9600);
  delay(1000); // Delay to allow time for serial monitor to open
  printString("Setup Block\n");
  
  // setup the ADC
  adc_init();

  // Vent
  Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4); // establishing signal pins for use by stepper motor driver componenent
  myStepper.setSpeed(5); // step motor settings
  myStepper.step(0);

  // Water Sensor
  *ddr_a |= 0b00000010; // Water sensor power set to output

  // LEDs
  *ddr_c |= 0b00001111; // LED pins set to output
  allLEDoff();
  
  // LCD
  LiquidCrystal lcd(52, 53, 48, 49, 46, 47);
  /*  ^^^ establishing pins for use by LCD in the following order:
   *  lcd(RS, E, DB4, DB5, DB6, DB7) - 4 data lines 
   *  for other syntaxes, see documentation for library:
   *  https://www.arduino.cc/reference/en/libraries/liquidcrystal/
   */
  lcd.begin(16, 2); //Tell the LCD that it is a 16x2 LCD
  temp = 72.5911;
  humid = 25.8744;

  // Misc
  currentTime = 2401;
  currentDate = "13/31/2022";
  state = IDLING; // Set initial state for test purpose

  printString("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\nMain Loop\n");
}

// MAIN LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {

  switch (state) {
  /* NOTE ->  FOR ALL STATES, realtime clock must be used to report via serial port the
  * time of each state transition and any changes to the stepper motor
  * position for the vent. FOR ALL STATES EXCEPT FOR DISABLED, temp and humid
  * is continuously monitored and reported to LCD once per minute, system responds to
  * changes in vent position control, and the stop button should turn fan motor off (if on) and
  * system should go to DISABLED state.*/
  
  case DISABLED : // 0 * * * * * * * * * * * * * *
  reportStateChange("DISABLED");
  while(state == DISABLED){
    LEDon(YLW);
    fanTemp(0.00); // FAN to OFF
    // START button monitored by ISR -> IDLE
    reportVentChange(updateVentPosition()); // always reports 0 in DISABLED state
    if(state != DISABLED){ // check if need to change state or continue running
      allLEDoff();
      break;
    }
  }
  break;

  case IDLING : // 1 * * * * * * * * * * * * * *
  reportStateChange("IDLING");
  while(state == IDLING){
    LEDon(GRN);
    if(checkWtr() <= 0.75){ // check water level, if voltage is below threshold change to Errorstate
      state = ERRORSTATE;
      allLEDoff();
      break;
    }
    reportVentChange(updateVentPosition());
    // START/STOP button turns off fan (if on) and change to DISABLED
    if(state != IDLING){ // check if need to change state or continue running
      allLEDoff();
      break;
    }
  }
  break;

  case ERRORSTATE : // 2 * * * * * * * * * * * * * *
  reportStateChange("ERRORSTATE");
  while(state == ERRORSTATE){
    LEDon(RED);
    // Motor is OFF and should not start
    // RESET button should trigger change to IDLE if water is good (checkWtr() > 0.6 [partial] OR > 8.0 [full])
    lcdDisplayError();
    // check reset button
    if(checkWtr() > 0.75){ // check water level, if voltage is above threshold change to Idling
      state = IDLING;
      allLEDoff();
      break;
    }
    lcdDisplayTempHumid(temp, humid); // once per minute
    reportVentChange(updateVentPosition());
    if(state != ERRORSTATE){ // check if need to change state or continue running
      allLEDoff();
      break;
    }
  }
  break;

  case RUNNING : // 3 * * * * * * * * * * * * * *
  reportStateChange("RUNNING");
  while(state == RUNNING){
    LEDon(BLU);
    if(checkWtr() <= 0.75){ // check water level, if voltage is below threshold change to Errorstate
      state = ERRORSTATE;
      allLEDoff();
      break;
    }
    lcdDisplayTempHumid(temp, humid); // once per minute
    reportVentChange(updateVentPosition());
    // blue LED on, all other LEDs off
    // fan motor on
    // stop button turns off fan (if on) and change to DISABLED
    // change to IDLE if temp drops below threshold
    if(state != RUNNING){ // check if need to change state or continue running
      allLEDoff();
      break;
    }
  }
  break;
 
  } // End switch-case block
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// MISC FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void reportStateChange(const char* stateIn){ // check if state switch is needed and return state #
  printString("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
  printString("State switched to ");
  printString(stateIn);
  printTimeReport();
}

void printTimeReport(void){
  Serial.print(" at ");
  Serial.print(currentTime);
  Serial.print(" on ");
  Serial.println(currentDate);
}

void printString(const char* stringIn){
  const char* text = stringIn;
  for(int i = 0; text[i] != '\0'; i++){
    U0putchar(text[i]);
  }
}

void printInt(int out_num)
{
  // clear a flag (for printing 0's in the middle of numbers)
  unsigned char print_flag = 0;
  // if its less than 0
  if(out_num < 0)
  {
    // get the 1000's digit, add to '0' 
    U0putchar('-');
    // set the print flag
    print_flag = 1;
    // mod the out num by 1000
    out_num = abs(out_num);
  }
  // if its greater than 1000
  if(out_num >= 1000)
  {
    // get the 1000's digit, add to '0' 
    U0putchar(out_num / 1000 + '0');
    // set the print flag
    print_flag = 1;
    // mod the out num by 1000
    out_num = out_num % 1000;
  }
  // if its greater than 100 or we've already printed the 1000's
  if(out_num >= 100 || print_flag)
  {
    // get the 100's digit, add to '0'
    U0putchar(out_num / 100 + '0');
    // set the print flag
    print_flag = 1;
    // mod the output num by 100
    out_num = out_num % 100;
  } 
  // if its greater than 10, or we've already printed the 10's
  if(out_num >= 10 || print_flag)
  {
    U0putchar(out_num / 10 + '0');
    print_flag = 1;
    out_num = out_num % 10;
  } 
  // always print the last digit (in case it's 0)
  U0putchar(out_num + '0');
}

// LCD FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// NOTE -> 16 Characters per line
void lcdDisplayError(){ // Displays low water error message to LCD
  lcd.clear();
  lcd.setCursor(0, 0); //Set invisible cursor to first character, first line (x = 0, y = 0)
  lcd.print("     ERROR"); // Output text at cursor
  lcd.setCursor(0, 1); // Set invisible cursor to first character, second line (x = 0, y = -1)
  lcd.print(" Low Water Level");
  delay(3000); // Display time
  lcd.clear(); // Wipe display
  
  lcd.setCursor(0, 0); // Reset cursor
  lcd.print("  Refill Water");
  lcd.setCursor(0, 1);
  lcd.print("  & Press Reset");
  delay(3000);
  
  lcd.setCursor(0, 0);
}

void lcdDisplayTempHumid(float t, float h){ // Displays atmospheric conditions to LCD
  // NOTE -> temp (t) max value is XXXXX.XX, humidity (h) max value is XX.XX
  // Alternatively, t max = XX.XXX... and h max = XX.XXX... will be correct but with rounding error
  lcd.clear();
  lcd.setCursor(0, 0); //Set invisible cursor to first character, first line (x = 0, y = 0)
  lcd.print("Temp: "); lcd.print(t); lcd.print("*C"); // Output "Temp: X*F" at cursor
  lcd.setCursor(0, 1); // Set invisible cursor to first character, second line (x = 0, y = -1)
  lcd.print("Humidity: "); lcd.print(h); lcd.print("%");
  delay(3000); // Display for time
}

void wipeLCD(){
  lcd.clear();
}

// WATER SENSOR FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// NOTE -> Do not provide continuous voltage to water sensor, will result in corrosion
// NOTE -> Sensor not-submerged: <0.6V, partially submerged ~0.7V, fully submerged ~0.9V
float checkWtr(void){
  *port_a |= 0b00000010; //apply voltage to sensor, pin D23 set HIGH
  delay(500); //allow voltage to stabilise
  unsigned int adc_reading = adc_read(0); //get the reading from the ADC, analog input pin A0
  *port_a &= 0b11111101; //cut off voltage to sensor pin D23 set LOW
  float wtrVoltage = adc_reading * (maxWtrVoltage / 1023.0); //convert 0-1023 analog signal to voltage
  return wtrVoltage;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to set prescaler selection to slow reading
  
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111;// clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111;// clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11011111;// clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

// LED FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void LEDon(int num){
  switch (num){
    case BLU: // 0
      *port_c |= 0b00000001;
      break;
      
    case GRN: // 1
      *port_c |= 0b00000010;
      break;
      
    case YLW: // 2
      *port_c |= 0b00000100;
      break;
      
    case RED: // 3
      *port_c |= 0b00001000;
      break;
  }
}

void LEDoff(int num){
  switch (num){
    case BLU: // 0
      *port_c &= 0b11111110;
      break;
      
    case GRN: // 1
      *port_c &= 0b11111101;
      break;
      
    case YLW: // 2
      *port_c &= 0b11111011;
      break;
      
    case RED: // 3
      *port_c &= 0b11110111;
      break;
  }
}

void allLEDoff(void){
  *port_c &= 0b11110000;
}

// STEPPER MOTOR/POTENTIOMETER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int updateVentPosition(void){ // change vent position according to potentiometer setting and return steps changed
  int potentValue;
  int desiredMotorPosition;
  int motorChange;

  if(state != DISABLED){ // if NOT in disabled state, change vent according to potentiometer and report step changes to vent
    printString("* * * Reading potentiometer voltage... * * *\n"); // For test purposes
    delay(1000); // allow time for setting to be selected
    potentValue = analogRead(A8);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - max V):
    float potentVoltage = potentValue * (maxPotentVoltage / 1023.0);

    //convert potentiometer voltage to changes in motor (0V = 0, max V = stepsPerRevolution):
    if(potentVoltage < 1){ // if voltage is less than 1.00V, full close vent
      desiredMotorPosition = 0;
    } else if(potentVoltage > 4){ // if voltage greater than 4.00V, full open vent
      desiredMotorPosition = stepsPerRevolution;
    } else { // otherwise, non-preset angle of vent
      desiredMotorPosition = ((stepsPerRevolution / maxPotentVoltage) * potentVoltage); // linear y = mx + b equation to achieve the 0V = 0 steps and max V = maxSteps behavior
    }
    motorChange = (desiredMotorPosition - motorPosition);
  
    // step motor to desired positon:
    myStepper.step(motorChange);
    motorPosition = motorPosition + motorChange;
    delay(1000); // To prevent motor stress if position is changing rapidly
    return motorChange;
  } else {
    return 0; // if in disabled state, do not read potentiometer and report 0 steps of vent change
  }
  
}

int getVentPosition(void){ // report stepper motor position
  return motorPosition;
}

void reportVentChange(int change){
  if(change != 0){
    printString("Vent changed by ");
    printInt(change);
    printString(" steps");
    printTimeReport();
  } else{
    printString("No vent change detected.\n");
  }
}

// TEMPERATURE/HUMIDTY & FAN FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
