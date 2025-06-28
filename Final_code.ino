#include <LiquidCrystal_I2C.h>

//defining sensor pins
const int VOLTMETER = A0; //voltmeter input pin
const int TEMPERATURE = A1; //temperature sensor input pin
const int CURRENTMETER = A2; //currentmeter input pin
const int RESISTANCEMETER = A3; //resistancemeter input pin

//defining button pins
const int VOLT_BUTTON = 2; //voltmeter button
const int CURRENT_BUTTON = 3; //currentmeter button
const int RES_BUTTON = 4; //resistancemeter button
const int TEMP_BUTTON = 5; //temperature sensor button
const int RELAY = 6; //relay switch pin

//defining unused pins as input with pullup resistors
const int PIN_PD7 = 7; //unused pin as input pullup resistor
const int PIN_PB0 = 8; //unused pin as input pullup resistor
const int PIN_PB1 = 9; //unused pin as input pullup resistor
const int PIN_PB2 = 10; //unused pin as input pullup resistor
const int PIN_PB3 = 11; //unused pin as input pullup resistor
const int PIN_PB4 = 12; //unused pin as input pullup resistor

//defining other constants
const float V_REF = 6.06; //reference voltage level
const float SHUNT_RES = 1.30; //reference shunt resistor R3
const float REF_RES = 9930.00; //serial resistor R8 for the resistancemeter
const int MENU_PERIOD = 500; //disable first button for 500 ms
const float IDEAL_LEVEL = 7.5; //defining minimum level of voltage on the battery

//defining constant resistor values
const float R1 = 9920.00;
const float R2 = 3270.00;
const float R4 = 993.00;
const float R5 = 5560.00;

//global variables
int flag = 0;
float result;
unsigned long menu_timer = 0;


LiquidCrystal_I2C lcd(0x27, 4, 20); //0x27 address of the LCD in hexa


float calculate_voltage()
{
  float resistor_ratio;
  int sensorValue = 0;
  float voltage;
  float v;
  resistor_ratio = (R2 / (R1 + R2));
  for (int i = 0; i < 20 ; i++) //calculating the average analog voltage by taking 20 measurments
    {
    sensorValue += analogRead(VOLTMETER);
    delay(3);
    }
  voltage = sensorValue * V_REF / (1023.00 * 20.00);
  v = voltage / resistor_ratio; //calculate the voltage
  return v;
}

float calculate_current()
{
  float current_gain;
  float voltage;
  int sensorValue = 0;
  float c;
  current_gain = 1+(R5/R4);
  for (int i = 0; i < 20 ; i++) //calculating the average analog voltage by taking 20 measurments
    {
    sensorValue += analogRead(CURRENTMETER);
    delay(3);
    }
  voltage = sensorValue * V_REF / (1023.00 * current_gain * 20.00); // Convert to voltage
  c = voltage * 1000.00 / SHUNT_RES; //calculate the current in mA
  return c;
}

float calculate_resistance()
{
  float voltage;
  float r;
  int sensorValue = 0;
  for (int i = 0; i < 20 ; i++) //calculating the average analog voltage by taking 20 measurments
    {
    sensorValue += analogRead(RESISTANCEMETER);
    delay(3);
    }
  voltage = sensorValue * V_REF / (1023.00 * 20.00); // Convert to voltage
  r = voltage * REF_RES / (V_REF - voltage); //calculate the resistance
  return r;
}

float temperature_measurement()
{
  float tempc; //variable to store temperature in degree Celsius
  int sensorValue = 0;
  for (int i = 0; i < 20 ; i++) //calculating the average analog voltage by taking 20 measurments
    {
    sensorValue += analogRead(TEMPERATURE);
    delay(3);
    }
  tempc = (sensorValue * V_REF) / (1023.00 * 20.00 * 0.01) - 68.00; //storing value in Degree Celsius
  return tempc;
}

float battery()
{
  float battery_level;
  digitalWrite(RELAY, HIGH);
  battery_level = calculate_voltage();
  digitalWrite(RELAY, LOW);
  return battery_level;
}

void menu()
{
  lcd.clear();
  lcd.setCursor(0,0);lcd.print("Press 1 - Temp."); //display choice
  lcd.setCursor(0,1);lcd.print("Press 2 - Resistance"); //display choice
  lcd.setCursor(0,2);lcd.print("Press 3 - Current"); //display choice
  lcd.setCursor(0,3);lcd.print("Press 4 - Voltage"); //display choice
}


void setup()
{
  //initializing the LCD  
  lcd.init(); //initializing LCD
  lcd.backlight(); //turning the backlight on for the LCD

  //initializing sensor pins
  pinMode(VOLTMETER, INPUT); //configuring sensor pin as input for voltmeter
  pinMode(TEMPERATURE, INPUT); //configuring sensor pin as input for temperature sensor
  pinMode(CURRENTMETER, INPUT); //configuring sensor pin as input for currentmeter
  pinMode(RESISTANCEMETER, INPUT); //configuring sensor pin as input for resistancemeter
  pinMode(RELAY, OUTPUT); //configuring pin as output for relay

  //initializing button pins
  pinMode(VOLT_BUTTON, INPUT_PULLUP); //configuring button pin as input with pullup resistor for voltmeter
  pinMode(CURRENT_BUTTON, INPUT_PULLUP); //configuring button pin as input with pullup resistor for currentmeter
  pinMode(RES_BUTTON, INPUT_PULLUP); //configuring button pin as input with pullup resistor for resistancemeter
  pinMode(TEMP_BUTTON, INPUT_PULLUP); //configuring button pin as input with pullup resistor for temperature sensor

  //initializing unused pins as input with pullup resistors
  pinMode(PIN_PD7, INPUT_PULLUP); //configuring pin as input with pullup resistor
  pinMode(PIN_PB0, INPUT_PULLUP); //configuring pin as input with pullup resistor
  pinMode(PIN_PB1, INPUT_PULLUP); //configuring pin as input with pullup resistor
  pinMode(PIN_PB2, INPUT_PULLUP); //configuring pin as input with pullup resistor
  pinMode(PIN_PB3, INPUT_PULLUP); //configuring pin as input with pullup resistor
  pinMode(PIN_PB4, INPUT_PULLUP); //configuring pin as input with pullup resistor

  //welcome message
  lcd.setCursor(7,1);
  lcd.print("WELCOME!");
  lcd.setCursor(0, 2);
  lcd.print("BSC GROUP MULTIMETER");
  delay(3000);
  lcd.clear();
  delay(1000);
  
  //battery level check
  if(battery() >= IDEAL_LEVEL)
  {
    lcd.setCursor(5,1);
    lcd.print("BATTERY OK");
    delay(3000);
    lcd.clear();
    delay(1000);
  }
  else
  {
    lcd.setCursor(5,1);
    lcd.print("BATTERY LOW");
    delay(3000);
    lcd.clear();
    delay(1000);
  }

  menu();
}


void loop()
{
  if(digitalRead(TEMP_BUTTON) == LOW)
  {
    if(millis() >= menu_timer)
    {  
      if(flag != 0)
      {
        menu();
        flag = 0;
      }
      else
      {
        flag = 1;
        lcd.clear();
      }
      menu_timer = millis() + MENU_PERIOD;
    }
  }
  if(digitalRead(RES_BUTTON) == LOW)
  {
    if(flag == 0)
    {
      flag = 2;
      lcd.clear();
    }
  }
  if(digitalRead(CURRENT_BUTTON) == LOW)
  {
    if(flag == 0)
    {
      flag = 3;
      lcd.clear();
    }
  }
  if(digitalRead(VOLT_BUTTON) == LOW)
  {
    if(flag == 0)
    {
      flag = 4;
      lcd.clear();
    }
  }
  if((digitalRead(RES_BUTTON) == LOW) && (digitalRead(CURRENT_BUTTON) == LOW))
  {
    if((flag == 2) || (flag == 3))
    {
      flag = 10;
      lcd.clear();
    }
  }

  switch (flag)
  {
    case 1:
      result = temperature_measurement();
      lcd.setCursor(0,0);
      lcd.print("Press 1 - Menu");
      // Display the measured temperature on the LCD
      lcd.setCursor(0, 2);
      lcd.print("Temperature (");
      lcd.print((char)223);
      lcd.print("C):");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(result); // Display temperature
    break;

    case 2:
      result = calculate_resistance();
      lcd.setCursor(0,0);
      lcd.print("Press 1 - Menu");
      // Display the measured resistance on the LCD
      lcd.setCursor(0, 2);
      lcd.print("Resistance (Ohm):");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      if(result > 150000)
      {
        lcd.print("inf");
      }
      else
      {
        lcd.print(result); // Display resistance
      }
    break;

    case 3:
      result = calculate_current();
      lcd.setCursor(0,0);
      lcd.print("Press 1 - Menu");
      // Display the measured current on the LCD
      lcd.setCursor(0, 2);
      lcd.print("Current (mA):");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(result); // Display current
    break;

    case 4:
      result = calculate_voltage();
      lcd.setCursor(0,0);
      lcd.print("Press 1 - Menu");
      // Display the measured voltage on the LCD
      lcd.setCursor(0, 2);
      lcd.print("Voltage (V):");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(result); // Display voltage
    break;

    case 10:
      result = battery();
      lcd.setCursor(0,0);
      lcd.print("Press 1 - Menu");
      // Display the measured voltage of the battery on the LCD
      lcd.setCursor(0, 2);
      lcd.print("Battery level (V):");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print(result); // Display voltage of the battery
    break;
  }
}