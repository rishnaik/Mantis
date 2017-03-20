
/*

   Libraries Included / Reference :
   - https://github.com/PaulStoffregen/Time
   - http://www.circuitstoday.com/tachometer-using-arduino
   - https://www.arduino.cc/en/reference/SD

*/


#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define NUMSAMPLES 20
#define VIN 5.05
#define MOTOR_C_DIR -1


// which analog pin to connect
const byte Wheel_pin = A6, Motor_Temp_pin = A3, Motor_C_pin = A4, Motor_V_pin = A5, Motor_RPM_pin = 2, Wheel_RPM_pin = 3; // Analog input pin that the potentiometer is attached to
const byte SD_SPI_CSpin = 53;
int Motor_C_Offset = 0; // usually 2465
unsigned long Motor_RPM = 0, Wheel_RPM = 0;
long debouncing_time = 2; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
float Motor_Temp = 0, Motor_C = 0, Motor_V = 0;
char i = 0;
float Motor_Temp_steinhart;
float Motor_Temp_avg = 0.0;
float Motor_V_avg = 0, Motor_C_avg = 0;


float Motor_RPM_kmph = 0.0, Motor_Power = 0.0, Wheel_RPM_kmph = 0.0;

volatile int Motor_RPM_rev = 0, Wheel_RPM_rev = 0;
float Motor_RPM_rev_temp = 0.0, Wheel_RPM_rev_temp = 0.0;
unsigned long timer = 0, oldtime = 0;

/*          error code        */
//
//error code  meaning
// 0000 0000        No error
// 0000 0001        Error Writing File
//
int error_code = B00000000;


// make a string for assembling the data to log:
String dataString = "";
String dataTitle = "";

File dataFile;
LiquidCrystal_I2C  lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the I2C bus address for an unmodified backpack


const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);




void Motor_RPM_irs() {
  /* if((long)(micros() - last_micros) >= debouncing_time*1000) {
     rev++;
     last_micros = micros();
    }*/
  Motor_RPM_rev++;
}


void Wheel_RPM_irs() {
  /* if((long)(micros() - last_micros) >= debouncing_time*1000) {
     rev++;
     last_micros = micros();
    }*/
  Wheel_RPM_rev++;
}



//          Function to return averaged Analog pin readings

float ReadAnalog(byte port, byte samplenos) {
  int j = 0;
  float avg = 0.0;
  do {
    avg += analogRead(port);
    delay(2);
    j++;
  } while (j < samplenos);
  avg /= samplenos;
  return (avg);
}

//          Function to print on Serial and LCD
void FormatPrint(String input, int row, int column, int maxlength) {
  int lengthofstring = input.length(), i;
  if (lengthofstring <= maxlength) {
    lcd.setCursor(column, row);
    for (i = 0; i < (maxlength - lengthofstring); i++) lcd.print("0");
    lcd.print(input);
  } else lcd.print("X");
}

void PrintSerial() {
  uint8_t degree[8]  = {0x6, 0x9, 0x9, 0x6, 0x0, 0x0, 0x0};
  lcd.createChar(0, degree);

  /*            Serial Printin Start        */
  Serial.print("RPM=");
  Serial.print(Motor_RPM);
  Serial.print("\t");
  Serial.print("Motor Voltage = ");
  Serial.print(Motor_V);
  Serial.print("\t");
  Serial.print("Motor Current = ");
  Serial.print(Motor_C, 0);
  Serial.print("\t");
  Serial.print("Motor Temp = ");
  Serial.println(Motor_Temp, 1);

  lcd.clear();
  lcd.setCursor(7, 2);
  if (Motor_RPM == 0) {
    lcd.print("0000");
  } else if (Motor_RPM < 10) {
    lcd.print("000");
    lcd.print(Motor_RPM, 0);
  }
  else if (Motor_RPM < 100) {
    lcd.print("00");
    lcd.print(Motor_RPM, 0);
  }
  else if (Motor_RPM < 1000) {
    lcd.print("0");
    lcd.print(Motor_RPM, 0);
  } else lcd.print(Motor_RPM, 0);
  lcd.print("RPM");


  lcd.setCursor(7, 1);
  if (Wheel_RPM_kmph < 10) {
    lcd.print("00");
    lcd.print(Motor_RPM_kmph, 0);
  }
  else if (Wheel_RPM_kmph < 100) {
    lcd.print("0");
    lcd.print(Wheel_RPM_kmph, 0);
  }
  else lcd.print(Wheel_RPM_kmph, 0);
  lcd.print(" km/h");

  /*lcd.setCursor(8,3);
    lcd.print(Motor_RPM);
    //lcd.print("RPM");*/


  lcd.setCursor(0, 0);
  if (Motor_Power > -0.2 && Motor_Power < 0.2) {
    lcd.print("0.00");
  }
  else lcd.print(Motor_Power, 2);
  lcd.print("kW");
  lcd.setCursor(0, 3);
  lcd.print(Motor_V, 1);
  lcd.print("V");


  lcd.setCursor(13, 3);
  if (Motor_Temp < 100) {
    lcd.print(" ");
    lcd.print(Motor_Temp, 1);
  }
  else lcd.print(Motor_Temp, 1);
  lcd.write(byte(0));
  lcd.print("C");



  lcd.setCursor(18, 0);
  lcd.print("0");
  if (Motor_C < 1 && Motor_C > -1) {
    lcd.setCursor(18, 0);
    lcd.print("0");
  }
  else if (Motor_C < 0) {
    if (Motor_C > -10) {
      lcd.setCursor(18, 0);
      lcd.print(Motor_C, 0);
    } else if (Motor_C > -100) {
      lcd.setCursor(17, 0);
      lcd.print(Motor_C, 0);
    } else if (Motor_C > -1000) {
      lcd.setCursor(16, 0);
      lcd.print(Motor_C, 0);
    }
  }

  else {
    if (Motor_C < 10) {
      lcd.setCursor(18, 0);
      lcd.print(Motor_C, 0);
    } else if (Motor_C < 100) {
      lcd.setCursor(17, 0);
      lcd.print(Motor_C, 0);
    } else if (Motor_C < 1000) {
      lcd.setCursor(16, 0);
      lcd.print(Motor_C, 0);
    }
  }
  lcd.setCursor(19, 0);
  lcd.print("A");
}


//          Initialiszation of all parameters
void setup() {
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_32;

  Motor_C_avg = ReadAnalog(Motor_C_pin, NUMSAMPLES);
  Motor_C_Offset = (( Motor_C_avg / 1023.0) * VIN * 1000.0);


  pinMode(Motor_RPM_pin, INPUT);
  pinMode(Wheel_RPM_pin, INPUT);

  // initialize serial communications at 115200 bps:
  Serial.begin(115200);
  Serial.println("Initializing..");

  lcd.begin (20, 4); // for 20 x 4 LCD module
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(1);

  if (!SD.begin(SD_SPI_CSpin)) {
    Serial.println("Card failed, or not present");
    error_code = B00000001;
    // don't do anything more:
    return;
  }
  setTime(14, 44, 30, 07, 02, 2017);


  if (!SD.exists("datalog.csv"))
  {
    dataFile = SD.open("datalog.csv", FILE_WRITE);
    dataTitle = "Date/Time,Time_Elapsed,Motor_RPM,Motor_Voltage,Motor_Current,Motor_Temp";
    dataFile.println(dataTitle);
    dataFile.close();
  }

  Serial.println("card initialized.");

  attachInterrupt(digitalPinToInterrupt(Motor_RPM_pin), Motor_RPM_irs, RISING); //attaching the Motor RPM interrupt
  attachInterrupt(digitalPinToInterrupt(Wheel_RPM_pin), Wheel_RPM_irs, RISING); //attaching the Motor RPM interrupt

}


void loop() {

  /*                     RPM Count                */
  delay(500);
  detachInterrupt(digitalPinToInterrupt(Motor_RPM_pin));           //detaches the Motor RPM interrupt
  detachInterrupt(digitalPinToInterrupt(Wheel_RPM_pin));           //detaches the Wheel RPM interrupt
  timer = millis() - oldtime;    //finds the time
  Motor_RPM_rev_temp = Motor_RPM_rev;
  Wheel_RPM_rev_temp = Wheel_RPM_rev;
  Motor_RPM = ((Motor_RPM_rev_temp / timer) * 60000.0) / 4.0 ; //calculates Motor rpm
  Serial.print("Motor_rev:"); Serial.print(Motor_RPM_rev); Serial.print(" timer:"); Serial.print(timer); Serial.print("Motor RPM:"); Serial.print(Motor_RPM);
  Wheel_RPM = ((Wheel_RPM_rev_temp / timer) * 60000.0) / 6.0 ; //calculates Wheel rpm
  Serial.print(" Wheel_rev:"); Serial.print(Wheel_RPM_rev); Serial.print(" timer:"); Serial.print(timer); Serial.print("Wheel RPM:"); Serial.println(Motor_RPM);



  /*                       Voltage sensor             */
  //Motor_V_avg = 0;
  Motor_V_avg = ReadAnalog(Motor_V_pin, NUMSAMPLES);;
  Motor_V = (((Motor_V_avg) / 1023.0) * VIN * 12.65) + 0.42 ; // Motor_V in V



  // 12.65 is the conversion factor (C.f) for the voltage divider (R1=229500,R2=19700,Vin=60) + Diode V loss = 0.42
  //Serial.print(analogRead(Motor_V_pin));

  /*                       Motor Temp sensor             */
  Motor_Temp_avg = ReadAnalog(Motor_Temp_pin, NUMSAMPLES);;
  Motor_Temp_avg = 1023.0 / Motor_Temp_avg - 1;
  Motor_Temp_avg = 9765.0 / Motor_Temp_avg; //Thermistor Series Resistor value = 9765
  Motor_Temp_steinhart = Motor_Temp_avg / 10000.0; // (R/Ro) -> Ro Nominal Resistance of thermistor at 25C = 10000
  Motor_Temp_steinhart = log(Motor_Temp_steinhart);                  // ln(R/Ro)
  Motor_Temp_steinhart /= 3950.0;                   // 1/B * ln(R/Ro) where B is the beta coefficient of the thermistor (usually 3000-4000) = 3950
  Motor_Temp_steinhart += 1.0 / (25.0 + 273.15); // + (1/To) -> To is the Nominal Temperature = 25
  Motor_Temp_steinhart = 1.0 / Motor_Temp_steinhart;                 // Invert
  Motor_Temp = Motor_Temp_steinhart - 273.15;                       // convert to C


  /*                       Battery Current sensor             */
  Motor_C_avg = ReadAnalog(Motor_C_pin, NUMSAMPLES);
  //Serial.println(( Motor_C_avg / 1023.0) * VIN * 1000.0);
  Motor_C = (Motor_C_Offset - (( Motor_C_avg / 1023.0) * VIN * 1000.0)) * MOTOR_C_DIR / 10; //Motor_C in x0.1A

  // 4730 is the C.f for the 200A current sensor
  //Further to be processed with Motor Amps = (Motor_C - OFFSET) /10
  // where 'OFFSET' = "Motor_C reading when V_BMS = 0" (@Vcc=4.95V it was 2348)


  /*                       Sensor Reading end        */


  /*                    Derived Calculations          */
  Motor_RPM_kmph = (((Motor_RPM / 4.67) * 0.6 * 3.14) / 60.0) * (18.0 / 5.0);  // Motor Speed in km/hr
  Motor_Power = Motor_C * Motor_V / 1000.0;                                // Power in kW
  Wheel_RPM_kmph = ((Wheel_RPM  * 0.54 * 3.14) / 60.0) * (18.0 / 5.0);  // Wheel Speed in km/hr
  //Serial.print("Wheel_RPM_")
  PrintSerial();




  /*            SD Card write           */


  time_t t = now(); // store the current time in time variable t
  dataString = String(day(t)) + "_" + String(month(t)) + "_" + String(year(t)) + " " + String(hour(t)) + ":" + String(minute(t)) + ":" + String(second(t))  + "," + String(Motor_RPM) + "," + String(Motor_V) + "," + String(Motor_C, 0) + "," + String(Motor_Temp, 1);
  dataFile = SD.open("datalog.csv", FILE_WRITE);


  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  /*          Re-enabling the RPM timer         */
  Motor_RPM_rev++;
  Wheel_RPM_rev = 0;
  oldtime = millis();           //saves the current time
  attachInterrupt(digitalPinToInterrupt(Motor_RPM_pin), Motor_RPM_irs, RISING); //attaching the Motor RPM interrupt
  attachInterrupt(digitalPinToInterrupt(Wheel_RPM_pin), Wheel_RPM_irs, RISING); //attaching the Wheel RPM interrupt
}
