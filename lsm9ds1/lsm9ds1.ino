
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <LiquidCrystal_I2C.h>

// i2c LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

const double Wheeldiam = 0.001289163652040719; //26inch wheel Diameter in Miles 
// i2c
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK 13 //A5 //pin other arduino board
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI 11 //A4//pin other arduino board
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

const int hallPin = 8;
int hallState = 0; 

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

  pinMode(hallPin, INPUT);
}


void setup()
{
  Serial.begin(115200);

  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
}

void loop()
{
  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.println(" m/s^2 ");
  //Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");


  lcd.setCursor(0, 0); // Sets the cursor to col 0 and row 0
  lcd.print("Accel: "); // Prints Sensor Val: to LCD
  lcd.print(a.acceleration.y); // Prints value on Potpin1 to LCD


  //  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  //  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  //  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  lcd.setCursor(0, 1); // Sets the cursor to col 1 and row 0
  //lcd.print("Tilt L/R: "); // Prints Sensor Val: to LCD
  //lcd.print(g.gyro.y); // Prints value on Potpin1 to LCD
  Serial.println();
  double Wspeed = Wheeldiam / double(millisBetweenHallReads()/3600000.0);
  lcd.print("MPH: "); lcd.print(Wspeed);
  Serial.println(Wspeed);
}
bool hallRead()
{
  // read the state of the pushbutton value:
  hallState = digitalRead(hallPin);
  return hallState;
}
unsigned long millisBetweenHallReads()
{
  while(true)
  {
    if(hallRead() == 1)
    {
      unsigned long firstMillis = millis();
      while (hallRead() == 1)
      {
        
      }
      while (hallRead() == 0)
      {
        
      }
      unsigned long secondMillis = millis();
      return (secondMillis - firstMillis);
    }
  }  
}
//void lcdPrint(char arrS[], char arrR[])
//{
//  lcd.setCursor(15,0); // set the cursor to column 15, line 0
//  for (int positionCounter1 = 0; positionCounter1 < 26; positionCounter1++)
//  {
//    lcd.scrollDisplayLeft(); //Scrolls the contents of the display one space to the left.
//    lcd.print(arrS[positionCounter1]); // Print a message to the LCD.
//    delay(tim); //wait for 250 microseconds
//  }
//  lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left  corner.
//  lcd.setCursor(15,1); // set the cursor to column 15, line 1
//  for (int positionCounter = 0; positionCounter < 26; positionCounter++)
//  {
//    lcd.scrollDisplayLeft(); //Scrolls the contents of the display one space to the left.
//    lcd.print(arrR[positionCounter]); // Print a message to the LCD.
//    delay(tim); //wait for 250 microseconds
//  }
//  lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left corner.
//}


