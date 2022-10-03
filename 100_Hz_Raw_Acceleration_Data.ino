// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ELE8066 MSc Project: The Development of a Wearable Device for Monitoring the Biometric Data of an Elite Competition Horse - James Butt 03/10/2022
// Interfacing the ESP32 with the Adafruit Ultimate GPS V3 Module and BNO055
// 9-Axis Orientation Sensor and Saving Data to the Micro-SD Card:
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - LIBRARIES - - - - - - - - - - - - - - - - - - - - -

// Importing the Adafruit OLED libraries:
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// The SSD1306 constructor accepts an ESP32 pin number to which reset pin of the display is connected.
// Because the current OLED doesn’t have a RESET pin, –1 is sent so none of the pins is used as a reset for the // display.
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Importing the TinyGPS and serial communications protocol libraries:
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Loading the BNO055 IMU sensor libraries:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Loading the Micro-SD libraries:
#include <SPI.h>
#include "FS.h"
#include "SdFat.h"

// Setting the SD card's maximum reading speed (MHz):
#define SD_SPI_Speed 25

// Creating the SD card object and file (this is required when using 'SdFat'):
SdFat SD_Card;

SdFile Data_File;

// Importing the 'string-to-double' library (for writing to the SD card):
#include <stdlib.h>

// - - - - - - - - - - - - - - - - TIMER & ISR INITIALISATION - - - - - - - - - - - - - - - - -

volatile int IMU_Interrupt_Counter, IMU_Readings;

// Declaring the pointer to the hw_timer_t variable type for the IMU:
hw_timer_t * Sensor_Timer = NULL;

// The 'PortMUX' variable is used for synchronisation.
// It ensures the ISRs and loop do not try to access the interrupt counters at the same time.
portMUX_TYPE Sensor_Timer_Mux = portMUX_INITIALIZER_UNLOCKED;

// The IMU interrupt-service-routine (ISR):
void IRAM_ATTR Sensor_Timer_ISR(){
  portENTER_CRITICAL_ISR(&Sensor_Timer_Mux);
  IMU_Interrupt_Counter  ++;
  portEXIT_CRITICAL_ISR(&Sensor_Timer_Mux);
 
}

volatile int Clock_Interrupt_Counter;

// Declaring the pointer to the hw_timer_t variable type for the clock:
hw_timer_t * Clock_Timer = NULL;

portMUX_TYPE Clock_Timer_Mux = portMUX_INITIALIZER_UNLOCKED;

// The ESP32 clock interrupt-service-routine (ISR):
void IRAM_ATTR Clock_Timer_ISR(){
  portENTER_CRITICAL_ISR(&Clock_Timer_Mux);
  Clock_Interrupt_Counter  ++;
  portEXIT_CRITICAL_ISR(&Clock_Timer_Mux);
 
}

// - - - - - - - - - - - - - - - VARIABLE & OBJECT DEFINITIONS - - - - - - - - - - - - - - - -

// Setting the GPS receiver / transmitter pins and communication speed:
static const int ESP32_RX_to_GPS_TX_Pin = 16, ESP32_TX_to_GPS_RX_Pin = 17;
static const int GPS_Baud_Rate = 9600;

// Creating the GPS object:
TinyGPSPlus GPS_Module;

int Satellite_Connections;

// Establishing serial connection with the GPS device:
SoftwareSerial GPS_Serial(ESP32_RX_to_GPS_TX_Pin, ESP32_TX_to_GPS_RX_Pin);

// Initialising a character array for saving GPS data to the SD card:
char GPS_Buffer[12];

// Creating the IMU sensor 'object':
Adafruit_BNO055 IMU_1 = Adafruit_BNO055();
Adafruit_BNO055 IMU_2 = Adafruit_BNO055();

// Creating an empty string to capture the IMU data to be logged:
String IMU_1_Data, IMU_2_Data;

// Setting the chip-select for the SD card SPI communication protocol:
#define SD_Chip_Select_Pin 2

// Formatting the red button as a digital input:
const int Button = 15;

// - - - - - - - - - - - - - - - - - FUNCTION DECLARATIONS - - - - - - - - - - - - - - - - - -

// Declaring functions / sub-routines:
void TCA9548A_Multiplexer(uint8_t Multiplexer_Bus_Select);

void Read_IMU_1_Data(Adafruit_BNO055 IMU_1, int Multiplexer_Bus_Select);
void Read_IMU_2_Data(Adafruit_BNO055 IMU_2, int Multiplexer_Bus_Select);

static void Smart_Delay(unsigned long Milliseconds);

static void GPS_Date_and_Time(TinyGPSDate &d, TinyGPSTime &t, int Satellites);
static void GPS_Float(float val, bool Valid, int Min_Length, int Decimals);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//                                         VOID SETUP
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void setup(){

  // Initialising communication with the serial monitor:
  Serial.begin(115200);

  // Setting the red button as an input (for executing the loop):
  pinMode(Button, LOW);
  pinMode(Button, INPUT);

// - - - - - - - - - - - - - - INITIALISING THE OLED, GPS & IMUs - - - - - - - - - - - - - - -

  // Initilialising communication between the ESP32 and the OLED at I2C address 0x3C:
   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0,8);


  // Turning on the the GPS module:
  GPS_Serial.begin(GPS_Baud_Rate);

  // Starting I2C communication with the TCA9548A multiplexer:
  Wire.begin();

  // The BNO055 uses an I2C technique called 'clock stretching' wherein the chip holds the SCL line
  // low whilst taking a reading. Initialising each IMU with a while loop insulates against this:

  // Initialising the first IMU on I2C bus number 1:
  TCA9548A_Multiplexer(1);
  while(!IMU_1.begin()){
    
    TCA9548A_Multiplexer(1);
    IMU_1.begin();
    Smart_Delay(10);
    
  }

  // Initialising the second IMU on I2C bus number 2:
  TCA9548A_Multiplexer(2);
  while(!IMU_2.begin()){
    
    TCA9548A_Multiplexer(2);
    IMU_2.begin();
    Smart_Delay(10);
    
  }

// - - - - - - - - - - - - - - - - INITIALISING THE SD CARD - - - - - - - - - - - - - - - - - -

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,8);

  display.print("Initialising the SD\ncard...");
  display.display();
  delay(1500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,8);
  
  if(!SD_Card.begin(SD_Chip_Select_Pin, SD_SCK_MHZ(SD_SPI_Speed))){

    Serial.println("Card initialisation failed, or the SD card is not present.");
    
    display.print("Card initialisation\nfailed, or the SD\ncard is not present.");
    display.display();
    
    while(1);
    
  }

  Serial.println("SD card initialised.");

  display.print("SD card initialised.\n");
  display.display();
  delay(1500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,8);

  // Requesting a button press before printing the 'ESP32 Sensor Data.csv' file headers and running the loop:
  Serial.println("GPS connection established.Press the red button to initiate the device...");
  
  display.print("GPS connection\nestablished.\nPress the red button to initiate the\ndevice...");
  display.display();

  while(LOW == digitalRead(Button));

// - - - - - - - - - - - -WRITING / APPENDING HEADERS TO THE .CSV FILE - - - - - - - - - - - -

  SD_Card.open("ESP32 Sensor Data.csv");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,8);
  
  Serial.println("Appending headers to the ESP32 Sensor Data file...");
    
  display.print("Appending headers to\nthe ESP32 Sensor\nData file.");
  display.display();

  Data_File.open("ESP32 Sensor Data.csv", O_WRITE | O_APPEND | O_CREAT);

  Data_File.println("IMU_1 LA Data,,,IMU_1 AV Data,,,IMU_2 LA Data,,,IMU_2 AV Data,,,Time,Latitude,Longitude,Altitude,Velocity,Satellites,Milliseconds,");
  Data_File.println("[d^2/dt^2][x_1]:,[d^2/dt^2][y_1]:,[d^2/dt^2][z_1]:,[d/dt][phi_1]:,[d/dt][theta_1]:,[d/dt][psi_1]:,[d^2/dt^2][x_2]:,[d^2/dt^2][y_2]:,[d^2/dt^2][z_2]:,[d/dt][phi_2]:,[d/dt][theta_2]:,[d/dt][psi_2]:,(HH:MM:SS):,(deg):,(deg):,(m):,(m/s):,Connected:,Elapsed:,");

  Data_File.close();
  
  delay(1500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,8);

  Serial.println("Device initialised. Measurement in progress.");

  display.print("Device initialised.\n\nMeasurement in\nprogress...");
  display.display();
  
// - - - - - - - - - - - - - - - - STARTING THE ESP32 TIMERS - - - - - - - - - - - - - - - - -

  // Initialising the IMU timer at 100 Hz (on ESP32 timer 0):
  Sensor_Timer = timerBegin(0, 80, true);
  timerAttachInterrupt(Sensor_Timer, &Sensor_Timer_ISR, true);
  timerAlarmWrite(Sensor_Timer, 10000, true);
  timerAlarmEnable(Sensor_Timer);

  // Setting IMU_Readings to 10 means the first iteration will include GPS data:
  IMU_Readings = 10;

  // Initialising the clock timer at 1000 Hz (on ESP32 timer 1):
  Clock_Timer = timerBegin(1, 80, true);
  timerAttachInterrupt(Clock_Timer, &Clock_Timer_ISR, true);
  timerAlarmWrite(Clock_Timer, 1000, true);
  timerAlarmEnable(Clock_Timer);

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//                                         VOID LOOP
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void loop(){

// - - - - - - - - - - - - - RECORDING & SAVING THE IMU DATA (100 Hz) - - - - - - - - - - - - -

  if(IMU_Interrupt_Counter > 0){

    Data_File.open("ESP32 Sensor Data.csv", O_APPEND | O_WRITE);

    // Reading data from the IMU on the left leg:
    Read_IMU_1_Data(IMU_1, 1);

    Data_File.print(IMU_1_Data.c_str());

    // Reading data from the IMU on the right leg:
    Read_IMU_2_Data(IMU_2, 2);

    Data_File.print(IMU_2_Data.c_str());

// - - - - - - - - - - - - - - RECORDING & SAVING GPS DATA (10 Hz) - - - - - - - - - - - - - - -

    if(IMU_Readings >= 9){

      // Pulling GPS data and storing to the SD card:
      GPS_Date_and_Time(GPS_Module.date, GPS_Module.time, GPS_Module.satellites.value());

      GPS_Float(GPS_Module.location.lat(), GPS_Module.location.isValid(), 3, 7);
      GPS_Float(GPS_Module.location.lng(), GPS_Module.location.isValid(), 3, 7);
      GPS_Float(GPS_Module.altitude.meters(), GPS_Module.location.isValid(), 3, 2);
      GPS_Float(GPS_Module.speed.mps(), GPS_Module.location.isValid(), 3, 2);
      
      GPS_Float(GPS_Module.satellites.value(), GPS_Module.satellites.isValid(), 1, 0);

      IMU_Readings = 0;
      
    }
    
    else{

      Data_File.print("NaN,NaN,NaN,NaN,NaN,NaN,");
    
      IMU_Readings ++;

    }

    // This external counter acts as a 'speed limit' for the loop execution:
    portENTER_CRITICAL(&Sensor_Timer_Mux);
    IMU_Interrupt_Counter --;
    portEXIT_CRITICAL(&Sensor_Timer_Mux);

    String Clock_Interrupt_Counter_String = String(Clock_Interrupt_Counter);

    Data_File.print(Clock_Interrupt_Counter_String.c_str());

    Data_File.print("\n");
     
    Data_File.close();
    
  }

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//                                         FUNCTIONS
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void TCA9548A_Multiplexer(uint8_t _Multiplexer_Bus_Select){

  // The multiplexer address is 0x70:
  Wire.beginTransmission(0x70);
  Wire.write(1 << _Multiplexer_Bus_Select);
  Wire.endTransmission();

}

void Read_IMU_1_Data(Adafruit_BNO055 IMU_1, int Multiplexer_Bus_Select){

  // Sometimes the Mux fails to select the correct bus on the first attempt, so it is called twice:
  TCA9548A_Multiplexer(Multiplexer_Bus_Select);
  TCA9548A_Multiplexer(Multiplexer_Bus_Select);
  
  imu::Vector<3> Linear_Acceleration_1 = IMU_1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> Angular_Velocity_1 = IMU_1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  IMU_1_Data = String(Linear_Acceleration_1.x()) + "," + String(Linear_Acceleration_1.y()) + "," + String(Linear_Acceleration_1.z())+ ","
             + String(Angular_Velocity_1.x()) + "," + String(Angular_Velocity_1.y()) + "," + String(Angular_Velocity_1.z()) + ",";

}

void Read_IMU_2_Data(Adafruit_BNO055 IMU_2, int Multiplexer_Bus_Select){

  // Sometimes the Mux fails to select the correct bus on the first attempt, so it is called twice:
  TCA9548A_Multiplexer(Multiplexer_Bus_Select);
  TCA9548A_Multiplexer(Multiplexer_Bus_Select);
  
  imu::Vector<3> Linear_Acceleration_2 = IMU_2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> Angular_Velocity_2 = IMU_2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  IMU_2_Data = String(Linear_Acceleration_2.x()) + "," + String(Linear_Acceleration_2.y()) + "," + String(Linear_Acceleration_2.z())+ ","
             + String(Angular_Velocity_2.x()) + "," + String(Angular_Velocity_2.y()) + "," + String(Angular_Velocity_2.z()) + ",";
             
}

// - - - - - - - - - - - - - - - - - - - - SMART DELAY - - - - - - - - - - - - - - - - - - - -

// A custom version of delay() to ensure the GPS object is being 'fed':
static void Smart_Delay(unsigned long Milliseconds){
  
  unsigned long Start = millis();
  
  do{
    
    while(GPS_Serial.available())
    
      GPS_Module.encode(GPS_Serial.read());
  }
  
  while(millis() - Start < Milliseconds);
  
}

// - - - - - - - - - - - - - - - - FORMATTING THE GPS DATE & TIME - - - - - - - - - - - - - - 

static void GPS_Date_and_Time(TinyGPSDate &d, TinyGPSTime &t, int Satellites){
  
  if(GPS_Module.satellites.value() <= 3){
    
    Data_File.print("NaN,");
    
  }
  
  else{
    
    sprintf(GPS_Buffer, "%02d:%02d:%02d,", t.hour(), t.minute(), t.second());

    Data_File.print(GPS_Buffer);
    
  }
  
  Smart_Delay(0);
  
}

// - - - - - - - - - - - FORMATTING GPS FLOATS AND SAVING TO THE SD CARD - - - - - - - - - - -

static void GPS_Float(float val, bool Valid, int Min_Length, int Decimals){

  if(Valid){
    
    dtostrf(val, Min_Length, Decimals, GPS_Buffer);

    Data_File.print(GPS_Buffer);
    Data_File.print(",");
    
  }
  
  else{

    Data_File.print("NaN,");
    
  }
  
  Smart_Delay(0);

}
