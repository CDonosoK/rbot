#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>


static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
float lat, lon, pitch, roll, yaw;
int pinLed = 5;

// GPS variables
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// Accel variables
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  accel.begin();
  mag.begin();
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);
}

void loop()
{
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      obtainGPS();
    }
  }
  obtainGyro();
  printInformation();
    
}

void obtainGyro(){
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  // leer el estado de los sensores
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  // usar el algoritmo de fusion de la libreria para combinar las mediciones
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    pitch = orientation.pitch;
    roll = orientation.roll;
    yaw = orientation.heading;
  }
}

void obtainGPS(){
  if (gps.location.isValid()){
    lat = gps.location.lat();
    lon = gps.location.lng();
  }
}

void printInformation(){
  Serial.print("Lat: ");
  Serial.print(lat);
  Serial.print(" - Lon: ");
  Serial.print(lon);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" - Yaw: ");
  Serial.print(yaw);
  Serial.print(" - Roll: ");
  Serial.println(roll);
}