/*
An Arduino code example for interfacing with the HMC5883

by: Jordan McConnell
 SparkFun Electronics
 created on: 6/30/11
 license: OSHW 1.0, http://freedomdefined.org/OSHW

Analog input 4 I2C SDA
Analog input 5 I2C SCL
*/

#include <Wire.h> //I2C Arduino Library

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

int accel_x, accel_y, accel_z; //accelerometer triple axis data
int mag_x,mag_y,mag_z; //magnometer triple axis data


void setup(){
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void loop(){
  

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    mag_x = Wire.read()<<8; //X msb
    mag_x |= Wire.read(); //X lsb
    mag_z = Wire.read()<<8; //Z msb
    mag_z |= Wire.read(); //Z lsb
    mag_y = Wire.read()<<8; //Y msb
    mag_y |= Wire.read(); //Y lsb
  }
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(mag_x);
  Serial.print("  y: ");
  Serial.print(mag_y);
  Serial.print("  z: ");
  Serial.println(mag_z);
  
  accel_x = analogRead(0);       // read analog input pin 0
  accel_y = analogRead(1);       // read analog input pin 1
  accel_z = analogRead(2);       // read analog input pin 1
  Serial.print("accelerations are x, y, z: ");
  Serial.print(accel_x, DEC);    // print the acceleration in the X axis
  Serial.print(" ");       // prints a space between the numbers
  Serial.print(accel_y, DEC);    // print the acceleration in the Y axis
  Serial.print(" ");       // prints a space between the numbers
  Serial.println(accel_z, DEC);  // print the acceleration in the Z axis

  
  float headingTC = CalculateHeadingTiltCompensated();
  Serial.print("Heading (Tilt Compensated):");
  Serial.println(RadiansToDegrees(headingTC));

  delay(250);
}

float RadiansToDegrees(float rads)
{
  // Correct for when signs are reversed.
  if(rads < 0)
    rads += 2*PI;
      
  // Check for wrap due to addition of declination.
  if(rads > 2*PI)
    rads -= 2*PI;
   
  // Convert radians to degrees for readability.
  float heading = rads * 180/PI;
       
  return heading;
}

float CalculateHeadingTiltCompensated()
{
  // We are swapping the accelerometers axis as they are opposite to the compass the way we have them mounted.
  // We are swapping the signs axis as they are opposite.
  // Configure this for your setup.
  float accX = -accel_y;
  float accY = -accel_x;
  
  float rollRadians = asin(accY);
  float pitchRadians = asin(accX);
  
  // We cannot correct for tilt over 40 degrees with this algorthem, if the board is tilted as such, return 0.
  if(rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
  {
    return 0;
  }
  
  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(rollRadians);
  float sinRoll = sin(rollRadians);  
  float cosPitch = cos(pitchRadians);
  float sinPitch = sin(pitchRadians);
  
  float Xh = mag_x * cosPitch + mag_z * sinPitch;
  float Yh = mag_x * sinRoll * sinPitch + mag_y * cosRoll - mag_z * sinRoll * cosPitch;
  
  float heading = atan2(Yh, Xh);
    
  return heading;
}


