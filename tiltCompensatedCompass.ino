/*
Analog input 4 I2C SDA
Analog input 5 I2C SCL
*/

#include <Wire.h> //I2C Arduino Library
#include <math.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

//accelerometer triple axis raw data
int accel_x = 0;
int accel_y = 0;
int accel_z = 0; 

//magnometer triple axis raw data
int mag_x = 0;
int mag_y = 0;
int mag_z = 0; 

int accel_x_axis = 5; //X = Pin 5
int accel_y_axis = 6; //Y = Pin 6
int accel_z_axis = 7; //Z = Pin 7

//by increasing alphaAccel the response will become faster
//but the noise will increae [alpha must be between 0 and 1]
//values for digital lowpass
float alphaAccel = 0.4;
float alphaMagnet = 0.4;

float Pitch=0;
float Roll=0;
float Yaw=0;

int xRaw=0;
int yRaw=0;
int zRaw=0;

unsigned int xOffset=0;
unsigned int yOffset=0;
unsigned int zOffset=0;

float xFiltered=0;
float yFiltered=0;
float zFiltered=0;

float xFilteredOld=0;
float yFilteredOld=0;
float zFilteredOld=0;

float xMagnetFiltered=0;
float yMagnetFiltered=0;
float zMagnetFiltered=0;

float xMagnetFilteredOld=0;
float yMagnetFilteredOld=0;
float zMagnetFilteredOld=0;

int xMagnetMax=0;
int yMagnetMax=0;
int zMagnetMax=0;

int xMagnetMin=10000;
int yMagnetMin=10000;
int zMagnetMin=10000;

float xMagnetMap=0;
float yMagnetMap=0;
float zMagnetMap=0;

float YawU;
float Azimuth;
int count = 0;

void setup(){
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  getAccelOffset();           //keep it flat and non moving on the table

  //there are other ways to calibrate the offset, each has some advantes
  //and disadvantes..
  //compare application note AN3447
  //http://www.freescale.com/files/sensors/doc/app_note/AN3447.pdf
}

void loop()
{
  
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

  accel_x = analogRead(accel_x_axis); // read analog input pin 5
  accel_y = analogRead(accel_y_axis); // read analog input pin 6
  accel_z = analogRead(accel_z_axis); // read analog input pin 7

  FilterAD();
  AD2Degree();
  getAzimuth();
  Send2Com();
  
}

void Send2Com()
{
  Serial.print("Pitch: ");
  Serial.print(int(Pitch*180/PI));
  Serial.print(" Roll: ");
  Serial.print(int(Roll*180/PI));

  Serial.print(" Yaw: ");
  Serial.print(int(Yaw));

  Serial.print(" Azimuth: ");
  Serial.println(int(Azimuth));
  delay(50);
}

void getAzimuth()
{
  //this part is required to normalize the magnetic vector
  if (xMagnetFiltered>xMagnetMax) {
    xMagnetMax = xMagnetFiltered;
  }
  if (yMagnetFiltered>yMagnetMax) {
    yMagnetMax = yMagnetFiltered;
  }
  if (zMagnetFiltered>zMagnetMax) {
    zMagnetMax = zMagnetFiltered;
  }

  if (xMagnetFiltered<xMagnetMin) {
    xMagnetMin = xMagnetFiltered;
  }
  if (yMagnetFiltered<yMagnetMin) {
    yMagnetMin = yMagnetFiltered;
  }
  if (zMagnetFiltered<zMagnetMin) {
    zMagnetMin = zMagnetFiltered;
  }

  float norm;

  xMagnetMap = float(map(xMagnetFiltered, xMagnetMin, xMagnetMax, -10000, 10000))/10000.0;
  yMagnetMap = float(map(yMagnetFiltered, yMagnetMin, yMagnetMax, -10000, 10000))/10000.0;
  zMagnetMap = float(map(zMagnetFiltered, zMagnetMin, zMagnetMax, -10000, 10000))/10000.0;


  //normalize the magnetic vector
  norm= sqrt( sq(xMagnetMap) + sq(yMagnetMap) + sq(zMagnetMap));
  xMagnetMap /=norm;
  yMagnetMap /=norm;
  zMagnetMap /=norm;

  //compare Applications of Magnetic Sensors for Low Cost Compass Systems by Michael J. Caruso
  //for the compensated Yaw equations...
  //http://www.ssec.honeywell.com/magnetic/datasheets/lowcost.pdf
  Yaw=atan2( (-yMagnetMap*cos(Roll) + zMagnetMap*sin(Roll) ) , xMagnetMap*cos(Pitch) + zMagnetMap*sin(Pitch)*sin(Roll)+ zMagnetMap*sin(Pitch)*cos(Roll)) *180/PI;
  YawU=atan2(-yMagnetMap, xMagnetMap) *180/PI;
  
  //correct for 360 degrees 
  if(YawU <= -1 && YawU >= -180){
    Azimuth = map(YawU, -180, -1, 1, 180) + 180;
  }else{
    Azimuth = YawU;
  }
}


void AD2Degree()
{
  // 3.3 = Vref; 1023 = 10Bit AD; 0.8 = Sensivity Accelerometer
  // (compare datasheet of your accelerometer)
  // the *Accel must be between -1 and 1; you may have to
  // to add/subtract +1 depending on the orientation of the accelerometer
  // (like me on the zAccel)
  // they are not necessary, but are useful for debugging
  //xAccel = xFiltered *3.3 / (1023.0*0.8);      
  //yAccel = yFiltered *3.3 / (1023.0*0.8);      
  //zAccel = zFiltered *3.3 / (1023.0*0.8)+1.0;

  // Calculate Pitch and Roll (compare Application Note AN3461 from Freescale
  // http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
  // Microsoft Excel switches the values for atan2
  // -> this info can make your life easier :-D
  //angled are radian, for degree (* 180/3.14159)
  Roll   = atan2(accel_y,sqrt(sq(accel_x)+sq(accel_z)));  
  Pitch  = atan2(accel_x,sqrt(sq(accel_y)+sq(accel_z)));

}

void FilterAD()
{
  // read from AD and subtract the offset
  xRaw=accel_x-xOffset;
  yRaw=accel_y-yOffset;
  zRaw=accel_z-zOffset;

  //Digital Low Pass - compare: (for accelerometer)
  //http://en.wikipedia.org/wiki/Low-pass_filter
  xFiltered= xFilteredOld + alphaAccel * (xRaw - xFilteredOld);
  yFiltered= yFilteredOld + alphaAccel * (yRaw - yFilteredOld);
  zFiltered= zFilteredOld + alphaAccel * (zRaw - zFilteredOld);

  xFilteredOld = xFiltered;
  yFilteredOld = yFiltered;
  zFilteredOld = zFiltered;


  xMagnetFiltered= xMagnetFilteredOld + alphaMagnet * (mag_x - xMagnetFilteredOld);
  yMagnetFiltered= yMagnetFilteredOld + alphaMagnet * (mag_y - yMagnetFilteredOld);
  zMagnetFiltered= zMagnetFilteredOld + alphaMagnet * (mag_z - zMagnetFilteredOld);

  xMagnetFilteredOld = xMagnetFiltered;
  yMagnetFilteredOld = yMagnetFiltered;
  zMagnetFilteredOld = zMagnetFiltered;

}

void getAccelOffset()
{ //you can make approx 60 iterations because we use an unsigned int
  //otherwise you get an overflow. But 60 iterations should be fine
  for (int i=1; i <= 60; i++){        
    xOffset += analogRead(accel_x_axis);    
    yOffset += analogRead(accel_y_axis);
    zOffset += analogRead(accel_z_axis);
  }
  xOffset /=60;  
  yOffset /=60;
  zOffset /=60;

  Serial.print("xOffset: ");
  Serial.print(xOffset);
  Serial.print("   yOffset: ");
  Serial.print(yOffset);
  Serial.print("   zOffset: ");
  Serial.println(zOffset);
}


