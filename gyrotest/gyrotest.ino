#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

long orientation = 0;
int prevZ;
bool flag;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

}

int trust = 20;
long int cpt=0;

double RxAccPrev;
double RyAccPrev;
double RzAccPrev;
  
double RxAcc;
double RyAcc;
double RzAcc;

static double AxzPrev;
static double AyzPrev;
static double AxyPrev;

double RateAxz;
double RateAyz;
double RateAxy;
  
double Axz;
double Ayz;
double Axy;

double RxGyro;
double RyGyro;
double RzGyro;
  
double RxEst;`
double RzEst;

double RxExt2;
double RyExt2;
double RzExt2;

double R;

double RzEst3;

unsigned long dt;
// this is part of milis
unsigned long prevtime = 0;

double angle;

// Main loop, read and display data
void loop() {
  
  // _______________
  // ::: Counter :::
  
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
  
 
 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  
  // Create 16 bits values from 8 bits data
  
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

   // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];
  
  // Display Gyroscope Values

  Serial.print(" ax ");
  Serial.print(ax, DEC);

  Serial.print(" ay ");
  Serial.print(ay,DEC);

  Serial.print(" az ");
  Serial.print(az,DEC);
  Serial.print(" ");
  Serial.print ("x ");
  Serial.print (gx,DEC); 
  Serial.print ("\t");
  Serial.print ("y ");
  Serial.print (gy,DEC);
  Serial.print ("\t");
  Serial.print ("z ");
  Serial.print (gz,DEC);  
  Serial.print ("\t");

  dt = millis()-prevtime;
  prevtime = millis();
  RxAccPrev = RxAcc;
  RyAccPrev = RyAcc;
  RzAccPrev = RzAcc;
  
  RxAcc = (ax/16384.0);
  RyAcc = (ay/16384.0);
  RzAcc = (az/16384.0);

  Serial.print(" RxAccPrev ");
  Serial.print(RxAccPrev);

  Serial.print(" RzAccPrev ");
  Serial.print(RzAccPrev);
//  Serial.print(" RyAcc ");
//  Serial.print(RyAcc);
  

  
  AxzPrev = atan2(RxAccPrev, RzAccPrev);
  AyzPrev = atan2(RyAccPrev, RzAccPrev);
  AxyPrev = atan2(RzAccPrev, RyAccPrev);

  Serial.print(" AxzPrev ");
  Serial.print(AxzPrev);

  RateAxz = ((double)gy)/16.4;
  RateAyz = ((double)gx)/16.4;
  RateAxy = ((double)gz)/16.4;

  // divide by 1000 to convert to seconds
  Axz = AxzPrev + RateAxz * ((double)dt)/1000.0;
  Ayz = AyzPrev + RateAyz * ((double)dt)/1000.0;
  Axy = AxyPrev + RateAxy * ((double)dt)/1000.0;

  Serial.print(" rate ");
  Serial.print(RateAxz * dt/1000.0);

  if(gz>15){
    angle = angle + RateAxz * dt/1000.0;
  }

  Serial.print(" angle: ");
  Serial.print(angle);
  
  RxGyro = sin(Axz / sqrt(1+pow(cos(Axz),2)*pow(tan(Ayz),2)));
  RyGyro = sin(Ayz / sqrt(1+pow(cos(Ayz),2)*pow(tan(Axz),2)));
  RzGyro = sqrt(1-pow(RxGyro,2)-pow(RyGyro,2));
  
  RxEst = (RxAcc + RxGyro*trust) / (1+trust);
  RyEst = (RyAcc + RyGyro*trust) / (1+trust);
  RzEst = (RzAcc + RzGyro*trust) / (1+trust);

  RxExt2 = (RxEst + RxGyro*trust) / (1+trust);
  RyExt2 = (RyEst + RyGyro*trust) / (1+trust);
  RzExt2 = (RzEst + RzGyro*trust) / (1+trust);

  R = sqrt(pow(RxExt2,2) + pow(RyExt2,2) + pow(RzExt2,2));

  RzEst3 = RzExt2/R;
  
  flag = false;
  if ( (abs(gx) > 600 || abs(gy) > 600)) {
    flag = true;
  }
  if((gz<0 && prevZ >0) && (gz>0 && prevZ <0)&&abs(gz) > 15){
    orientation -= prevZ/10;
  }
  else if ( (abs(gx) > 600 || abs(gy) > 600) && abs(gz) > 100) {
    orientation += gz/10;
  }
  else if (abs(gz) > 15 && !flag) {
    orientation += gz/10;
  }
  prevZ = gz;


  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;

  
  


  // Create 16 bits values from 8 bits data
  
  // End of line
  Serial.println("");
//  delay(100);    
}
