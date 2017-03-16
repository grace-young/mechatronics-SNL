#include <Wire.h>

#define COMMS_OUT_TO_BRAIN 9

//Gyro Stuff
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
//Gyro stuff ends

//commands to Brain 
#define   GYRO_NEGATIVE             20
#define   GYRO_POSITIVE             40

//Gyro variables
long orientation = 0;
int prevZ;
bool flag;
long int cpt=0; //count
bool statedPositive;
//Gyro variables end

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Gyro Configuration
  // Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  

}

void loop() {
   communicateGyroInfo();
  ///////////GYRO

//  Serial.print (cpt++,DEC); //Used to debug gyro
//  Serial.print ("\t");
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Create 16 bits values from 8 bits data
  
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

   // Accelerometer
//  int16_t ax=-(Buf[0]<<8 | Buf[1]);
//  int16_t ay=-(Buf[2]<<8 | Buf[3]);
//  int16_t az=Buf[4]<<8 | Buf[5];

//  Serial.print("ax ");
//  Serial.print(ax, DEC);
//
//  Serial.print("ay ");
//  Serial.print(ay,DEC);
//
//  Serial.print("az ");
//  Serial.print(az,DEC);
//  Serial.print(" ");
//  Serial.print ("x ");
//  Serial.print (gx,DEC); 
//  Serial.print ("\t");
//  Serial.print ("y ");
//  Serial.print (gy,DEC);
//  Serial.print ("\t");
//  Serial.print("z ");
//  Serial.println(gz,DEC);  
//  Serial.print ("\t");
//  Serial.print (prevZ);
//  Serial.print("   ");
  // 100 was nice 
 //Serial.print("orientation");
  //Serial.println(orientation);


  
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
//  Serial.println("");

////////////GYRO END
}

void communicateGyroInfo(){
  if(orientation <= 0){
    analogWrite(COMMS_OUT_TO_BRAIN,GYRO_NEGATIVE);
    //Serial.println("negative");
  } else if(orientation > 0){
    analogWrite(COMMS_OUT_TO_BRAIN,GYRO_POSITIVE);
    //Serial.println("positive");
  }
}

//Gyro Functions
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
