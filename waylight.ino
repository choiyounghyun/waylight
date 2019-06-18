#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include<Wire.h>
#include<math.h>

#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68


Servo servo;

char valorbyte[8];
int graus=0,contador=0;
byte valor=0;
char buffer[5];               //통신을 할때 buffer배열에 전송받은 데이터 입력
int pos;
char bufferIndex = 0; 
int a,b;
int def=90;
int laser = 13;
SoftwareSerial mySerial(2,3);  //tx rx
SoftwareSerial BTSerial(4,5); //tx rx
LiquidCrystal_I2C lcd(0x27,16,2);

//칼만
struct GyroKalman{
  float x_angle, x_bias;
  float P_00, P_01, P_10, P_11;
  float Q_angle, Q_gyro;
  float R_angle;
};

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

static const float R_angle = 0.3;     //.3 default
static const float Q_angle = 0.01;  //0.01 (Kalman)
static const float Q_gyro = 0.04; //0.04 (Kalman)

const int lowX = -2150;
const int highX = 2210;
const int lowY = -2150;
const int highY = 2210;
const int lowZ = -2150;
const int highZ = 2550;

unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

typedef union accel_t_gyro_union
{
  struct
  {
  uint8_t x_accel_h;
  uint8_t x_accel_l;
  uint8_t y_accel_h;
  uint8_t y_accel_l;
  uint8_t z_accel_h;
  uint8_t z_accel_l;
  uint8_t t_h;
  uint8_t t_l;
  uint8_t x_gyro_h;
  uint8_t x_gyro_l;
  uint8_t y_gyro_h;
  uint8_t y_gyro_l;
  uint8_t z_gyro_h;
  uint8_t z_gyro_l;
  } reg;

  struct
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

int xInit[5] = {0,0,0,0,0};
int yInit[5] = {0,0,0,0,0};
int zInit[5] = {0,0,0,0,0};
int initIndex = 0;
int initSize = 5;
int xCal = 0;
int yCal = 0;
int zCal = 1800;


void setup(){
  int error;
  uint8_t c;

  initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);
  Serial.begin(9600);
  Wire.begin();
   // BTSerial.begin(9600);
  servo.attach(9);
  servo.write(def);
  pinMode(laser, OUTPUT);
//  digitalWrite(laser, HIGH);
  lcd.init();
  lcd.backlight();
  lcd.print("Ready");
    MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

 }

void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  curSensoredTime = millis();

//  Serial.println(F(""));
//  Serial.println(F("MPU-6050"));
  
  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if(error != 0) {
    Serial.print(F("Read accel, temp and gyro, error = "));
    Serial.println(error,DEC);
  }
  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
  
  // Print the raw acceleration values
//  Serial.print(F("accel x,y,z: "));
//  Serial.print(accel_t_gyro.value.x_accel, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.y_accel, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.z_accel, DEC);
//  Serial.println(F(""));
  
  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet:
  // 340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412
//  Serial.print(F("temperature: "));
//  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
//  Serial.print(dT, 3);
//  Serial.print(F(" degrees Celsius"));
//  Serial.println(F(""));
  
  // Print the raw gyro values.
//  Serial.print(F("gyro x,y,z : "));
//  Serial.print(accel_t_gyro.value.x_gyro, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.y_gyro, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.z_gyro, DEC);
//  Serial.println(F(""));


  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  if(prevSensoredTime > 0) {
    int gx1=0, gy1=0, gz1 = 0;
    float gx2=0, gy2=0, gz2 = 0;

    int loopTime = curSensoredTime - prevSensoredTime;

    gx2 = angleInDegrees(lowX, highX, accel_t_gyro.value.x_gyro);
    gy2 = angleInDegrees(lowY, highY, accel_t_gyro.value.y_gyro);
    gz2 = angleInDegrees(lowZ, highZ, accel_t_gyro.value.z_gyro);

    predict(&angX, gx2, loopTime);
    predict(&angY, gy2, loopTime);
    predict(&angZ, gz2, loopTime);

    gx1 = update(&angX, accel_t_gyro.value.x_accel) ;
    gy1 = update(&angY, accel_t_gyro.value.y_accel) ;
    gz1 = update(&angZ, accel_t_gyro.value.z_accel) ;

    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    if(initIndex < initSize) {
      xInit[initIndex] = gx1;
      yInit[initIndex] = gy1;
      zInit[initIndex] = gz1;
      if(initIndex == initSize - 1) {
        int sumX = 0; int sumY = 0; int sumZ = 0;
        for(int k=1; k <= initSize; k++) {
          sumX += xInit[k];
          sumY += yInit[k];
          sumZ += zInit[k];
        }

        xCal -= sumX/(initSize -1);
        yCal -= sumY/(initSize -1);
        zCal = (sumZ/(initSize -1) - zCal);
      }
      initIndex++;
    }
    
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    else {
        gx1 += xCal;
        gy1 += yCal;
        //gz1 += zCal;
  
      // if(gz1 < 1400 && -250 < gy1 && gy1 < 250 && gx1 < 500) {
      //  Serial.print(F("Turn right"));
      //  Serial.println(F(""));
      //}

    }
 
//  mySerial.begin(9600);
 //   leitura();
 //   mySerial.end();
  //  delay(2000);
 // servo.attach(9);
   while(BTSerial.available()) {
    buffer[bufferIndex]  = BTSerial.read();   //블루투스 통신으로 버퍼배열에 데이터 수신
    bufferIndex++;   //데이터 수신 후 버퍼 인덱스 1 증가
      digitalWrite(laser, HIGH);

  }  
   if(bufferIndex!=0) {
      int pos = atoi(buffer);        
      Serial.println("==========================================================================");
      Serial.println("Input Data Android -> Arduino    |    azimuth    |    roll   |   pitch    ");
      Serial.println("---------------------------------+---------------+-----------+------------");
   //   Serial.println(pos);
   //   Serial.print("                                ");
    //  Serial.println(pos);
        a=pos;
   }
  int x;
if(a>360 && a<=9999)
{
  x=a/100;
  a=a-x*100;
}
if(a>=10000)
a=a/100;
    for(int a=0;a<5;a++) {
      buffer[a] = NULL;
    }
    bufferIndex = 0;
  BTSerial.end();
 
  mySerial.begin(9600);
  leitura();
  b=graus;
   mySerial.end();
  BTSerial.begin(9600);
ServoControl(a, b);

Serial.print(a);
Serial.print("                                ");
Serial.print(b);
Serial.print("             ");

float roll, pitch;
    roll=map(gx1,-16384,16384,-90,90);
    pitch=map(gy1,-16384,16384,-90,90);   
    Serial.print(pitch, 2);
    Serial.print("        ");
    Serial.println(roll, 2);
  
lcd.home();
lcd.print("AndData : ");

lcd.print(a);
lcd.setCursor(0,1);
lcd.print("heading : ");
lcd.print(b);



/*
if(b>a)
{
  if(b-a>=180) servo.write(0);
  else if(b-a>=90 && b-a<180)  servo.write(180);
  else
  servo.write(def+(b-a));
}

if(a>b)
{
  if(a-b>=180) servo.write(180);
  else if(b-a>=90 && b-a<180) servo.write(0);
  else
  servo.write(def-(a-b));
}

if(b>=0 && b <90)
{
   if(a>b && a<90)
      servo.write(def-(a-b));
   else if(b>a && a<90)
      servo.write(def+(a-b));
   else if(a>b && a>=90) 
   {
    if(a-b>=90 && a-b<180)
      servo.write(0);
    else if(a-b>=180)
      servo.write(180);
    else
      servo.write(def-(a-b));

   }
}


else if(b>=90 && b <180)
{
   if(a>b && a<180)
      servo.write(def-(a-b));
   else if(b>a && a<180)
      servo.write(def+(a-b));
   else if(a>b && a>=180) 
   {
    if(a-b>=90)
      servo.write(0);
    else if(a-b>=180)
      servo.write(180);
    else
      servo.write(def-(a-b));

   }
}

else if(b>=180 && b <270)
{
   if(a>b && a<270)
      servo.write(def-(a-b));
   else if(b>a && a<270)
      servo.write(def+(a-b));
   else if(a>b && a>=270) 
   {
    if(a-b>=90)
      servo.write(0);
    else if(a-b>=180)
      servo.write(180);
    else
      servo.write(def-(a-b));

    
   }
}

else if(b>=270 && b <360)
{
   if(a>b && a<360)
      servo.write(def-(a-b));
   else if(b>a && a<360)
      servo.write(def+(a-b));
   else if(b<a && a>=0) 
   {
    if(a+abs(360-b)>=90)
      servo.write(0);
    else if(a+abs(360-b)>=180)
      servo.write(180);
    else
      servo.write(def-(a-b));

   }
}
*/
/*
988989898989
if(b>=90 && b <180)
{
   if(a>b)
      servo.write(def-(b-a));
   else
      servo.write(def+(b+a));
}


if(b>=180 && b <270)
{
   if(a>b)
      servo.write(def-(b-a));
   else
      servo.write(def+(b+a));
}


if(b>=270 && b <360)
{
   if(a>b)
      servo.write(def-(b-a));
   else
      servo.write(def+(b+a));
}
*/
//SVC(a,b);
     //digitalWrite(laser, LOW);

   delay(1000);
  // servo.detach();
   lcd.clear();
  }
     prevSensoredTime = curSensoredTime;

}




void leitura(){
  valor=0;
  mySerial.write(0x31);
  while(valor==0){
    if(mySerial.available()){
      valorbyte[contador]=mySerial.read();
      contador=(contador+1)%8;
      if(contador==0){
        graus=(valorbyte[2]-48)*100+(valorbyte[3]-48)*10+(valorbyte[4]-48);  
        valor=1;
      }
    }
  }
//  Serial.println(graus);
  delay(300);
}

//int SVC(int b, int c){

/*if( b> c){
  if(b-c > 360-(b-c)){
    servo.write(360-(b-c));
    Serial.println(360-(b-c));
  }
  else if(b-c < 360-(b-c)){
    servo.write(0);
    Serial.println("0");
    }
  }
  else if(b<c){
  if((c-b)<360-(c-b)){
    servo.write(c-b);
    Serial.println(c-b);
  }
  else if((c-b)>360-(c-b)){
      servo.write(0);
      Serial.println("0");
  }
  }
  if(b>c){
    if(b-c<360-(b-c)){
      if(b-c<=90) {servo.write(90-(b-c));
      
      }
      else if((b-c) > 90) {servo.write(0);
      
      }
    }
    else if(b-c>360-(b-c)){
      if(360-(b-c)<=90)servo.write(90+(360-(b-c)));
      else if(360-(b-c)>90)servo.write(180); 
    }
  }
  else if(c>b){
    if(c-b<360-(c-b)){
      if((c-b)<=90) {servo.write(90+(c-b));
     
      
      }
      else if((c-b)>90) servo.write(180);
      }
    else if(c-b>360-(c-b)){
      if(360-(c-b)<=90){ servo.write(90-(360-(c-b)));
   
      }
      else if(360-(c-b) >90) {servo.write(0);
      
      }
      }
  }
  else{
    Serial.println("함수 방위각 , 지자기 방위각 같음");
    servo.write(90);
    }
  
}*/

int ServoControl(int a, int b){
    if(b>a)
    {
      if(b-a>=180) servo.write(0);
      else if(b-a>=90 && b-a<180)  servo.write(180);
      else
      servo.write(def+(b-a));
    }
    
    if(a>b)
    {
      if(a-b>=180) servo.write(180);
      else if(b-a>=90 && b-a<180) servo.write(0);
      else
      servo.write(def-(a-b));
    }
}

//********************************************
//********************************************
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  
  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);
  
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0); // return : no error
}


int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start); // write the start address
  if (n != 1)
    return (-20);
    
  n = Wire.write(pData, size); // write data bytes
  if (n != size)
    return (-21);
    
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
  return (0); // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}

/**************************************************
 * Raw data processing
 **************************************************/
float angleInDegrees(int lo, int hi, int measured) {
  float x = (hi - lo)/180.0;
  return (float)measured/x;
}

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) {
  kalman->Q_angle = Q_angle;
  kalman->Q_gyro = Q_gyro;
  kalman->R_angle = R_angle;
  
  kalman->P_00 = 0;
  kalman->P_01 = 0;
  kalman->P_10 = 0;
  kalman->P_11 = 0;
}

/*
* The kalman predict method.
* kalman    the kalman data structure
* dotAngle    Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
*           This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
*/
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
  kalman->x_angle += dt * (dotAngle - kalman->x_bias);
  kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_angle;
  kalman->P_01 += -1 * dt * kalman->P_11;
  kalman->P_10 += -1 * dt * kalman->P_11;
  kalman->P_11 += kalman->Q_gyro;
}

/*
* The kalman update method
* kalman  the kalman data structure
* angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
*/
float update(struct GyroKalman *kalman, float angle_m) {
  const float y = angle_m - kalman->x_angle;
  const float S = kalman->P_00 + kalman->R_angle;
  const float K_0 = kalman->P_00 / S;
  const float K_1 = kalman->P_10 / S;
  kalman->x_angle += K_0 * y;
  kalman->x_bias += K_1 * y;
  kalman->P_00 -= K_0 * kalman->P_00;
  kalman->P_01 -= K_0 * kalman->P_01;
  kalman->P_10 -= K_1 * kalman->P_00;
  kalman->P_11 -= K_1 * kalman->P_01;
  return kalman->x_angle;
}
