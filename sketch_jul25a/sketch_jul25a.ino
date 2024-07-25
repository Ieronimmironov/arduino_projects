#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#define CH1 3
#define CH2 9
MPU6050 mpu;
//volatile bool mpuFlag = false;
uint8_t fifoBuffer[45];


int ch1Value;
int ch2Value;

Servo arm1, arm2, arm3, arm4;



int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 

void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(1000000UL);
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  



 // attachInterrupt(0, dmpReady, RISING);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  arm1.attach(5);
  arm2.attach(6);
  arm3.attach(7);
  arm4.attach(8);

}

float deg[3], pos[4];
int16_t ai[3], gi[3];
float a[3], g[3];
float sign1, sign2, dd = 0.0;
int plot = 0;
bool first = true;
float ypr[3];


void loop() {
  
  ch1Value = readChannel(CH1, 0, 100, 0);
  ch2Value = readChannel(CH2, 0, 100, 0);

  static uint32_t tmr;
  if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;
      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // выводим результат в радианах (-3.14, 3.14)
      Serial.print(ypr[0]); // вокруг оси Z
      Serial.print(',');
      Serial.print(ypr[1]); // вокруг оси Y
      Serial.print(',');
      Serial.print(ypr[2]); // вокруг оси X
      Serial.println();
      // для градусов можно использовать degrees()
      tmr = millis();  // сброс таймера
    }
  }
  //my code
  sign_writer();
  Serial.print("Ch1: ");
  //Serial.print(ch1Value);
  Serial.println(sign1);//
  Serial.print(" | Ch2: ");
  Serial.println(sign2);//
  //Serial.println(ch2Value);
  //Serial.println(sign1);
  //Serial.println(sign2);

  deg[0] = ypr[0] * 180.0 / M_PI;//z
  deg[1] = ypr[1] * 180.0 / M_PI;//y
  deg[2] = ypr[2] * 180.0 / M_PI;//x
  mpu.getMotion6(&ai[0], &ai[1], &ai[2], &gi[0], &gi[1], &gi[2]);
  if(first)
  {
    dd = deg[2];
    first = false;
  }
  if(sign1 >= 65.0)
  {
    for(int i = 0; i < 3; i++)
    {
      a[i] = ai[i] / 8192.0 * 9.8;
      // Serial.print(a[i]);
      // Serial.print(", ");
    }
    for(int i = 0; i < 3; i++)
    {
      //g[i] = gi[i] / 32768.0 * M_PI;
      g[i] = gi[i] / 32768.0 * 180;
      // Serial.print(g[i]);
      // Serial.print(", ");
    }
    Serial.println(" ");
    if(g[2] >= 0.5)//выравнивание направления1*
    {
      if(deg[2] > dd)
      {
        if(pos[0] >= 30.0)
        {
          pos[0] /= 2.0;
        }
        else if(pos[1] < 30.0)
        {
          pos[0] = 0.0;
          pos[1] = 30.0;
        }
        else {
          pos[0] = 0.0;
          pos[1] *= 1.5;
        }
      }
    }
    else if(g[2] <= -0.5)
    {
      if(deg[2] < dd)
      {
        if(pos[1] >= 30.0)
        {
          pos[1] /= 2.0;
        }
        else if(pos[0] < 30.0)
        {
          pos[1] = 0.0;
          pos[0] = 30.0;
        }
        else {
          pos[1] = 0.0;
          pos[0] *= 1.5;
        }
      }
      

    }//1*
    //выравнивание горизонтали2*
      
    //2*
    
    if(a[1] < -0.5)//-0.03
    {
      //posmn(0.5);
    }
    posmn(sign2 / 100.0);
    pos_corrector();
    pos_writer();
    Serial.println(deg[2]);
    Serial.println(dd);
  }
  else if(sign1 >= 25.0)
  {
    first = true;
    pos[2] = sign2 * 1.8;
    pos[3] = 180 - pos[2];
    pos[0] = 0.0;
    pos[1] = 0.0;
    pos_writer();
  }
  else
  {
    first = true;
    for(int i = 0; i < 2; i++)
    {
      pos[i] = 0.0;
      pos[i + 2] = 90.0;
    }
    pos_writer();
  }
  delay(10);
  //.
}
void posmn(float mn)
{
  for(int i = 0; i < 4; i++)
  {
    pos[i] *= mn;
  }
}
void pos_corrector()
{
  for(int i = 0; i < 4; i++)
  {
    if(pos[i] > 180)
    {
      pos[i] = 180;
    }
    else if(pos[i] < 0)
    {
      pos[i] = 0;
    }
    // else if(pos[i] < -180)
    // {
    //   pos[i] = -180;
    // }
  }
}
void pos_writer()
{
  arm1.write(pos[0]);
  arm2.write(pos[1]);
  arm3.write(pos[2]);
  arm4.write(pos[3]);
}
void sign_writer()
{
  if(ch1Value < 0)
  {
    ch1Value = 0;
  }
  else if(ch1Value > 100)
  {
    ch1Value = 100;
  }
  if(ch2Value < 0)
  {
    ch2Value = 0;
  }
  else if(ch2Value > 100)
  {
    ch2Value = 100;
  }
  sign1 = ch1Value;
  sign2 = ch2Value;
}