#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#define CH1 3
#define CH2 9
MPU6050 mpu;
volatile bool mpuFlag = false;  // флаг прерывания готовности
uint8_t fifoBuffer[45];         // буфер

int16_t a[3];
int16_t g[3];

int ch1Value;
int ch2Value;

Servo arm1, arm2, arm3, arm4;

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
template <class t> 
void kof(float k, t* arr, int size) {
  for(int i = 0; i < size; i++) {
    arr[i] *= k;
  }
}
void pr(float* arr, int size) {
  for(int i = 0; i < size; i++) {
    Serial.print(arr[0]); // вокруг оси Z
    if(size - i > 1) {
      Serial.print(',');
    }
    else {
      Serial.println();
    }
  }
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  //Wire.setClock(1000000UL);     // разгоняем шину на максимум
  // инициализация DMP и прерывания
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpReady, RISING);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  arm1.attach(5);
  arm2.attach(6);
  arm3.attach(7);
  arm4.attach(8);
}
// прерывание готовности. Поднимаем флаг
void dmpReady() {
  mpuFlag = true;
}
void loop() {
  // по флагу прерывания и готовности DMP
  if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // переменные для расчёта (ypr можно вынести в глобал)
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    // расчёты
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuFlag = false;
    // выводим результат в радианах (-3.14, 3.14)
    float k = 180.0 / 3.14159265358979;
    /*Serial.print(ypr[0] * k); // вокруг оси Z
    Serial.print(',');
    Serial.print(ypr[1] * k); // вокруг оси Y
    Serial.print(',');
    Serial.print(ypr[2] * k); // вокруг оси X
    Serial.println();*/
    // для градусов можно использовать degrees()
    mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);      // запросить данные

    kof(k, ypr, 3);
    kof(16.0 / 32768.0, a, 3);
    kof(250.0 / 32768.0, g, 3);
    //pr(a, 3);
    //pr(g, 3);
    pr(ypr, 3);
    ch1Value = readChannel(CH1, 0, 100, 0);
    ch2Value = readChannel(CH2, 0, 100, 0);
    Serial.print("Ch1: ");
    Serial.print(ch1Value);
    Serial.print(" | Ch2: ");
    Serial.println(ch2Value);
  }
}