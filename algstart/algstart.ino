#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <time.h>
#define CH1 3
#define CH2 9
MPU6050 mpu;
volatile bool mpuFlag = false;  // флаг прерывания готовности
uint8_t fifoBuffer[45];         // буфер

int16_t ai[3];
int16_t gi[3];
float a[3], g[3], e[3] = {0, 0, 0};
float time_to_o;
float pos_o;// ±10
bool is_first = true;
unsigned long lst = 0;

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
template <class t, class f>
void reload(t* arr1, f* arr2, int size) {
  for(int i = 0; i < size; i++) {
    arr1[i] = arr2[i];
  }
}
void ygle(double time) {
  for(int i = 0; i < 3; i++) {
    e[i] = (g[i] - e[i]) / time;
  }
}
void saveg() {
  for(int i = 0; i < 3; i++) {
    e[i] = g[i];
  }
}
void calc(float* ypr) {
  float path = min_path(ypr[2]);
  
}
float min_path(float y) {//кратчайший путь и направление для возвращения к начальному положению
  float path1 = pos_o - y;
  float path2 = (180.0 - abs(y)) + (180.0 - abs(pos_o));
  return path1 > path2 ? (path2 * (-path1/abs(path1))) : path1;//path2 имеет знак противоположный path1
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
    //
    float k = 180.0 / 3.14159265358979;
    mpu.getMotion6(&ai[0], &ai[1], &ai[2], &gi[0], &gi[1], &gi[2]);      // запросить данные
    reload(a, ai, 3);
    reload(g, gi, 3);
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
    if(is_first) {
      is_first = false;
      pos_o = ypr[2];
    }
    unsigned long now = micros();
    double time = (now - lst) / 1000000.0;
    lst = now;
    ygle(time);
    //alg
    //calc();
    //alg
    saveg();
  }
}