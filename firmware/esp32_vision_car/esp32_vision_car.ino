/* ESP32 classic DevKit 示例，Arduino-ESP32 2.x (PlatformIO 固定版本)。
 * 不适用于 S3/C3 的原样管脚；先确认板卡。低速架空、急停可用才允许上电。
 * V seq v_mps w_radps pan_deg\n -> ACK seq\n
 */
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

constexpr int IN1=18, IN2=19, IN3=21, IN4=22, ENA=25, ENB=26, SERVO=27;
constexpr int CH_LEFT=0, CH_RIGHT=1, CH_SERVO=2;
constexpr float TRACK=0.18f, WHEEL_MAX=0.30f;
constexpr int LEFT_SIGN=1, RIGHT_SIGN=1, SERVO_SIGN=1;
constexpr float SERVO_CENTER_US=1500, SERVO_US_PER_DEG=10;
constexpr unsigned long WATCHDOG_MS=300;
char line[128];
size_t used=0;
bool overflowed=false, haveSequence=false;
unsigned long lastCommand=0, lastSequence=0;

// 先撤销 PWM，再设置方向，避免换向期间通电。
void wheel(int channel, int a, int b, float speed) {
  ledcWrite(channel, 0);
  digitalWrite(a, speed>0 ? HIGH:LOW);
  digitalWrite(b, speed<0 ? HIGH:LOW);
  ledcWrite(channel, (unsigned int)(constrain(fabsf(speed),0.0f,1.0f)*255));
}

// 看门狗和非法帧均调用；L298N 为滑行停止，急停须直接切断驱动电源。
void stopMotors() {
  wheel(CH_LEFT,IN1,IN2,0); wheel(CH_RIGHT,IN3,IN4,0);
}

// 50Hz 舵机脉宽，摄像头向左为正；实机需校准极限避免堵转。
void panTo(float degrees) {
  float us=constrain(SERVO_CENTER_US+SERVO_SIGN*degrees*SERVO_US_PER_DEG,600.0f,2400.0f);
  ledcWrite(CH_SERVO, (unsigned int)(us/20000.0f*65535));
}

// 只接受完整、有限、低速且序号递增的命令；重新连接必须先收到零速握手。
void parseLine() {
  unsigned long seq;
  float v,w,pan;
  int consumed=0;
  if (sscanf(line,"V %lu %f %f %f %n",&seq,&v,&w,&pan,&consumed)!=4 ||
      line[consumed]!='\0' || !isfinite(v) || !isfinite(w) || !isfinite(pan) ||
      fabsf(v)>0.1501f || fabsf(w)>0.5001f || fabsf(pan)>85) {
    stopMotors(); return;
  }
  if (!haveSequence) {
    if (fabsf(v)>0.00001f || fabsf(w)>0.00001f) { stopMotors(); return; }
  } else if (seq<=lastSequence) {
    // Python 重启从 seq=0 开始，只允许零速帧重建会话。
    if (seq!=0 || fabsf(v)>0.00001f || fabsf(w)>0.00001f) { stopMotors(); return; }
  }
  haveSequence=true; lastSequence=seq; lastCommand=millis();
  float left=(v-w*TRACK/2)/WHEEL_MAX, right=(v+w*TRACK/2)/WHEEL_MAX;
  float scale=fmaxf(1.0f,fmaxf(fabsf(left),fabsf(right)));
  wheel(CH_LEFT,IN1,IN2,LEFT_SIGN*left/scale);
  wheel(CH_RIGHT,IN3,IN4,RIGHT_SIGN*right/scale);
  panTo(pan);
  Serial.print("ACK "); Serial.println(seq);
}

// 上电先关闭电机；外部 ENA/ENB 下拉抵御复位时引脚高阻。
void setup() {
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcSetup(CH_LEFT,20000,8); ledcAttachPin(ENA,CH_LEFT);
  ledcSetup(CH_RIGHT,20000,8); ledcAttachPin(ENB,CH_RIGHT);
  ledcSetup(CH_SERVO,50,16); ledcAttachPin(SERVO,CH_SERVO);
  stopMotors(); panTo(0); Serial.begin(115200);
}

// 丢弃超长整行；串口处理预算避免垃圾流饿死独立看门狗。
void loop() {
  for (int count=0; count<256 && Serial.available(); ++count) {
    char c=(char)Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {
      if(!overflowed) { line[used]='\0'; parseLine(); }
      else stopMotors();
      used=0; overflowed=false;
    } else if(used<sizeof(line)-1 && !overflowed) line[used++]=c;
    else { overflowed=true; stopMotors(); }
  }
  if(millis()-lastCommand>WATCHDOG_MS) { stopMotors(); haveSequence=false; }
}
