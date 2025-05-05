//========================================================================================================================//
//                                                 用户自定义宏定义                            d:\biyelunwen\毕业论文\龙\Chinese-version\src\MPU6050\MPU6050.h                     //                                                                 
//===================================================

// 接收机配置
#define USE_SBUS_RX  // 使用S.Bus接收机协议

// 惯性测量单元配置
#define USE_MPU6050_I2C  // 使用I2C接口的MPU6050传感器

// 陀螺仪满量程范围（度/秒）
#define GYRO_250DPS  // 选择250度/秒量程

// 加速度计满量程范围（G值） 
#define ACCEL_2G  // 选择2G量程


//========================================================================================================================//

// 引入库文件
#include <Wire.h>     // I2C通信库
#include <SPI.h>      // SPI通信库
#include <PWMServo.h> 
#include <AS5047P.h>  // AS5047P磁性编码器驱动库

#include "src/SBUS/SBUS.h"  // S.Bus协议库 
#include "src/MPU6050/MPU6050.h" // MPU6050驱动库
MPU6050 mpu6050;  // 创建MPU6050对象


//========================================================================================================================//

// AS5047P编码器配置

// 引脚定义：
// 绿色 - 片选信号(CS) - 引脚10
// 黄色 - 主输出从输入(MOSI) - 引脚11
// 白色 - 主输入从输出(MISO) - 引脚12c:\Users\1\Desktop\111swashplateless_helicopter-main\swashplateless_helicopter-main\code\v2\Chinese-version\radioComm.ino
// 蓝色 - 时钟信号(SCK) - 引脚13

#define AS5047P_CHIP_SELECT_PORT 10  // 片选引脚
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000  // 自定义SPI总线速率（100kHz）

AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);  // 初始化编码器对象


//========================================================================================================================//

// 陀螺仪和加速度计量程选择及比例因子配置

// 陀螺仪量程选择定义
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
// 加速度计量程选择定义
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

// 当前量程配置
#define GYRO_SCALE GYRO_FS_SEL_250  // 选择250度/秒量程
#define GYRO_SCALE_FACTOR 131.0     // 250dps量程的比例因子

#define ACCEL_SCALE ACCEL_FS_SEL_2  // 选择2G量程
#define ACCEL_SCALE_FACTOR 16384.0  // 2G量程的比例因子


//========================================================================================================================//
//                                               用户自定义变量                                                 //                           
//========================================================================================================================//

// 无线电失效保护值（单位：微秒）
unsigned long channel_1_fs = 1000; // 油门通道
unsigned long channel_2_fs = 1500; // 副翼通道
unsigned long channel_3_fs = 1500; // 升降舵通道
unsigned long channel_4_fs = 1500; // 方向舵通道
unsigned long channel_5_fs = 2000; // 襟翼通道（大于1500为油门切断）
unsigned long channel_6_fs = 2000; // 辅助通道1

// 滤波器参数（默认值针对2kHz循环速率优化，不建议修改）
float B_madgwick = 0.04;  // 马格维奇滤波器参数
float B_accel = 0.14;     // 加速度计低通滤波参数（MPU6050默认：0.14）
float B_gyro = 0.1;       // 陀螺仪低通滤波参数（MPU6050默认：0.1）
float B_mag = 1.0;        // 磁力计低通滤波参数

// IMU校准参数（需通过calculate_IMU_error()函数校准，注释掉校准代码后保留这些值）
float AccErrorX = 0.0;  // 加速度计X轴误差补偿
float AccErrorY = 0.0;  // 加速度计Y轴误差补偿
float AccErrorZ = 0.0;  // 加速度计Z轴误差补偿
float GyroErrorX = 0.0; // 陀螺仪X轴误差补偿
float GyroErrorY= 0.0;  // 陀螺仪Y轴误差补偿
float GyroErrorZ = 0.0; // 陀螺仪Z轴误差补偿

// 控制器参数（注意默认值）：
float i_limit = 25.0;     // 积分限幅值（安全保护）
float maxRoll = 30.0;     // 最大滚转角（角度模式）/最大滚转角速率（速率模式）
float maxPitch = 30.0;    // 最大俯仰角（角度模式）/最大俯仰角速率（速率模式）
float maxYaw = 160.0;     // 最大偏航角速率（度/秒）

// 角度模式PID参数
float Kp_roll_angle = 0.2;    // 滚转角P增益
float Ki_roll_angle = 0.3;    // 滚转角I增益
float Kd_roll_angle = 0.05;   // 滚转角D增益
float B_loop_roll = 0.9;      // 滚转阻尼系数（0-1之间）
float Kp_pitch_angle = 0.2;   // 俯仰角P增益
float Ki_pitch_angle = 0.3;   // 俯仰角I增益
float Kd_pitch_angle = 0.05;  // 俯仰角D增益
float B_loop_pitch = 0.9;     // 俯仰阻尼系数

// 速率模式PID参数
float Kp_roll_rate = 0.15;    // 滚转角速率P增益
float Ki_roll_rate = 0.2;     // 滚转角速率I增益
float Kd_roll_rate = 0.0002;  // 滚转角速率D增益
float Kp_pitch_rate = 0.15;   // 俯仰角速率P增益
float Ki_pitch_rate = 0.2;    // 俯仰角速率I增益
float Kd_pitch_rate = 0.0002; // 俯仰角速率D增益

float Kp_yaw = 0.3;           // 偏航角速率P增益
float Ki_yaw = 0.05;          // 偏航角速率I增益
float Kd_yaw = 0.00015;       // 偏航角速率D增益


//========================================================================================================================//
//                                                   引脚声明                                                       //                           
//========================================================================================================================//                                          

// 无线电接收机引脚：
const int ch1Pin = 15;   // 油门通道
const int ch2Pin = 16;   // 副翼通道
const int ch3Pin = 17;   // 升降舵通道
const int ch4Pin = 20;   // 方向舵通道
const int ch5Pin = 21;   // 襟翼通道（油门切断）
const int ch6Pin = 22;   // 辅助通道1
const int PPM_Pin = 23;  // PPM信号输入

// OneShot125电调输出引脚：
const int m2Pin = 1;       // 电机控制引脚

// PWM舵机/电调输出：
const int servo1Pin = 0;   // 舵机控制引脚

PWMServo servo1;  // 创建舵机控制对象


//========================================================================================================================//
//                                                   全局变量                                                       //                           
//========================================================================================================================//

// 通用变量
float dt;                   // 循环时间间隔
unsigned long current_time, prev_time;  // 当前时间和上一循环时间
unsigned long print_counter, serial_counter;// 打印计数器

// 无线电通信相关：
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

SBUS sbus(Serial5);         // 创建S.Bus对象
uint16_t sbusChannels[16];  // 存储S.Bus通道值
bool sbusFailSafe;          // S.Bus失效保护状态
bool sbusLostFrame;         // S.Bus帧丢失标志

// AS5047P编码器相关：
float motorRads;            // 电机旋转弧度（0-1范围）

// IMU相关：
float AccX, AccY, AccZ;     // 加速度计原始数据
float AccX_prev, AccY_prev, AccZ_prev;  // 上一次加速度数据
float GyroX, GyroY, GyroZ;  // 陀螺仪原始数据
float GyroX_prev, GyroY_prev, GyroZ_prev; // 上一次陀螺数据

float roll_IMU, pitch_IMU, yaw_IMU;     // 经过滤波的IMU姿态角
float roll_IMU_prev, pitch_IMU_prev;    // 上一次姿态角

// 四元数初始化（马格维奇滤波器）
float q0 = 1.0f; // 初始化四元数w分量
float q1 = 0.0f; // 初始化四元数x分量
float q2 = 0.0f; // 初始化四元数y分量
float q3 = 0.0f; // 初始化四元数z分量

// 归一化期望状态：
float thro_des, roll_des, pitch_des, yaw_des;  // 归一化的期望控制量
float roll_passthru, pitch_passthru, yaw_passthru; // 直通控制量

// 控制器相关：
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol;
float integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol;
float integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// 混合器相关：
float s1_command_scaled, m2_command_scaled;  // 缩放后的控制信号
int s1_command_PWM, m2_command_PWM;          // PWM输出值

// 飞行状态：
bool armedFly = false;  // 飞行解锁标志


//========================================================================================================================//
//                                                   主程序setup()函数                                               //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); // 初始化USB串口通信（500kbps）
  delay(500);
  
  // 初始化所有引脚
  pinMode(m2Pin, OUTPUT);  // 设置电机控制引脚为输出模式
  servo1.attach(servo1Pin, 900, 2100);  // 初始化舵机对象（PWM范围900-2100μs）
  delay(5);

  // 初始化无线电通信
  radioSetup();
  
  // 设置遥控器通道安全值
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  // 初始化AS5047P编码器
  AS5047Pinit();
  delay(5);

  // 初始化IMU通信
  IMUinit();
  delay(5);

  // 解锁舵机通道
  servo1.write(0);  // 舵机初始位置（0度对应900μs）
  delay(5);

  // 解锁电调
  m2_command_PWM = 125;  // OneShot125协议解锁脉冲（125-250μs）
  armMotors();           // 循环发送解锁信号直到电调响应
}


//========================================================================================================================//
//                                                   主循环loop()函数                                                 //                           
//========================================================================================================================//

void loop() {
  // 记录时间信息
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;  // 计算循环时间间隔（秒）

  //-----------------调试信息输出-----------------//
  
  // 每100ms输出一次调试数据（取消注释需要的调试项）：
  // printRadioData();       // 输出遥控器通道值（1000-2000范围）
  // printDesiredState();    // 输出期望的飞行状态
  // printAS5047PData();     // 输出编码器数据
  printGyroData();          // 输出陀螺仪数据（-250~250度/秒）
  // printAccelData();       // 输出加速度计数据（-2~2G）
  // printRollPitchYaw();    // 输出姿态角（度）
  // printPIDoutput();       // 输出PID控制器输出
  // printMotorCommands();   // 输出电机控制信号（125-250μs）
  // printLoopRate();        // 输出循环频率（微秒/循环）

  // 获取当前解锁状态
  armedStatus(); // 检查油门是否低于安全值且襟翼通道高于安全值

  // 获取电机旋转角度
  getMotorRads();

  // 获取IMU数据
  getIMUdata(); // 读取原始数据并进行滤波处理
  Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt); // 姿态解算

  // 计算期望飞行状态
  getDesState(); // 将原始通道值转换为归一化控制量

  // PID控制器（选择一种模式）
  controlANGLE(); // 角度模式控制（姿态稳定）

  // 执行器混合与信号缩放
  controlMixer(); // 混合PID输出到执行器指令
  scaleCommands(); // 将归一化指令缩放到PWM范围

  // 油门切断检测
  throttleCut(); // 根据襟翼通道状态切断电机

  // 发送执行器信号
  commandMotors(); // 输出PWM信号驱动电机和舵机
  servo1.write(s1_command_PWM); // 更新舵机位置

  // 获取下一循环指令
  getCommands(); // 读取最新遥控器信号
  failSafe();    // 处理遥控信号丢失情况

  // 维持循环频率
  loopRate(2000); // 限制循环频率不超过2000Hz（所有参数针对此频率优化）
}


//========================================================================================================================//
//                                                   功能函数                                                       //                           
//========================================================================================================================//

void controlMixer() {
  // 功能：混合PID输出到执行器指令
  // 说明：根据姿态角和油门值计算电机和舵机控制信号
  
  s1_command_scaled = thro_des + abs(pitch_PID) + abs(roll_PID) + (pitch_PID * cos(motorRads)) + (roll_PID * sin(motorRads));
  m2_command_scaled = (thro_des/10) + (0.5 + yaw_PID);  // 舵机控制信号（中心偏移补偿）
}

void armedStatus() {
  // 功能：检测解锁条件
  // 条件：襟翼通道高于安全值且油门低于安全阈值
  if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050) && (channel_2_pwm > 1050)) {
    armedFly = true;  // 解锁飞行状态
  }
}

void AS5047Pinit() {
  // 功能：初始化AS5047P编码器
  while (!as5047p.initSPI())  // 等待SPI初始化完成
  {
    Serial.println("AS5047P初始化失败");
    Serial.println("请检查编码器接线或尝试重启");
    while (1) {}  // 进入死循环等待修复
  }
}

void IMUinit() {
  // 功能：初始化MPU6050传感器
  
  Wire.begin();                 // 初始化I2C总线
  Wire.setClock(1000000);       // 设置I2C时钟频率（1MHz）
  
  mpu6050.initialize();         // 初始化MPU6050
  
  if (mpu6050.testConnection() == false) {  // 验证传感器连接
    Serial.println("MPU6050连接失败");
    Serial.println("请检查传感器接线或尝试重启");
    while(1) {}  // 进入死循环等待修复
  }

  // 设置量程范围（根据宏定义配置）
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

void getMotorRads() {
  // 功能：获取电机旋转角度（0-1范围）
  float angle = as5047p.readAngleDegree();  // 读取编码器角度（0-360度）
  
  // 将角度转换为0-1范围（考虑旋转方向）
  if (angle <= 180) {
    motorRads = angle / 180.0;  // 0-180度映射到0-1
  } else {
    motorRads = (360 - angle) / 180.0;  // 180-360度映射到1-0
  }
}

void getIMUdata() {
  // 功能：获取并处理IMU数据
  
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  
  mpu6050.getMotion6(&AcX, &AcZ, &AcY, &GyX, &GyZ, &GyY);  // 读取原始数据

  // 处理加速度计数据（单位转换和轴方向校正）
  AccX = AcX / ACCEL_SCALE_FACTOR;  // 转换为G值
  AccY = -AcY / ACCEL_SCALE_FACTOR; // 反转Y轴方向
  AccZ = AcZ / ACCEL_SCALE_FACTOR;

  // 应用校准补偿值
  AccX -= AccErrorX;
  AccY -= AccErrorY;
  AccZ -= AccErrorZ;

  // 加速度计低通滤波
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  // 处理陀螺仪数据（单位转换和轴方向校正）
  GyroX = GyX / GYRO_SCALE_FACTOR;  // 转换为度/秒
  GyroY = -GyY / GYRO_SCALE_FACTOR; // 反转Y轴方向
  GyroZ = GyZ / GYRO_SCALE_FACTOR;

  // 应用校准补偿值
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  // 陀螺仪低通滤波
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  // 功能：马格维奇四元数姿态解算算法
  // 说明：融合陀螺仪和加速度计数据进行姿态估计
  
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // 将陀螺仪数据从度/秒转换为弧度/秒
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // 计算四元数导数（基于陀螺仪数据）
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // 当加速度计数据有效时进行梯度下降校正
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 归一化加速度数据
    recipNorm = sqrt(ax*ax + ay*ay + az*az);
     ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // 计算辅助变量
     _2q0 = 2.0f * q0;
     _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
     _2q3 = 2.0f * q3;
     _4q0 = 4.0f * q0;
     _4q1 = 4.0f * q1;
     _4q2 = 4.0f * q2;
     _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
     q1q1 = q1 * q1;
     q2q2 = q2 * q2;
     q3q3 = q3 * q3;

    // 计算梯度下降校正量
     s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
     s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
     s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
     s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    // 归一化校正量
     recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 更新四元数导数
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  // 四元数积分（姿态更新）
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  // 四元数归一化
   recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // 四元数转欧拉角
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29577951;  // 滚转角（度）
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2), -0.999999, 0.999999)) * 57.29577951;  // 俯仰角（度）
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29577951;  // 偏航角（度）
}

void getDesState() {
  // 功能：将原始通道值转换为归一化控制量
  
  // 油门通道（0-1范围）
  thro_des = (channel_1_pwm - 1000.0)/1000.0;  
  
  // 副翼/升降舵/方向舵通道（-1~1范围）
  roll_des = (channel_2_pwm - 1500.0)/500.0;   
  pitch_des = (channel_3_pwm - 1500.0)/500.0; 
  yaw_des = (channel_4_pwm - 1500.0)/500.0;   
  
  // 直通控制量（-0.5~0.5范围）
  roll_passthru = roll_des/2.0;   
  pitch_passthru = pitch_des/2.0; 
  yaw_passthru = yaw_des/2.0; 
  
  // 限制控制量范围
  thro_des = constrain(thro_des, 0.0, 1.0);   
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll;   
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; 
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw;   
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlANGLE() {
  // 功能：基于角度误差的PID控制
  
  // 滚转控制
  error_roll = roll_des - roll_IMU;                      // 计算角度误差
  integral_roll = integral_roll_prev + error_roll*dt;    // 积分项
  if (channel_1_pwm < 1060) integral_roll = 0;           // 低油门时禁止积分
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  // 积分限幅
  derivative_roll = GyroX;                               // 微分项（使用陀螺仪数据）
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll);  // PID输出
  
  // 俯仰控制（结构同滚转）
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch);
  
  // 偏航控制（基于陀螺仪速率）
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  derivative_yaw = (error_yaw - error_yaw_prev)/dt;  // 计算角速率误差
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw);  // PID输出
  
  // 更新历史变量
  integral_roll_prev = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  // 功能：将归一化控制量转换为PWM信号
  
  // 电机控制信号缩放（125-250μs）
  m2_command_PWM = m2_command_scaled*125 + 125;  // 缩放并偏移
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);  // 限制范围
  
  // 舵机控制信号缩放（0-180度）
  s1_command_PWM = s1_command_scaled*180;        // 缩放
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);  // 限制范围
}

void getCommands() {
  // 功能：获取遥控器通道值
  
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {  // 读取S.Bus数据
    // S.Bus通道值缩放（根据接收机特性调整）
    float scale = 0.615;  // 缩放系数
    float bias  = 895.0;  // 偏移量
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias; 
  }

  // 低通滤波关键通道（减少噪声）
  float b = 0.7;  // 滤波系数（0-1，越大越平滑）
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  
  // 更新历史值
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  // 功能：失效保护处理
  
  // 检测无效通道值
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = (channel_1_pwm < minVal || channel_1_pwm > maxVal) ? 1 : 0;
  int check2 = (channel_2_pwm < minVal || channel_2_pwm > maxVal) ? 1 : 0;
  int check3 = (channel_3_pwm < minVal || channel_3_pwm > maxVal) ? 1 : 0;
  int check4 = (channel_4_pwm < minVal || channel_4_pwm > maxVal) ? 1 : 0;
  int check5 = (channel_5_pwm < minVal || channel_5_pwm > maxVal) ? 1 : 0;
  int check6 = (channel_6_pwm < minVal || channel_6_pwm > maxVal) ? 1 : 0;
  
  // 如果任一通道异常，恢复安全值
  if (check1 + check2 + check3 + check4 + check5 + check6 > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void commandMotors() {
  // 功能：输出电机控制信号（OneShot125协议）
  
  int wentLow = 0;
  int pulseStart, timer;
  int flagM2 = 0;
  
  digitalWrite(m2Pin, HIGH);  // 拉高电平开始脉冲
  pulseStart = micros();      // 记录脉冲起始时间
  
  // 等待脉冲结束
  while (wentLow < 1 ) { 
    timer = micros();
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);  // 拉低完成脉冲
      wentLow = 1;
      flagM2 = 1;
    }
  }
}

void armMotors() {
  // 功能：解锁电调（发送多次解锁信号）
  for (int i = 0; i <= 50; i++) {
    commandMotors();  // 发送解锁脉冲
    delay(2);         // 短暂延时
  }
}

void throttleCut() {
  // 功能：油门切断处理
  if ((channel_5_pwm < 1500) || (armedFly == false)) {
    armedFly = false;  // 解锁状态置否
    
    m2_command_PWM = 184;    // 最小油门（184μs）
    s1_command_PWM = 0;      // 舵机归零
  }
}

void loopRate(int freq) {
  // 功能：维持固定循环频率
  
  float invFreq = 1.0/freq*1000000.0;  // 计算周期（微秒）
  unsigned long checker = micros();
  
  // 等待直到达到指定周期
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}


//========================================================================================================================//
//                                                   调试函数                                                       //                           
//========================================================================================================================//

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print("CH1:");
    Serial.print(channel_1_pwm);
    Serial.print(",CH2:");
    Serial.print(channel_2_pwm);
    Serial.print(",CH3:");
    Serial.print(channel_3_pwm);
    Serial.print(",CH4:");
    Serial.print(channel_4_pwm);
    Serial.print(",CH5:");
    Serial.print(channel_5_pwm);
    Serial.print(",CH6:");
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print(F("油门:"));
    Serial.print(thro_des);
    Serial.print(F(" 滚转:"));
    Serial.print(roll_des);
    Serial.print(F(" 俯仰:"));
    Serial.print(pitch_des);
    Serial.print(F(" 偏航:"));
    Serial.println(yaw_des);
  }
}

void printAS5047PData(){
  if (current_time - print_counter > 500) {
    print_counter = micros();
    
    Serial.print("电机角度:");
    Serial.println(motorRads, 6);  // 打印6位小数
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print("陀螺X:");
    Serial.print(GyroX);
    Serial.print("°/s Y:");
    Serial.print(GyroY);
    Serial.print("°/s Z:");
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F("g AccY:"));
    Serial.print(AccY);
    Serial.print(F("g AccZ:"));
    Serial.println(AccZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print(F("Roll:"));
    Serial.print(roll_IMU);
    Serial.print(F("° Pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F("° Yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print("Roll_PID:");
    Serial.print(roll_PID);
    Serial.print(" Pitch_PID:");
    Serial.print(pitch_PID);
    Serial.print(" Yaw_PID:");
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10) {
    print_counter = micros();
    
    Serial.print("Servo PWM:");
    Serial.print(s1_command_PWM);
    Serial.print(" Motor PWM:");
    Serial.println(m2_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    
    Serial.print(F("循环时间:"));
    Serial.print(dt*1000000.0);
    Serial.println("μs");
  }
}


//========================================================================================================================//
//                                                   工具函数                                                       //                           
//========================================================================================================================//

float invSqrt(float x) {
  // 功能：快速计算平方根倒数（牛顿迭代法优化实现）
  return 1.0/sqrtf(x); 
}
