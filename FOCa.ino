// PWM 输出引脚定义

int pwmA =  32;
int pwmB =  33;
int pwmC =  25 ;

//初始变量及引脚定义

//宏定义实现的约束函数，限制值的范围
//使用三元运算符，把amt的范围约束在low和high之间
#define _constrain(amt,low,high)((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float voltage_limit = 10;   
float voltage_supply = 12.6;     //定义板子的供电电压
float shaft_angle = 0,open_loop_timestamp = 0 ;
float zero_electric_angle = 0;
float Ualpha = 0;
float Ubeta = 0;
float Ua=0, Ub=0, Uc=0, dc_a=0, dc_b=0, dc_c=0;



void setup() {
  // put your setup code here, to run once:
  //只跑一次做初始化,放置一些加载项

  //初始化串口
  Serial.begin(115200);   

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);  //V4电机使能
  
  //设置PWM
  pinMode(pwmA,OUTPUT);  //把引脚配置为输出状态，使其可以主动驱动电路输出高低电平或者是输出pwm信号
  pinMode(pwmB,OUTPUT);
  pinMode(pwmC,OUTPUT);
  ledcAttachPin(pwmA,0);   //把引脚pwmA绑定到芯片内部PWM通道上
  ledcAttachPin(pwmB,1);
  ledcAttachPin(pwmC,2); 

 // ESP32 LEDC PWM 通道参数配置函数    (pwm通道号，pwm开关频率，分辨率位数)
  ledcSetup(0,30000,8);     
  ledcSetup(1,30000,8);
  ledcSetup(2,30000,8);
  Serial.println("完成PWN初始化设置");
  delay(3000);

}

//写完setup函数就可以在后边写一些具体的功能函数


//电角度求解
//pole_paries 极对数
float _electricalAngel(float shaft_angle,float pole_paries)
{
  return (shaft_angle * pole_paries ) ;
}

//归一化角度到【0，2PI】
float _normalizeAngle(float angle)     //fomd 是c++标准数学库的浮点数求模（取余）
{
  float a = fmod (angle,2*PI);
  return a>= 0 ? a : (a + 2*PI);   //如果 a>= 0说明角度本就是正余数，直接返回 a。如果 a<0,说明余数是负数，给它加上一个整周期 2PI，将其平移到正数区间 [0, 2PI)。
}

//设置PWN到控制器的输出

void  setPwm(float Ua,float Ub,float Uc )   //计算三相电压 、占空比、最终以pwm的形式去驱动电机
{

  //限制上限
  Ua = _constrain(Ua,0.0f,voltage_limit);
  Ub = _constrain(Ub,0.0f,voltage_limit);
  Uc = _constrain(Uc,0.0f,voltage_limit);

  //计算占空比
  //限制占空比从0到1
  dc_a = _constrain(Ua/voltage_supply,0.0f,1.0f);
  dc_b = _constrain(Ub/voltage_supply,0.0f,1.0f);
  dc_c = _constrain(Uc/voltage_supply,0.0f,1.0f);

//把占空比写入PWM到PWM 0 1 2 通道
  ledcWrite(0,dc_a * 255);
  ledcWrite(0,dc_b * 255);
  ledcWrite(0,dc_c * 255);

}
//然后开始按照foc环节，加入到克拉克变换和帕克变换


 //Uq决定了力矩的大小

void setPhaseVoltage(float Uq,float Ud,float angle_el)           //进行park逆变换、克拉克逆变换、并且调用setpwm函数
{
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);   //加上一个初始零点电角度/初始电角度偏移
 
  //帕克变换      //简单模型中忽略Ud，只保留Uq
  Ualpha = -Uq * sin(angle_el);
  Ubeta = Uq * cos(angle_el);

  //克拉克逆变换
  // voltage_supply/2,电机供电电压/2，将生成的Ua、Ub、Uc平移到供电电压的中间值上，避免产生小于零的部分
  Ua = Ualpha + voltage_supply/2;
  Ub = (sqrt(3) * Ubeta - Ualpha)/2 + voltage_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta)/2 + voltage_supply / 2;

  setPwm(Ua,Ub,Uc);
}

//开环速度函数
float velocityOpenloop(float target_velocity)
{
  unsigned long now_us = micros();  //获取从开启芯片以来的微秒数,它的精度是 4 微秒。micros()返回的是一个无符号长整型(unsigned long)的值

  //计算当前每个Loop的运行时间间隔
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数,在由70分钟跳变到0时,Ts会出现异常,因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒,则将其设置为一个较小的默认值,即 1e-3f
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;     //防止溢出

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度,存储在 shaft_angle 变量中。在此之前,还需要对轴角度进行归一化,以确保其值在 0 到 2π 之间。
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);  //shaft_angle 是一个主轴的机械角度，因为这个开环的函数没有编码传感器来反馈真实的物理位置，所以需要给一个虚拟的参考角度
  //以目标速度为 10 rad/s 为例,如果时间间隔是 1 秒,则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量,才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒,那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度,才能实现相同的目标速度。因此,电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // 使用早前设置的voltage_limit作为Uq值。这个值会直接影响输出力矩
  float Uq = voltage_limit;

  setPhaseVoltage(Uq,  0, _electricalAngel(shaft_angle, 7));     //通过当前的主轴角度以及给定的角度来确定当前的主轴位置，函数得到Uq、Ud、当前主轴的电气位置后，进行快速的逆帕克变换，把Uq、Ud转换为Ualpha、Ubeta
      //通过这个函数输出PWM波形
  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return Uq;
}
void loop() {
  // put your main code here, to run repeatedly:
  //相当于 main
  velocityOpenloop(5);      //设置目标角速度
}
