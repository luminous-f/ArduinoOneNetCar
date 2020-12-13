
//引入ESP8266.h头文件，建议使用教程中修改后的文件
#include "ESP8266.h"
#include "SoftwareSerial.h"
#include "dht11.h"
#include <Servo.h>
//#include "Windows.h"

//#include <PubSubClient.h>
#define INTERVAL_SENSOR 5000 //定义传感器采样及发送时间间隔

//创建dht11示例

dht11 DHT11;

//定义DHT11接入Arduino的管脚
#define DHT11PIN 4

int leftCounter=0,  rightCounter=0;
unsigned long time = 0, old_time = 0; // 时间标记
unsigned long time1 = 0; // 时间标记
float lv,rv;//左、右轮速度

#define STOP        0
#define FORWARD     1
#define BACKWARD    2
#define TURNLEFT    3
#define TURNRIGHT   4
#define CHANGESPEED 5

int leftMotor1 = 10;
int leftMotor2 = 11;
int rightMotor1 = 12;
int rightMotor2 = 13;
bool speedLevel=0;
//int cmd=6;
int distance=0;

int leftPWM = 5;
int rightPWM = 6;
Servo myServo;  //舵机

int inputPin=7;   // 定义超声波信号接收接口
int outputPin=8;  // 定义超声波信号发出接口

//配置ESP8266WIFI设置
#define SSID "hi-there"    //填写2.4GHz的WIFI名称，不要使用校园网
#define PASSWORD "1371464193"//填写自己的WIFI密码
#define HOST_NAME "api.heclouds.com"  //API主机名称，连接到OneNET平台，无需修改
#define DEVICE_ID "651024436"       //填写自己的OneNet设备ID
#define HOST_PORT (80)                //API端口，连接到OneNET平台，无需修改
String APIKey = "SNfGlr7QfTy=rC09U1vA5A=0DRY="; //与设备绑定的APIKey

#define INTERVAL_SENSOR 5000 //定义传感器采样及发送时间间隔

//定义ESP8266所连接的软串口
/*********************
 * 该实验需要使用软串口
 * Arduino上的软串口RX定义为D3,
 * 接ESP8266上的TX口,
 * Arduino上的软串口TX定义为D2,
 * 接ESP8266上的RX口.
 * D3和D2可以自定义,
 * 但接ESP8266时必须恰好相反
 *********************/

 SoftwareSerial mySerial(3, 2);
ESP8266 wifi(mySerial);

bool SpeedDetection();
void RightCount_CallBack();
void LeftCount_CallBack();
void motorRun1(int cmd);
void motorRun(int cmd,int value);
void avoidance();
void avoidance2();
int getDistance();




void setup()
{ 
  
  mySerial.begin(115200); //初始化软串口
  Serial.begin(115200);     //初始化串口
  Serial.print("setup begin\r\n");


  //以下为ESP8266初始化的代码
  Serial.print("FW Version: ");
  Serial.println(wifi.getVersion().c_str());

  if (wifi.setOprToStation()) {
    Serial.print("to station ok\r\n");
  } else {
    Serial.print("to station err\r\n");
  }

  //ESP8266接入WIFI
  if (wifi.joinAP(SSID, PASSWORD)){
    Serial.print("Join AP success\r\n");
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } else {
    Serial.print("Join AP failure\r\n");
  }
  Serial.println("");
  Serial.print("DHT11 LIBRARY VERSION: ");
  Serial.println(DHT11LIB_VERSION);
   
    mySerial.println("AT+UART_CUR=9600,8,1,0,0");
    mySerial.begin(9600);
    Serial.println("setup end\r\n");
  
  //舵机引脚初始化
  myServo.attach(9);
  
  attachInterrupt(0,RightCount_CallBack, FALLING);
  attachInterrupt(1,LeftCount_CallBack, FALLING);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
   //超声波控制引脚初始化
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

}

unsigned long net_time1 = millis(); //数据上传服务器时间

void loop(){
   // put your main code here, to run repeatedly:

  //以下OneNet代码


  if (net_time1 > millis())
      net_time1 = millis();

  if (millis() - net_time1 > INTERVAL_SENSOR) //发送数据时间间隔
  {
    int chk = DHT11.read(DHT11PIN);

    Serial.print("Read sensor: ");
    switch (chk) {
      case DHTLIB_OK:
        Serial.println("OK");
        break;
      case DHTLIB_ERROR_CHECKSUM:
        Serial.println("Checksum error");
        break;
      case DHTLIB_ERROR_TIMEOUT:
        Serial.println("Time out error");
        break;
      default:
        Serial.println("Unknown error");
        break;
    }

    float sensor_hum = (float)DHT11.humidity;
    float sensor_tem = (float)DHT11.temperature;
    Serial.print("Humidity (%): ");
    Serial.println(sensor_hum, 2);

    Serial.print("Temperature (oC): ");
    Serial.println(sensor_tem, 2);
    Serial.println("");

    if (wifi.createTCP(HOST_NAME, HOST_PORT)) { //建立TCP连接，如果失败，不能发送该数据
      
      Serial.print("create tcp ok\r\n");
      
      char buf[10];
      //拼接发送data字段字符串 
      String jsonToSend = "{\"DISTANCE\":";
      dtostrf(getDistance(), 2, 2, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += ",\"Humidity\":";
      dtostrf(sensor_tem, 2, 2, buf);
      jsonToSend += "\"" + String(buf) + "\"";
      jsonToSend += "}";

      //拼接POST请求字符串
      String postString = "POST /devices/";
      postString += DEVICE_ID;
      postString += "/datapoints?type=3 HTTP/1.1";
      postString += "\r\n";
      postString += "api-key:";
      postString += APIKey;
      postString += "\r\n";
      postString += "Host:api.heclouds.com\r\n";
      postString += "Connection:close\r\n";
      postString += "Content-Length:";
      postString += jsonToSend.length();
      postString += "\r\n";
      postString += "\r\n";
      postString += jsonToSend;
      postString += "\r\n";
      postString += "\r\n";
      postString += "\r\n";

      const char *postArray = postString.c_str(); //将str转化为char数组

      Serial.println(postArray);
      wifi.send((const uint8_t *)postArray, strlen(postArray)); //nd发送命令，参数必须是这两种格式，尤其是(const uint8_t*)
      Serial.println("send success");
      if (wifi.releaseTCP()) { //释放TCP连接
        Serial.print("release tcp ok\r\n");
      } else {
        Serial.print("release tcp err\r\n");
      }
      postArray = NULL; //清空数组，等待下次传输数据
    } else {
      Serial.print("create tcp err\r\n");
    }

    Serial.println("");

    net_time1 = millis();
  }
   
   SpeedDetection();
   //int cmd = Serial.read();
   //if(Serial.available()>0）
   //{
   //cmd=Serial.parseInt();
   //}//可以试试，只返回有效数据，据说很乖
   
    delay(3);
  if(Serial.available()>0)
  //&&cmd<6&&cmd>=0)
  { int cmd = Serial.read();
    Serial.println(cmd);
    Serial.println("xxx");
    
    //Serial.print(cmd);
    delay(5);
    motorRun1(cmd);
    //delay(200);
    if(speedLevel)  //根据不通的档位输出不同速度
    {
      analogWrite(leftPWM, 120);
      analogWrite(rightPWM, 120);
    }
    else
    {
      analogWrite(leftPWM, 250);
      analogWrite(rightPWM, 250);
    }
    avoidance2();
  }
  else 
     {avoidance2();}
  
}

 /*
 * *速度计算
 */
bool SpeedDetection()
{
  time = millis();//以毫秒为单位，计算当前时间 
  if(abs(time - old_time) >= 1000) // 如果计时时间已达1秒
  {  
    detachInterrupt(0); // 关闭外部中断0
    detachInterrupt(1); // 关闭外部中断1
    //把每一秒钟编码器码盘计得的脉冲数，换算为当前转速值
    //转速单位是每分钟多少转，即r/min。这个编码器码盘为20个空洞。
    lv =(float)leftCounter*60/20;//小车车轮电机转速
    rv =(float)rightCounter*60/20;//小车车轮电机转速
    Serial.print("left:");
    Serial.print(lv);//向上位计算机上传左车轮电机当前转速的高、低字节
    Serial.print("     right:");
    Serial.println(rv);//向上位计算机上传左车轮电机当前转速的高、低字节
    //恢复到编码器测速的初始状态
    leftCounter = 0;   //把脉冲计数值清零，以便计算下一秒的脉冲计数
    rightCounter = 0;
    old_time=  millis();     // 记录每秒测速时的时间节点   
    attachInterrupt(0, RightCount_CallBack,FALLING); // 重新开放外部中断0
    attachInterrupt(1, LeftCount_CallBack,FALLING); // 重新开放外部中断0
    return 1;
  }
  else
    return 0;
}
/*
 * *右轮编码器中断服务函数
 */
void RightCount_CallBack()
{
  rightCounter++;
}
/*
 * *左轮编码器中断服务函数
 */
void LeftCount_CallBack()
{
  leftCounter++;
}
/*小车运动函数**/
void motorRun1(int cmd)
{
  switch(cmd){
    case FORWARD:
      Serial.println("FORWARD"); //输出状态
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      delay(500);
      Stop();
      break;
     case BACKWARD:
      Serial.println("BACKWARD"); //输出状态
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      delay(500);
      Stop();
      break;
     case TURNLEFT:
      Serial.println("TURN  LEFT"); //输出状态
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      delay(200);
      Stop();
      break;
     case TURNRIGHT:
      Serial.println("TURN  RIGHT"); //输出状态
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
       delay(200);
      Stop();
      break;
      case CHANGESPEED:
      Serial.println("CHANGE SPEED"); //输出状态
      if(speedLevel)  //接收到换挡命令的时候切换档位
        speedLevel=0;
      else
        speedLevel=1;
      break;
     default:
      Serial.println("STOP"); //输出状态
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);
  }
}
void Stop(){
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);
  }

void motorRun(int cmd,int value)
{
  analogWrite(leftPWM, value);  //设置PWM输出，即设置速度
  analogWrite(rightPWM, value);
  switch(cmd){
    case FORWARD:
    
      Serial.println("FORWARD"); //输出状态
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      break;
     case BACKWARD:
      Serial.println("BACKWARD"); //输出状态
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      break;
     case TURNLEFT:
      Serial.println("TURN  LEFT"); //输出状态
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      break;
     case TURNRIGHT:
      Serial.println("TURN  RIGHT"); //输出状态
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      break;
     default:
      Serial.println("STOP"); //输出状态
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);
  }
}
void avoidance()
{
  int pos;
  int dis[3];//距离
  motorRun(FORWARD,200);
  myServo.write(90);
  dis[1]=getDistance(); //中间
  
  if(dis[1]<30)
  {
    motorRun(STOP,0);
    for (pos = 90; pos <= 150; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    dis[2]=getDistance(); //左边
    for (pos = 150; pos >= 30; pos -= 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      if(pos==90)
        dis[1]=getDistance(); //中间
    }
    dis[0]=getDistance();  //右边
    for (pos = 30; pos <= 90; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    if(dis[0]<dis[2]) //右边距离障碍的距离比左边近
    {
      //左转
      motorRun(TURNLEFT,250);
      delay(200);
    }
    else  //右边距离障碍的距离比左边远
    {
      //右转
      motorRun(TURNRIGHT,250);
      delay(200);
    } 
  }
}
void avoidance2()
{
  int pos;
  int dis[3];//距离
  //motorRun(FORWARD,200);
  myServo.write(90);
  dis[1]=getDistance(); //中间
  Serial.print("dis[1]=");
  Serial.println(dis[1]);
  if(dis[1]<30)
  {
    motorRun(STOP,0);
    for (pos = 90; pos <= 150; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    dis[2]=getDistance(); //左边
     Serial.print("dis[2]=");
  Serial.println(dis[2]);
    for (pos = 150; pos >= 30; pos -= 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      if(pos==90)
        dis[1]=getDistance(); //中间
    }
    dis[0]=getDistance();  //右边
     Serial.print("dis[0]=");
  Serial.print(dis[0]);
    for (pos = 30; pos <= 90; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    if(dis[0]<dis[2]) //右边距离障碍的距离比左边近
    {
      //左转
      motorRun(TURNLEFT,250);
      delay(200);
    }
    else  //右边距离障碍的距离比左边远
    {
      //右转
      motorRun(TURNRIGHT,250);
      delay(200);
    } 
  }
}

int getDistance()
{
 digitalWrite(outputPin, LOW); // 使发出发出超声波信号接口低电平2μs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); // 保持发出超声波信号接口低电平
  distance = pulseIn(inputPin, HIGH); // 读出脉冲时间
  distance= distance/58; // 将脉冲时间转化为距离（单位：厘米）
  //Serial.println("The distance is:");
  //Serial.println(distance); //输出距离值
 
  if (distance >=50)
  {
    //如果距离小于50厘米返回数据
    return 50;
  }//如果距离小于50厘米小灯熄灭
  else
    return distance;
}
