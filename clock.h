#define INTERVAL_SENSOR   17000             //定义传感器采样时间间隔  597000
#define INTERVAL_NET      17000             //定义发送时间
//传感器部分================================   
#include <Wire.h>                                  //调用库  
#include "./ESP8266.h"
#include "I2Cdev.h"                                //调用库  
//温湿度   
#include <SHT2x.h>
//光照
#define  sensorPin_1  A0
#define SSID           "111"                   // cannot be longer than 32 characters!
#define PASSWORD       "11111111"
#define humanHotSensor 4//PIR传感器D4
#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

#define HOST_NAME   "api.heclouds.com"
#define DEVICEID   "20427237"
#define PROJECTID "105155"
#define HOST_PORT   (80)
String apiKey="pUpCi73WZHE2uyeAliyrPIRGL=Q=";
char buf[10];
/*==============================================================================*/
/* Useful Constants */
#define SECS_PER_HOUR (3600UL)
#define INTERVAL_LCD             20             //定义OLED刷新时间间隔  
unsigned long lcd_time = millis();                 //OLED刷新时间计时器
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);     //设置OLED型号  
//-------字体设置，大、中、小
#define setFont_L u8g.setFont(u8g_font_7x13)
#define setFont_M u8g.setFont(u8g_font_fixed_v0r)
#define setFont_S u8g.setFont(u8g_font_fixed_v0r)
#define setFont_SS u8g.setFont(u8g_font_fub25n)
long previousMillis = 0;        // 存储LED最后一次的更新
long interval = 5000;           // 闪烁的时间间隔（毫秒）
unsigned long currentMillis=0;
void TemRead();

#ifdef ESP32
#error "This code is not recommended to run on the ESP32 platform! Please check your Tools->Board setting."
#endif

/**
**CoreUSB UART Port: [Serial1] [D0,D1]
**Core+ UART Port: [Serial1] [D2,D3]
**/
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1284P__) || defined (__AVR_ATmega644P__) || defined(__AVR_ATmega128RFA1__)
#define EspSerial Serial1
#define UARTSPEED  115200
#endif

/**
**Core UART Port: [SoftSerial] [D2,D3]
**/
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); /* RX:D2, TX:D3 */

#define EspSerial mySerial
#define UARTSPEED  9600
#endif

#define SSID        "111"
#define PASSWORD    "11111111"

ESP8266 wifi(&EspSerial);

#include <TimeLib.h>

uint8_t buffer[128] = {0};
static uint8_t upd_id = 0;
uint32_t len = 0;
int Year, Month, Day, Hour, Minute, Second, Weekday;
time_t prevDisplay = 0;


#define INTERVAL_sensor 2000
unsigned long sensorlastTime = millis();

float tempOLED, humiOLED, lightnessOLED;

#define INTERVAL_OLED 1000
bool humanHotState = false;
boolean on_off;
boolean statusChange;
String mCottenData;
String jsonToSend;

//3,传感器值的设置 
float sensor_tem, sensor_hum, sensor_lux;                    //传感器温度、湿度、光照   
char  sensor_tem_c[7], sensor_hum_c[7], sensor_lux_c[7] ;    //换成char数组传输
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); /* RX:D3, TX:D2 */
ESP8266 wifi(mySerial);
//ESP8266 wifi(Serial1);                                      //定义一个ESP8266（wifi）的对象
unsigned long net_time1 = millis();                          //数据上传服务器时间
unsigned long sensor_time = millis();                        //传感器采样时间计时器

//int SensorData;                                   //用于存储传感器数据
String postString;                                //用于存储发送数据的字符串
//String jsonToSend;                                //用于存储发送的json格式参数
void updateTimeData() {
  do {
    delay(200);
    registerUDPAndSendRecvData();
    if (len > 0) {
      getTimeStampAndSetRTC();
      unregisterUDP();
    } else {
      unregisterUDP();
    }
  } while (!len);
}
int a;
void getTimeStampAndSetRTC() {
  Serial.print("Received:[");
  unsigned long t = (((unsigned long)buffer[40] << 24) |
                     ((unsigned long)buffer[41] << 16) |
                     ((unsigned long)buffer[42] <<  8) |
                     (unsigned long)buffer[43]) - 2208988800UL;

  Serial.print("Unix timestamp:");
  Serial.print(t);
  Serial.print("]\r\n");

  setTime(t);
  adjustTime(TIME_ZONE * SECS_PER_HOUR);
}

void registerUDPAndSendRecvData() {
  if (wifi.registerUDP(upd_id, HOST_NAME, HOST_PORT)) {
    Serial.print("register udp ");
    Serial.print(upd_id);
    Serial.println(" ok");
  } else {
    Serial.print("register udp ");
    Serial.print(upd_id);
    Serial.println(" err");
  }

  static const char PROGMEM
  timeReqA[] = { 227,  0,  6, 236 }, timeReqB[] = {  49, 78, 49,  52 };
  // Assemble and issue request packet
  uint8_t       buf[48];
  memset(buf, 0, sizeof(buf));
  memcpy_P( buf    , timeReqA, sizeof(timeReqA));
  memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));

  wifi.send(upd_id, (const uint8_t*)buf, 48);
  //uint32_t len = wifi.recv(upd_id, buffer, sizeof(buffer), 10000);
  len = wifi.recv(upd_id, buffer, sizeof(buffer), 10000);
}
void unregisterUDP() {
  if (wifi.unregisterUDP(upd_id)) {
    Serial.print("unregister udp ");
    Serial.print(upd_id);
    Serial.println(" ok");
  } else {
    Serial.print("unregister udp ");
    Serial.print(upd_id);
    Serial.println(" err");
  }
}

//*****串口打印日期时间*****
void serialClockDisplay(int _year, int _month, int _day, int _hour, int _minute, int _second) { 
  Year=_year, Month= _month, Day=_day, Hour=_hour, Minute=_minute, Second=_second;
  if (_year < 1000) {
    Serial.print("20");
  }
  Serial.print(_year, DEC);
  Serial.print('/');
  if (_month < 10) {
    Serial.print("0");
  }
  Serial.print(_month, DEC);
  Serial.print('/');
  if (_day < 10) {
    Serial.print("0");
  }
  Serial.print(_day, DEC);
  Serial.print("   ");
  Serial.print(_hour, DEC);
  Serial.print(':');
  if (_minute < 10) {
    Serial.print("0");
  }
  Serial.print(_minute, DEC);
  Serial.print(':');
  if (_second < 10) {
    Serial.print("0");
  }
  Serial.println(_second, DEC);
  Serial.println();
}
void setup(void)     //初始化函数  
{       
  //初始化串口波特率  
    Wire.begin();
    Serial.begin(115200);   
    while(!Serial);
    pinMode(sensorPin_1, INPUT);

  pinMode(humanHotSensor, INPUT);
   //ESP8266初始化
    Serial.print("setup begin\r\n");   

  Serial.print("FW Version:");
  Serial.println(wifi.getVersion().c_str());

  if (wifi.setOprToStationSoftAP()) {
    Serial.print("to station + softap ok\r\n");
  } else {
    Serial.print("to station + softap err\r\n");
  }

  if (wifi.joinAP(SSID, PASSWORD)) {      //加入无线网
    Serial.print("Join AP success\r\n");  
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } else {
    Serial.print("Join AP failure\r\n");
  }

  if (wifi.disableMUX()) {
    Serial.print("single ok\r\n");
  } else {
    Serial.print("single err\r\n");
  }

  Serial.print("setup end\r\n");
    
  updateTimeData();
}
void loop(void)     //循环函数  
{   

  if(millis() - sensor_time > INTERVAL_SENSOR)              //传感器采样时间间隔  
  {  
    getSensorData();                                        //读串口中的传感器数据
    sensor_time = millis();
  }  

    
  if (net_time1 > millis())  net_time1 = millis();
  
  if (millis() - net_time1 > INTERVAL_NET)                  //发送数据时间间隔
  {                
    updateSensorData();                                     //将数据上传到服务器的函数
    net_time1 = millis();
  }u8g.firstPage();
     do{
      
   light= analogRead(Light_PIN); 
     setFont_L;
     u8g.setPrintPos(0, 10);
     u8g.print(Year,DEC);
     u8g.print("y ");
     u8g.setPrintPos(65, 10);
     u8g.print(Month,DEC);
     u8g.print("m");
     u8g.setPrintPos(0, 25);
     u8g.print(Hour,DEC);
        u8g.print("h");
        u8g.setPrintPos(65, 25);
     u8g.print(Minute,DEC);
        u8g.print("m");
        u8g.setPrintPos(0, 40);
     u8g.print(Second,DEC);
     u8g.print("s");
     
u8g.setPrintPos(65, 40);
     u8g.print(light,DEC);
     u8g.print("lux");
     
     u8g.setPrintPos(0, 55);
     u8g.print(termo.getTemperature());
     u8g.print("c");
     
u8g.setPrintPos(65, 55);
 u8g.print(termo.getHumidity());//打印湿度
 u8g.print("hpa");
   if (sensor_time > millis())  sensor_time = millis();  
  if (now() != prevDisplay) {
    prevDisplay = now();
    serialClockDisplay(year(), month(), day(), hour(), minute(), second());
    }
    if( Hour==23)
       {
    
        tone(buzzer_pin,500);    //在端口输出频率
    delay(5);      //该频率维持5毫秒   
    noTone(buzzer_pin);
}

   
  } while( u8g.nextPage() );
  
}

void getSensorData(){  
    humanHotState = digitalRead(humanHotSensor);
  sensor_tem = SHT2x.readT() ;   
    sensor_hum = SHT2x.readRH();    
    //获取光照
    sensor_lux = analogRead(A0);    
    delay(1000);
    dtostrf(sensor_tem, 2, 1, sensor_tem_c);
    dtostrf(sensor_hum, 2, 1, sensor_hum_c);
    dtostrf(sensor_lux, 3, 1, sensor_lux_c);
}
void updateSensorData() {
  if (wifi.createTCP(HOST_NAME, HOST_PORT)) { //建立TCP连接，如果失败，不能发送该数据
    Serial.print("create tcp ok\r\n");

jsonToSend="{\"Temperature\":";
    dtostrf(sensor_tem,1,2,buf);
    jsonToSend+="\""+String(buf)+"\"";
    
    jsonToSend+=",\"Humidity\":";
    dtostrf(sensor_hum,1,2,buf);
    jsonToSend+="\""+String(buf)+"\"";
    jsonToSend+=",\"Light\":";
    dtostrf(humanHotState,1,2,buf);
    jsonToSend+="\""+String(buf)+"\"";
    jsonToSend+="}";



    postString="POST /devices/";
    postString+=DEVICEID;
    postString+="/datapoints?type=3 HTTP/1.1";
    postString+="\r\n";
    postString+="api-key:";
    postString+=apiKey;
    postString+="\r\n";
    postString+="Host:api.heclouds.com\r\n";
    postString+="Connection:close\r\n";
    postString+="Content-Length:";
    postString+=jsonToSend.length();
    postString+="\r\n";
    postString+="\r\n";
    postString+=jsonToSend;
    postString+="\r\n";
    postString+="\r\n";
    postString+="\r\n";

  const char *postArray = postString.c_str();                 //将str转化为char数组
  Serial.println(postArray);
  wifi.send((const uint8_t*)postArray, strlen(postArray));    //send发送命令，参数必须是这两种格式，尤其是(const uint8_t*)
  Serial.println("send success");   
     if (wifi.releaseTCP()) {                                 //释放TCP连接
        Serial.print("release tcp ok\r\n");
        } 
     else {
        Serial.print("release tcp err\r\n");
        }
      postArray = NULL;                                       //清空数组，等待下次传输数据
  
  } else {
    Serial.print("create tcp err\r\n");
  }
  
}
