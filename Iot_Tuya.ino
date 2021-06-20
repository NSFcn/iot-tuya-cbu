/*
 * @FileName: Iot_Tuya.ino
 * @Author: Genicre
 * @Email: 1406140765@qq.com
 * @Date: 2021-06-12
 */

#include "TuyaWifi.h"
#include <SoftwareSerial.h>
#include <FastLED.h>

#define NUM_LEDS 16             //LED灯珠数量
#define LED_DATA_PIN 6              //Arduino输出控制信号引脚
#define LED_TYPE WS2812         //LED灯带型号
#define COLOR_ORDER GRB         //RGB灯珠中红色、绿色、蓝色LED的排列顺序

SoftwareSerial DebugSerial(8,9);
TuyaWifi my_device;             //建立CBU模块my_device
CRGB Leds[NUM_LEDS];            //建立幻彩光带Leds
CRGBPalette16 Colour_P;
unsigned char led_state = 0;    //指示灯的状态，引脚13
int key_pin = 7;                //配网按键引脚
unsigned int dp_enum_value;
unsigned char Shift_flag,Breath_Brightness,Brightness;
unsigned char DPID_BREATHING_MODE_state,DPID_RUNING_MODE_state;
unsigned char Colour_H,Colour_L,Colour_Offset;
unsigned char Colour_P_i;
//幻彩灯带的功能码
#define DPID_SWITCH 20
#define DPID_WORK_MODE 21
#define DPID_BRIGHT_VALUE 22
#define DPID_COLOUR_DATA 24
#define DPID_BREATHING_MODE 101
#define DPID_RUNING_MODE 102

//将功能码和其数据类型对应起来
unsigned char dp_array[][2] =
{
  {DPID_SWITCH, DP_TYPE_BOOL},
  {DPID_WORK_MODE, DP_TYPE_ENUM},
  {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
  {DPID_COLOUR_DATA, DP_TYPE_STRING},
  {DPID_BREATHING_MODE, DP_TYPE_BOOL},
  {DPID_RUNING_MODE, DP_TYPE_BOOL},
};

unsigned char pid[] = {"djykzfpqxtzcftie"};   //设备id
unsigned char mcu_ver[] = {"1.0.0"};          //软件版本
unsigned long last_time = 0;                  //非阻塞延时时间变量
void setup() 
{DebugSerial.begin(9600);
  Serial.begin(9600);
  //初始化幻彩灯
  LEDS.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(Leds, NUM_LEDS);
  //初始化指示灯，配网按键
  Gpio_Init();
  //验证模组ID和版本号
  my_device.init(pid, mcu_ver);
  //导入功能码
  my_device.set_dp_cmd_total(dp_array, 6);
  //注册上传和回调函数
  my_device.dp_process_func_register(dp_process);
  my_device.dp_update_all_func_register(dp_update_all);
  last_time = millis();
  fill_solid(Leds, NUM_LEDS, CRGB::Black);
  Brightness=50;
  FastLED.setBrightness(Brightness);
  FastLED.show();
  dp_enum_value=5;
}

void loop() 
{
  random16_add_entropy( random());
  //接收模块数据
  my_device.uart_service();
  //按下按键7配网
  Web_Config(key_pin);
  //正常运行指示灯
  //if (my_device.mcu_get_wifi_work_state() == WIFI_CONN_CLOUD)
  if(1)
  {
    if (millis()- last_time >= 10)
    {
        if(Breath_Brightness==255)
          Shift_flag=0;
        if(Breath_Brightness==0)
          Shift_flag=1;
        if(Shift_flag)
          Breath_Brightness++;
        else
          Breath_Brightness--;
      Colour_Offset++;
      //tuya_uart.wifi_uart_write_frame(GET_LOCAL_TIME_CMD, MCU_TX_VER, 0);
      last_time = millis();
      Led_Flash(LED_BUILTIN);
    }
  }
  //灯带的运行模式
  Led_Mode(dp_enum_value);
  if(DPID_BREATHING_MODE_state)
    FastLED.setBrightness(Breath_Brightness);
  else
    FastLED.setBrightness(Brightness);
  FastLED.show();
}
/************************************************
函数功能：接收数据解读
参数：dp点，数据，数据长度
 ************************************************/
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    case DPID_SWITCH:
      dp_enum_value=4;
      led_state = my_device.mcu_get_dp_download_data(dpid, value, length);
      if (led_state) {
        digitalWrite(12, HIGH);
        fill_solid(Leds, 16, CRGB::White);
      } else {
        digitalWrite(12, LOW);
        fill_solid(Leds, NUM_LEDS, CRGB::Black);
      }
      FastLED.show();
      my_device.mcu_dp_update(dpid, led_state, length);
    break;
    case DPID_WORK_MODE:
      dp_enum_value  = my_device.mcu_get_dp_download_data(dpid, value, length);
      if(dp_enum_value==1)
      {
        switch(Colour_P_i){
          case 0: Colour_P=CloudColors_p;break;
          case 1: Colour_P=LavaColors_p;break;
          case 2: Colour_P=HeatColors_p;break;
          case 3: Colour_P=ForestColors_p;break;
          case 4: Colour_P=RainbowColors_p;break;
          case 5: Colour_P=RainbowStripeColors_p;break;
          case 6: Colour_P=PartyColors_p;break;
        }
        Colour_P_i++;
        if( Colour_P_i>=7)
          Colour_P_i=0;
      }
      my_device.mcu_dp_update(dpid, dp_enum_value, length);
    break;
    case DPID_BRIGHT_VALUE:
      Brightness=my_device.mcu_get_dp_download_data(dpid, value, length);
      my_device.mcu_dp_update(DPID_BRIGHT_VALUE,Brightness , length);
      DPID_BREATHING_MODE_state=0;
      my_device.mcu_dp_update(DPID_BREATHING_MODE, DPID_BREATHING_MODE_state,false);
    break;
    case DPID_COLOUR_DATA:
      dp_enum_value=3;
      GetData_and_Show(value,length);
      my_device.mcu_dp_update(dpid, value, length);
    break;
    case DPID_BREATHING_MODE:
      DPID_BREATHING_MODE_state=my_device.mcu_get_dp_download_data(dpid, value, length);
      my_device.mcu_dp_update(dpid, value, length);
    break;
    case DPID_RUNING_MODE:
      DPID_RUNING_MODE_state=my_device.mcu_get_dp_download_data(dpid, value, length);
      my_device.mcu_dp_update(dpid, value, length);
    break;
    default:break;
  }
  return SUCCESS;
}
/************************************************
函数功能：上传状态
 ************************************************/
void dp_update_all(void)
{
  //my_device.mcu_dp_update(DPID_SWITCH, led_state, 1);
}
/************************************************
函数功能：gpio初始化
 ************************************************/

void Gpio_Init(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(key_pin, INPUT_PULLUP);
  pinMode(LED_DATA_PIN, OUTPUT);
  digitalWrite(LED_DATA_PIN, LOW);
  
}
/************************************************
函数功能：LED闪烁
参数：LED引脚
 ************************************************/
void Led_Flash(int pin)
{
  if (led_state == LOW)
    led_state = HIGH;
  else 
    led_state = LOW;
  digitalWrite(pin, led_state); 
}
/************************************************
函数功能：模组配网
参数：配网引脚
 ************************************************/
void Web_Config(int pin)
{
  if (digitalRead(pin) == LOW)
  {
    delay(80);
    fill_solid(Leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    if (digitalRead(pin) == LOW)
    {
     my_device.mcu_set_wifi_mode(SMART_CONFIG);
     while(digitalRead(pin) == LOW);
     fill_solid(Leds, NUM_LEDS, CRGB::Black);
    }
  }
}
/************************************************
函数功能：模组配网
参数：配网引脚
 ************************************************/
 void Led_Mode(unsigned int enum_value)
 {
      switch(enum_value) {
        case 0:
          if(DPID_RUNING_MODE_state)
          {
            Colour_L=0+Colour_Offset;
            Colour_H=230+Colour_Offset;
          }else
          {
            Colour_L=0;
            Colour_H=230;
          }
          fill_gradient(Leds, 0, CHSV(Colour_L, 255,255) , NUM_LEDS-1, CHSV(Colour_H,255,255), LONGEST_HUES);   //彩虹
        break;
        case 1:
          if(DPID_RUNING_MODE_state)
            Palette(Colour_Offset,Colour_P);
          else
            Palette(0,Colour_P);
        break;
        case 2:
          Fire2012WithPalette();
        break;
        default:break;
      }
 }
  /************************************************
函数功能：色板模式
参数：色板号
 ************************************************/
void GetData_and_Show(const unsigned char value[], unsigned short length)
{
  unsigned char H,S;
  unsigned int data[length];
  for (unsigned int i=0; i<length; i++) {
    if(value[i]>='0'&&value[i]<'a')
      data[i] = value[i]-'0';
    else
      data[i] = value[i]-'a'+10;
  }
  H=(unsigned char)((data[1]*256+data[2]*16+data[3])*0.708);
  S=(20+data[5]*256+data[6]*16+data[7])/4;
  fill_solid(Leds, NUM_LEDS, CHSV(H,S,255));
}
 /************************************************
函数功能：色板模式
参数：色板号
 ************************************************/
template <typename PALETTE>
void Palette(unsigned char StartIndex,const  PALETTE& pal)
{
  fill_palette (Leds, NUM_LEDS, StartIndex, 8, pal, 255, LINEARBLEND);
  
}
/************************************************
函数功能：fire模式
 ************************************************/
#define COOLING  55
#define SPARKING 120
void Fire2012WithPalette()
{
  static byte heat[NUM_LEDS];
  for( int i = 0; i < NUM_LEDS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }
  if( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160,255) );
  }
  for( int j = 0; j < NUM_LEDS; j++) {
    byte colorindex = scale8( heat[j], 240);
    CRGB color = ColorFromPalette( HeatColors_p, colorindex);
    int pixelnumber;
    if( false ) {
      pixelnumber = (NUM_LEDS-1) - j;
    } else {
      pixelnumber = j;
    }
    Leds[pixelnumber] = color;
  }
}
