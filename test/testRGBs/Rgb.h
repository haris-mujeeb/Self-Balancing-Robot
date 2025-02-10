#include "Adafruit_NeoPixel.h"

#define NUMPIXELS 4
#define RGB_PIN 3

class RGB : public Adafruit_NeoPixel {
public:
  RGB() : Adafruit_NeoPixel(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800) {}

  uint32_t led_rgb_new[NUMPIXELS];
  uint32_t led_rgb_old[NUMPIXELS];
  int brightness = 50;
  unsigned long rgb_delay_time = 0;
  unsigned long dispaly_timeout = 0;
  char flag = 0;

  void initialize()
  {
    begin();
    setBrightness(brightness);
    show();
  }
  

  void setColorNew(unsigned char r0, unsigned char g0, unsigned char b0,
                   unsigned char r1, unsigned char g1, unsigned char b1,
                   unsigned char r2, unsigned char g2, unsigned char b2,
                   unsigned char r3, unsigned char g3, unsigned char b3)
  {
    led_rgb_new[0] = Color(r0, g0, b0);
    led_rgb_new[1] = Color(r1, g1, b1);
    led_rgb_new[2] = Color(r2, g2, b2);
    led_rgb_new[3] = Color(r3, g3, b3);
  }
  
  
  void setColorOld(unsigned char r0, unsigned char g0, unsigned char b0,
                   unsigned char r1, unsigned char g1, unsigned char b1,
                   unsigned char r2, unsigned char g2, unsigned char b2,
                   unsigned char r3, unsigned char g3, unsigned char b3)
  {
    led_rgb_old[0] = Color(r0, g0, b0);
    led_rgb_old[1] = Color(r1, g1, b1);
    led_rgb_old[2] = Color(r2, g2, b2);
    led_rgb_old[3] = Color(r3, g3, b3);
  }


  void blink(unsigned long delay_time) {
    if ((millis() - previous_millis < delay_time) && (delay_flag == 0)) {
      delay_flag = 1;
      for (size_t i = 0; i < numPixels(); i++) {
        setPixelColor(i, led_rgb_new[i]);
      }
      show();
    }
    else if ((millis() - previous_millis < delay_time * 2) && (millis() - previous_millis > delay_time) && (delay_flag == 1)) {
      delay_flag = 2;
      for (size_t i = 0; i < numPixels(); i++) {
        setPixelColor(i, led_rgb_old[i]);
      }
      show();
    }
    else if (millis() - previous_millis >= delay_time * 2)
    {
      delay_flag = 0;
      previous_millis = millis();
    }
  }
  

  void lightOff() {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }


  void brightRedColor() {
    setColorNew(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
    setColorOld(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
  }


  void flashRedColor() {
    setColorNew(255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightBlueColor() //亮蓝灯
  {
    setColorNew(0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255);
    setColorOld(0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255);
  }
  void flashBlueColorFront() //前面闪蓝灯
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashBlueColorBack() //后面闪蓝灯
  {
    setColorNew(0, 0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashBlueColorLeft() //左侧闪蓝灯
  {
    setColorNew(0, 0, 0, 0, 0, 255, 0, 0, 255, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashBlueColorRight() //右侧闪蓝灯
  {
    setColorNew(0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightYellowColor() //亮黄灯
  {
    setColorNew(255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0);
    setColorOld(255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0);
  }
  void flashYellowColorFront() //前面闪黄灯
  {
    setColorNew(0, 0, 0, 0, 0, 0, 255, 255, 0, 255, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashYellowColorback() //后面闪黄灯
  {
    setColorNew(255, 255, 0, 255, 255, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashYellowColorLeft() //左侧闪黄灯
  {
    setColorNew(0, 0, 0, 255, 255, 0, 255, 255, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashYellowColorRight() //右侧闪黄灯
  {
    setColorNew(255, 255, 0, 0, 0, 0, 0, 0, 0, 255, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void brightGreenColor() //亮绿灯
  {
    setColorNew(0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0);
    setColorOld(0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0);
  }
  void flashGreenColorFront() //前面闪绿灯
  {
    setColorNew(0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashGreenColorBack() //后面闪绿灯
  {
    setColorNew(0, 255, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashGreenColorLeft() //左侧闪绿灯
  {
    setColorNew(0, 0, 0, 0, 255, 0, 0, 255, 0, 0, 0, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  void flashGreenColorRight() //右侧闪绿灯
  {
    setColorNew(0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0);
    setColorOld(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

private:
  unsigned char delay_flag = 0;
  unsigned long previous_millis = 0;
} rgb;
