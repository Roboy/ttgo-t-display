#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "bmp.h"
#include <Arduino.h>
#include <analogWrite.h>
int brightness[6] = {0};

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23

#define TFT_BL          4  // Display backlight control pin
#define ADC_EN          14
#define ADC_PIN         34
#define BUTTON_1        35
#define BUTTON_2        0
#define GATE0 12
#define GATE1 13
#define GATE2 15
#define GATE3 2
#define GATE4 17
#define GATE5 22

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

char buff[512];
int vref = 1100;
int btnCick = false;

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V";
        Serial.println(voltage);
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(voltage,  tft.width() / 2, tft.height() / 2 );
    }
}

void button_init()
{
    btn1.setLongClickHandler([](Button2 & b) {
      brightness[0]=0;
      int32_t color = (brightness[0]<<16|brightness[0]<<8|brightness[0]);
      tft.fillScreen(color);
      analogWrite(GATE0, brightness[0]);
      analogWrite(GATE1, brightness[0]);
      analogWrite(GATE2, brightness[0]);
      analogWrite(GATE3, brightness[0]);
      analogWrite(GATE4, brightness[0]);
      analogWrite(GATE5, brightness[0]);
    });

    btn1.setPressedHandler([](Button2 & b) {

      brightness[0]-=10;
      if(brightness[0]<=0){
        brightness[0] = 0;
      }
      int32_t color = (brightness[0]<<16|brightness[0]<<8|brightness[0]);
      tft.fillScreen(color);
      analogWrite(GATE0, brightness[0]);
      analogWrite(GATE1, brightness[0]);
      analogWrite(GATE2, brightness[0]);
      analogWrite(GATE3, brightness[0]);
      analogWrite(GATE4, brightness[0]);
      analogWrite(GATE5, brightness[0]);
    });

    btn2.setLongClickHandler([](Button2 & b) {
      brightness[0]=255;
      int32_t color = (brightness[0]<<16|brightness[0]<<8|brightness[0]);
      tft.fillScreen(color);
      analogWrite(GATE0, brightness[0]);
      analogWrite(GATE1, brightness[0]);
      analogWrite(GATE2, brightness[0]);
      analogWrite(GATE3, brightness[0]);
      analogWrite(GATE4, brightness[0]);
      analogWrite(GATE5, brightness[0]);
    });

    btn2.setPressedHandler([](Button2 & b) {
      brightness[0]+=10;
      if(brightness[0]>=255){
        brightness[0] = 255;
      }
      int32_t color = (brightness[0]<<16|brightness[0]<<8|brightness[0]);
      tft.fillScreen(color);
      analogWrite(GATE0, brightness[0]);
      analogWrite(GATE1, brightness[0]);
      analogWrite(GATE2, brightness[0]);
      analogWrite(GATE3, brightness[0]);
      analogWrite(GATE4, brightness[0]);
      analogWrite(GATE5, brightness[0]);
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);

    if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
         pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
         digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }

    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  240, 135, ttgo);
    espDelay(1000);

    button_init();
    pinMode(GATE0, OUTPUT);
    pinMode(GATE1, OUTPUT);
    pinMode(GATE2, OUTPUT);
    pinMode(GATE3, OUTPUT);
    pinMode(GATE4, OUTPUT);
    pinMode(GATE5, OUTPUT);

    analogWrite(GATE0,0);
    analogWrite(GATE1,0);
    analogWrite(GATE2,0);
    analogWrite(GATE3,0);
    analogWrite(GATE4,0);
    analogWrite(GATE5,0);

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }
}



void loop()
{
    button_loop();
}
