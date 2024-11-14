
/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "Arduino.h"
#include "Adafruit_GFX.h"
#include "esp_rom_gpio.h"
#include "GxEPD2_BW.h"
#include <SPI.h>

extern "C"
{
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "Config.h"
#include <math.h>
}

// Fonts
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans16pt7b.h>
#include <Fonts/FreeSans20pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSans30pt7b.h>
#include <Fonts/FreeSans35pt7b.h>
#include <Fonts/FreeSans40pt7b.h>
#include <Fonts/FreeSans45pt7b.h>
#include <Fonts/FreeSans50pt7b.h>
#include <Fonts/Meteocons_Regular_50.h>
///////

int x_batt = 720;
int y_batt = 30;

#define EPD_Switch GPIO_NUM_7
#define CS GPIO_NUM_1
#define DC GPIO_NUM_17
#define RST GPIO_NUM_38
#define BUSY GPIO_NUM_48


GxEPD2_BW<GxEPD2_750_GDEY075T7, GxEPD2_750_GDEY075T7::HEIGHT> display(GxEPD2_750_GDEY075T7(CS, DC, RST, BUSY)); // GDEW075T7 800x480

int humidity = 0;
float temperatureF = 0.0;
volatile bool intFlag = false;
int first_boot = 1;

// Thermostat data
int battery_percent = 100;
RTC_DATA_ATTR static float count_value = 24.5;
char *date = "05/11/2024";
char *todays_time = "10:10 AM";
char *Default = "Nature-inspired display loading...";
char *Nudge_line1 = "Help us save the planet!";
char *Nudge_line2 = "by turning OFF the AC when not in use";
char *Nudge_line3 = "1234567890123456789012345678901234567890123456789012345678";

char *Inactive_msg = "This is an example paragraph that will be automatically wrapped and aligned. It is important to ensure that text fits within the display width and that each line is properly spaced. We need to make sure that the line spacing is consistent and the text is readable.";
int logo_R = 8; // Right side logo
int logo_L = 9; // Left side logo

void display_initialisations(void);
void main_screen_display(void);
void setTemperature_partial_update(void);
void battery_percentage_display(int battery_percent);
void roomTemperature_partial_update(float sensor_temperature_value);
void humidity_partial_update(int humidity);
void date_partial_update(char *date);
void time_partial_update(char *time);
void logo1_partial_update(int logo_R);
void logo2_partial_update(int logo_L);
void nudge_partial_update(int number_of_lines, char *position);
void inactive_screen(const String &Inactive_message, char *position);
void drawWrappedText(const String &text, int x, int y, int maxWidth, char *alignment, int lineSpacing);
void inactive_screen_call(void);
void Inactive_message_Init(void);
extern "C"
{
  void app_main(void);
  uint8_t HDC1080_humid_data();
  uint8_t HDC1080_temp_data();
}

void Inactive_message_Init(void){

  pinMode(CS, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(BUSY, OUTPUT);
  pinMode(EPD_Switch, OUTPUT);
   humidity = 20;
  temperatureF = 30.0;
  display_initialisations();
}

void Display_Main(void)
{
  InitMain();
  pinMode(CS, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(BUSY, OUTPUT);
  pinMode(EPD_Switch, OUTPUT);
  display_initialisations();
  main_screen_display();
  logo1_partial_update(logo_R);
  // battery_percentage_display(battery_percent);
  logo2_partial_update(logo_L);
  // nudge_partial_update(2, "CENTER");
  InitTempButton();
}
void inactive_screen_call(void){
char back_msg[256];
read_string_from_nvs("back_msg", back_msg, sizeof(back_msg));
inactive_screen(back_msg, "CENTER");

}
void display_initialisations()
{
  digitalWrite(EPD_Switch, 1);
  display.init();
  display.setRotation(0);
  display.setFullWindow();
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
}

void humidity_partial_update(int humidity)
{
  display.setTextSize(1);
  display.setFont(&FreeSans40pt7b);
  display.setPartialWindow(640, 210, 90, 80);
  display.setCursor(648, 280);
  String humidity_val = String(humidity);
  display.firstPage();
  do
  {
    display.print(humidity_val);
  } while (display.nextPage());
}

void roomTemperature_partial_update(float sensor_temperature_value)
{
  display.setFont(&FreeSans40pt7b);
  // display.setTextSize(2);
  display.setPartialWindow(20, 210, 150, 95);
  display.setCursor(25, 280);
  String sensor_val = String(sensor_temperature_value);
  display.firstPage();
  do
  {
    display.print(sensor_val);
  } while (display.nextPage());
  display.setTextSize(1);
}
void setTemperature_partial_update()
{
  display.setFont(&FreeSans45pt7b);
  display.setTextSize(2);
  display.setPartialWindow(227, 170, 343, 150);
  display.setCursor(235, 295);
  printf("set temp in cpp is %f",set_temperature);
  String button_val = String(set_temperature);
  display.firstPage();
  do
  {
    display.print(button_val);
  } while (display.nextPage());
  display.setTextSize(1);
}

void date_partial_update(char *date)
{
  display.setFont(&FreeSans16pt7b);
  display.setPartialWindow(20, 10, 230, 50);
  display.setCursor(50, 40);
  display.firstPage();
  do
  {
    display.print(date);
  } while (display.nextPage());
}

void time_partial_update(char *time)
{
  display.setFont(&FreeSans16pt7b);
  display.setPartialWindow(280, 10, 210, 50);
  display.setCursor(330, 40);
  display.firstPage();
  do
  {
    display.print(time);
  } while (display.nextPage());
}

void logo1_partial_update(int logo)
{
  display.setFont(&Meteocons_Regular_50); // Set font
  display.setPartialWindow(610, 10, 60, 50);
  display.setCursor(620, 55); // Set the position to start printing text (x,y)
  display.firstPage();
  do
  {
    display.println(logo);
  } while (display.nextPage());
}

void logo2_partial_update(int logo)
{
  display.setFont(&Meteocons_Regular_50); // Set font
  display.setPartialWindow(540, 10, 60, 50);
  display.setCursor(550, 55); // Set the position to start printing text (x,y)
  display.firstPage();
  do
  {
    display.println(logo);
  } while (display.nextPage());
}

void nudge_partial_update(int number_of_lines, char *position)
{
  display.setPartialWindow(5, 370, 800, 100);

  if (number_of_lines == 1)
  {
    display.setFont(&FreeSans20pt7b);
    display.firstPage();
    do
    {
      drawWrappedText(Nudge_line1, 20, 425, display.width() - 50, position, 5);
    } while (display.nextPage());
  }

  else if (number_of_lines == 2)
  {
    display.setFont(&FreeSans16pt7b);
    display.firstPage();
    do
    {
      drawWrappedText(Nudge_line1, 20, 415, display.width() - 50, position, 5);
      drawWrappedText(Nudge_line2, 20, 450, display.width() - 50, position, 5);
    } while (display.nextPage());
  }

  else if (number_of_lines == 3)
  {
    display.setFont(&FreeSans12pt7b);
    display.firstPage();
    do
    {
      drawWrappedText(Nudge_line1, 20, 400, display.width() - 50, position, 5);
      drawWrappedText(Nudge_line2, 20, 430, display.width() - 50, position, 5);
      drawWrappedText(Nudge_line3, 20, 460, display.width() - 50, position, 5);

    } while (display.nextPage());
  }

  else
  {
    display.setFont(&FreeSans20pt7b);
    display.firstPage();
    do
    {
      drawWrappedText(Nudge_line1, 10, 430, display.width() - 20, position, 5);
    } while (display.nextPage());
  }
}

void inactive_screen(const String &Inactive_message, char *position)
{
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.setFont(&FreeSans24pt7b);
    display.setCursor(20, 140);
    display.setTextWrap(1);
    display.print("Closing the window while the heater is ON can save 200kg of CO2 per year.");
    display.setTextWrap(0);
    display.setCursor(50, 360);
    display.print("Open the door to a greener future!");

    display.setFont(&FreeSansBold24pt7b);
    display.setCursor(140, 280);
    display.print("Be a climate champion.");
    // drawWrappedText(Inactive_message, 10, 40, display.width() - 20, position, 5);
  } while (display.nextPage());
}

void main_screen_display()
 {
//   humidity= HDC1080_humid_data();
//   float sensor_temperature_value = HDC1080_temp_data();
// Initialize variables with default values
float room_temp = 0.0f;
int humidityf = 0;
float set_temp = 0.0f;
char date[11] = "NA";  // Default to "NA" for date
char time[9] = "NA";   // Default to "NA" for time

// Try to read values, keeping defaults if read fails
if (read_float_from_nvs("room_temp_f", &room_temp) != ESP_OK) {
    room_temp = 0.0f;
}
if (read_int_from_nvs("humidity_f", &humidityf) != ESP_OK) {
    humidityf = 0.0f;
}
if (read_float_from_nvs("set_temp_f", &set_temp) != ESP_OK) {
    set_temp = 0.0f;
}
if (read_string_from_nvs("date_str", date, sizeof(date)) != ESP_OK) {
    strcpy(date, "NA");
}
if (read_string_from_nvs("time_str", time, sizeof(time)) != ESP_OK) {
    strcpy(time, "NA");
}
  // humidity= 10();
  // float sensor_temperature_value = 50;
 display.firstPage();
  do
  {
    // Borders
    display.fillRect(0, 356, 850, 1, GxEPD_BLACK);
    display.fillRect(0, 68, 850, 1, GxEPD_BLACK);
    display.fillRect(268, 0, 1, 68, GxEPD_BLACK);
    display.fillRect(532, 0, 1, 68, GxEPD_BLACK);

    ///////////////////////////////////////// Battery percentage sign/////////////////////////////////////////////
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(714, 33);
    display.print("%");
    // Battery symbol
    display.drawRect(x_batt + 12, y_batt - 12, 40, 20, GxEPD_BLACK);
    display.fillRect(x_batt + 52, y_batt - 7, 4, 11, GxEPD_BLACK);
    display.fillRect(x_batt + 14, y_batt - 10, 36 * battery_percent / 100.0, 16, GxEPD_BLACK);

    // If the battery percent is critically low, display this symbol.
    if (battery_percent <= 10)
    {
      display.setCursor(746, 33);
      display.print("X");
    }
    // If the battery percent is low, display this symbol.
    display.setFont(&FreeSansBold18pt7b);
    if (battery_percent > 10 && battery_percent <= 20)
    {
      display.setCursor(778, 39);
      display.print("!");
    }

    display.setFont(&FreeSansBold9pt7b);
    if (battery_percent == 100)
    {
      display.setCursor(680, 33);
    }
    else if (battery_percent < 100 && battery_percent > 0)
    {
      display.setCursor(690, 33);
    }
    else if (battery_percent == 0)
    {
      display.setCursor(700, 33);
    }
    // Display the battery percentage value
    display.print(battery_percent);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////Room temperature and Humidity value//////////////////////////////////////////////////
    display.setFont(&FreeSans40pt7b);
    display.setCursor(25, 280);
    String sensor_val = String(room_temp, 1);
    display.print(sensor_val);
    display.setCursor(648, 280);
    String humidity_val = String(humidityf);
    display.print(humidity_val);

    display.setFont(&FreeSans45pt7b);
    display.setTextSize(2);
    display.setCursor(235, 295);
    char button_val[10];
    dtostrf(count_value, 4, 1, button_val);
   
    display.print(button_val);
    display.setTextSize(1);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Degree symbol for Set and Room temperature
    display.setFont(&FreeSans12pt7b);
    display.setCursor(178, 230);
    display.print("o");

    // Words- "Room Temperature"
    display.setCursor(75, 180);
    display.print("Room");
    display.setCursor(35, 205);
    display.print("Temperature");
    // Word - "Humidity"
    display.setCursor(675, 180);
    display.print("Room");
    display.setCursor(663, 205);
    display.print("Humidity");

    display.setFont(&FreeSans16pt7b);
    display.setCursor(390, 120);
    display.print("Set");
    display.setCursor(330, 150);
    display.print("Temperature");
    display.setCursor(50, 40);
    display.print(date);
    display.setCursor(330, 40);
    display.print(time);
    display.setCursor(30, 410);
    drawWrappedText(Nudge_line1, 10, 400, display.width() - 20, "CENTER", 5);
    display.setCursor(575, 180);
    display.print("o");

    // Celsius symbol
    display.setFont(&FreeSans20pt7b);
    display.setCursor(188, 255);
    display.print("C");

    // Humidity percentage
    display.setFont(&FreeSans24pt7b);
    display.setCursor(738, 270);
    display.print("%");
    display.setCursor(592, 210);
    display.print("C");

    // Weather icons
    display.setFont(&Meteocons_Regular_50); // Set font
    display.setCursor(620, 55);             // Set the position to start printing text (x,y)
    display.println(logo_R);
    display.setCursor(550, 55); // Set the position to start printing text (x,y)
    display.println(logo_L);

  } while (display.nextPage());
}

void battery_percentage_display(int battery_percent)
{
  display.setPartialWindow(680, 18, 120, 30);
  do
  {
    // Battery percentage sign
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(714, 33);
    display.print("%");
    // Battery symbol
    display.drawRect(x_batt + 12, y_batt - 12, 40, 20, GxEPD_BLACK);
    display.fillRect(x_batt + 52, y_batt - 7, 4, 11, GxEPD_BLACK);
    display.fillRect(x_batt + 14, y_batt - 10, 36 * battery_percent / 100.0, 16, GxEPD_BLACK);

    // If the battery percent is critically low, display this symbol.
    if (battery_percent <= 10)
    {
      display.setCursor(746, 33);
      display.print("X");
    }
    // If the battery percent is low, display this symbol.
    display.setFont(&FreeSansBold18pt7b);
    if (battery_percent > 10 && battery_percent <= 20)
    {
      display.setCursor(778, 39);
      display.print("!");
    }

    display.setFont(&FreeSansBold9pt7b);
    // Cursor settings based on the battery percentage value
    if (battery_percent == 100)
    {
      display.setCursor(680, 33);
    }
    else if (battery_percent < 100 && battery_percent > 0)
    {
      display.setCursor(690, 33);
    }
    else if (battery_percent == 0)
    {
      display.setCursor(700, 33);
    }
    // Display the battery percentage value
    display.print(battery_percent);

  } while (display.nextPage());
}


void drawWrappedText(const String &text, int x, int y, int maxWidth, char *alignment, int lineSpacing)
{
  int16_t x1, y1;
  uint16_t textWidth, textHeight;
  int cursorX = x;
  int cursorY = y;
  String line = "";

  // Measure font height and width of an empty string
  display.getTextBounds("A", x, y, &x1, &y1, &textWidth, &textHeight);

  for (int i = 0; i < text.length(); ++i)
  {
    line += text[i];
    display.getTextBounds(line, x, y, &x1, &y1, &textWidth, &textHeight);

    // If the text width exceeds the maxWidth and the current character is a space
    if (textWidth > maxWidth && text[i] == ' ')
    {
      // Draw the current line
      if (strcmp(alignment, "LEFT") == 0)
      {
        display.setCursor(cursorX, cursorY);
      }
      else if (strcmp(alignment, "CENTER") == 0)
      {
        display.setCursor(x + (maxWidth - textWidth) / 2, cursorY);
      }
      else if (strcmp(alignment, "RIGHT") == 0)
      {
        display.setCursor(x + maxWidth - textWidth, cursorY);
      }
      else
      {
        printf("Wrong alignment selected\n");
      }
      display.print(line);

      // Move to the next line with consistent spacing
      cursorY += textHeight + lineSpacing;
      line = ""; // Reset line
    }
    else if (textWidth > maxWidth && text[i] != ' ')
    {
      // Handle case where a single word is too long to fit on a line
      int lastSpace = line.lastIndexOf(' ');
      if (lastSpace != -1)
      {
        String word = line.substring(0, lastSpace);
        line.remove(0, lastSpace + 1);

        // Draw the word and move to the next line
        if (strcmp(alignment, "LEFT") == 0)
        {
          display.setCursor(cursorX, cursorY);
        }
        else if (strcmp(alignment, "CENTER") == 0)
        {
          display.getTextBounds(word, x, y, &x1, &y1, &textWidth, &textHeight);
          display.setCursor(x + (maxWidth - textWidth) / 2, cursorY);
        }
        else if (strcmp(alignment, "RIGHT") == 0)
        {
          display.getTextBounds(word, x, y, &x1, &y1, &textWidth, &textHeight);
          display.setCursor(x + maxWidth - textWidth, cursorY);
        }
        else
        {
          printf("Wrong alignment selected\n");
        }

        display.print(word);
        cursorY += textHeight + lineSpacing;
      }
    }
  }

  // Draw the last line if there's any text left
  if (line.length() > 0)
  {
    if (strcmp(alignment, "LEFT") == 0)
    {
      display.setCursor(cursorX, cursorY);
    }
    else if (strcmp(alignment, "CENTER") == 0)
    {
      display.getTextBounds(line, x, y, &x1, &y1, &textWidth, &textHeight);
      display.setCursor(x + (maxWidth - textWidth) / 2, cursorY);
    }
    else if (strcmp(alignment, "RIGHT") == 0)
    {
      display.getTextBounds(line, x, y, &x1, &y1, &textWidth, &textHeight);
      display.setCursor(x + maxWidth - textWidth, cursorY);
    }
    else
    {
      printf("Wrong alignment selected\n");
    }

    display.print(line);
  }
}