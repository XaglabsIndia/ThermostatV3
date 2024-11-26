
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
#include "logo.h"
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
char *Inactive_msg = "This is an example paragraph that will be automatically wrapped and aligned. It is important to ensure that text fits within the display width and that each line is properly spaced. We need to make sure that the line spacing is consistent and the text is readable.";
int logo_R = 1; // Right side logo
int logo_L = 2; // Left side logo
char Nudge_line1[MAX_LINE_LENGTH] = {0};
char Nudge_line2[MAX_LINE_LENGTH] = {0};
char Nudge_line3[MAX_LINE_LENGTH] = {0};
int number_of_nudge_lines = 0;
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
void inactive_screen(void);
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
  InitTempButton();
}
void inactive_screen_call(void){
inactive_screen();
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
back_text_t process_back_message(void) {
    back_text_t result = {{{0}}, 0};
    char back_msg[BACK_MAX_MSG_LENGTH] = {0};
    
    // Read message from NVS
    esp_err_t ret = read_string_from_nvs("back_msg", back_msg, sizeof(back_msg));
    if (ret != ESP_OK || back_msg[0] == '\0') {
        ESP_LOGI("MAinDisplay", "No back message found or empty");
        strcpy(result.lines[0], "Welcome Back");
        result.num_lines = 1;
        return result;
    }

    ESP_LOGI("MAinDisplay", "Original back message: '%s', length: %d", back_msg, strlen(back_msg));
    
    char *start = back_msg;
    int current_line = 0;
    
    while (*start && current_line < BACK_MAX_LINES) {
        // Skip leading spaces
        while (*start == ' ') start++;
        if (!*start) break;

        char *end = start;
        char *last_space = NULL;
        int chars_count = 0;
        
        // Find word boundary within line limit
        while (*end && chars_count < BACK_CHARS_PER_LINE) {
            if (*end == ' ') {
                last_space = end;
            }
            end++;
            chars_count++;
        }
        
        // Determine cut point
        char *cut_point;
        if (chars_count < BACK_CHARS_PER_LINE) {
            // Remaining text fits in line
            cut_point = end;
        } else if (last_space && last_space > start) {
            // Cut at word boundary
            cut_point = last_space;
        } else {
            // Force cut at line length
            cut_point = start + BACK_CHARS_PER_LINE;
        }
        
        // Copy line
        int line_length = cut_point - start;
        if (line_length > 0) {
            strncpy(result.lines[current_line], start, line_length);
            result.lines[current_line][line_length] = '\0';
            
            // Trim trailing spaces
            while (line_length > 0 && result.lines[current_line][line_length-1] == ' ') {
                result.lines[current_line][--line_length] = '\0';
            }
            
            ESP_LOGI("MAinDisplay", "Line %d: '%s' (length: %d)", 
                    current_line + 1, 
                    result.lines[current_line], 
                    strlen(result.lines[current_line]));
            
            current_line++;
        }
        
        // Move to next line start
        start = cut_point;
        while (*start == ' ') start++;
    }
    
    result.num_lines = current_line;
    ESP_LOGI("MAinDisplay", "Total back message lines: %d", result.num_lines);
    
    return result;
}

void update_back_display_message(void) {
    back_text_t wrapped_msg = process_back_message();
    
    // Update RTC variable
    number_of_inactive_msg_lines = wrapped_msg.num_lines;
    
    // Clear all lines first
    memset(Back_line1, 0, BACK_LINE_LENGTH);
    memset(Back_line2, 0, BACK_LINE_LENGTH);
    memset(Back_line3, 0, BACK_LINE_LENGTH);
    memset(Back_line4, 0, BACK_LINE_LENGTH);
    memset(Back_line5, 0, BACK_LINE_LENGTH);
    
    // Copy new lines
    if (wrapped_msg.num_lines >= 1) {
        strncpy(Back_line1, wrapped_msg.lines[0], BACK_LINE_LENGTH - 1);
    }
    if (wrapped_msg.num_lines >= 2) {
        strncpy(Back_line2, wrapped_msg.lines[1], BACK_LINE_LENGTH - 1);
    }
    if (wrapped_msg.num_lines >= 3) {
        strncpy(Back_line3, wrapped_msg.lines[2], BACK_LINE_LENGTH - 1);
    }
    if (wrapped_msg.num_lines >= 4) {
        strncpy(Back_line4, wrapped_msg.lines[3], BACK_LINE_LENGTH - 1);
    }
    if (wrapped_msg.num_lines >= 5) {
        strncpy(Back_line5, wrapped_msg.lines[4], BACK_LINE_LENGTH - 1);
    }
}
void inactive_screen(void) {
    // Update the message first
    update_back_display_message();

    display.setFullWindow();
    display.firstPage();
    do {
        display.setFont(&FreeSansBold24pt7b);
        
        switch(number_of_inactive_msg_lines) {
            case 5:
                drawWrappedText(String(Back_line1), 10, 150, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line2), 10, 210, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line3), 10, 270, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line4), 10, 330, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line5), 10, 390, display.width() - 30, "CENTER", 8);
                break;
                
            case 4:
                drawWrappedText(String(Back_line1), 10, 150, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line2), 10, 210, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line3), 10, 270, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line4), 10, 330, display.width() - 30, "CENTER", 8);
                break;
                
            case 3:
                drawWrappedText(String(Back_line1), 10, 170, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line2), 10, 230, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line3), 10, 290, display.width() - 30, "CENTER", 8);
                break;
                
            case 2:
                drawWrappedText(String(Back_line1), 10, 210, display.width() - 30, "CENTER", 8);
                drawWrappedText(String(Back_line2), 10, 270, display.width() - 30, "CENTER", 8);
                break;
                
            case 1:
            default:
                drawWrappedText(String(Back_line1), 10, 250, display.width() - 30, "CENTER", 8);
                break;
        }
        
    } while (display.nextPage());
    
    ESP_LOGI("Inactive", "Inactive screen displayed with %d lines", number_of_inactive_msg_lines);
}
wrapped_text_t process_front_message(void) {
    wrapped_text_t result = {{{0}}, 0};
    char front_msg[MAX_MSG_LENGTH] = {0};
    
    esp_err_t ret = read_string_from_nvs("front_msg", front_msg, sizeof(front_msg));
    if (ret != ESP_OK || front_msg[0] == '\0') {
        ESP_LOGI("MainDisplay", "No message found or empty message");
        strcpy(result.lines[0], "Default");
        result.num_lines = 1;
        return result;
    }

    char *msg_ptr = front_msg;
    int current_line = 0;
    int msg_len = strlen(front_msg);
    
    while (*msg_ptr && current_line < MAX_LINES) {
        // Skip leading spaces at start of new line
        while (*msg_ptr == ' ') {
            msg_ptr++;
        }
        
        if (!*msg_ptr) break;  // End if we've reached end of string

        // Find the potential cut point
        int chars_left = strlen(msg_ptr);
        int check_length = (chars_left > CHARS_PER_LINE) ? CHARS_PER_LINE : chars_left;
        
        // Look ahead to see if we're in the middle of a word at cut point
        int cut_point = check_length;
        bool in_word = false;
        
        if (check_length < chars_left && msg_ptr[check_length] != ' ' && msg_ptr[check_length] != '\0') {
            // We're in the middle of a word, look back for last space
            int last_space = -1;
            for (int i = 0; i < check_length; i++) {
                if (msg_ptr[i] == ' ') {
                    last_space = i;
                }
            }
            
            if (last_space != -1) {
                // Found a space, cut there
                cut_point = last_space;
            }
        }
        
        // Copy the line
        strncpy(result.lines[current_line], msg_ptr, cut_point);
        result.lines[current_line][cut_point] = '\0';
        
        // Trim trailing spaces
        int end = cut_point - 1;
        while (end >= 0 && result.lines[current_line][end] == ' ') {
            result.lines[current_line][end] = '\0';
            end--;
        }
        
        current_line++;
        msg_ptr += cut_point;
        
        // Skip spaces between words
        while (*msg_ptr == ' ') {
            msg_ptr++;
        }
    }
    
    result.num_lines = current_line;
    
    // Debug output
    ESP_LOGI("MainDisplay", "Original message: %s", front_msg);
    for (int i = 0; i < result.num_lines; i++) {
        ESP_LOGI("MainDisplay", "Line %d: '%s'", i + 1, result.lines[i]);
    }
    
    return result;
}

void update_display_message(void) {
    wrapped_text_t wrapped_msg = process_front_message();
    
    // Update global variables for display
    number_of_nudge_lines = wrapped_msg.num_lines;
    
    if (wrapped_msg.num_lines >= 1) {
        strncpy(Nudge_line1, wrapped_msg.lines[0], sizeof(Nudge_line1) - 1);
        Nudge_line1[sizeof(Nudge_line1) - 1] = '\0';
    }
    if (wrapped_msg.num_lines >= 2) {
        strncpy(Nudge_line2, wrapped_msg.lines[1], sizeof(Nudge_line2) - 1);
        Nudge_line2[sizeof(Nudge_line2) - 1] = '\0';
    }
    if (wrapped_msg.num_lines >= 3) {
        strncpy(Nudge_line3, wrapped_msg.lines[2], sizeof(Nudge_line3) - 1);
        Nudge_line3[sizeof(Nudge_line3) - 1] = '\0';
    }
}

int get_hour_from_timestr(const char* time_str) {
    int hour = 0;
    char meridian[3] = {0};
    
    if (strlen(time_str) >= 7) {  
        sscanf(time_str, "%d:%*d%2s", &hour, meridian);
        
        if (strcmp(meridian, "PM") == 0 && hour != 12) {
            hour += 12;
        } else if (strcmp(meridian, "AM") == 0 && hour == 12) {
            hour = 0;
        }
    }
    
    return hour;
}

bool is_night_time(const char* time_str) {
    int hour = get_hour_from_timestr(time_str);
    return (hour >= 18 || hour <= 3);
}

void main_screen_display()
 {

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
  update_display_message();
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
    
    ////////////////////////////// Set Temperature, Room temperature and Humidity value//////////////////////////////////////////////////
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
    dtostrf(set_temp, 4, 1, button_val);
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
    // display.setCursor(330, 40);
    // display.print(todays_time);
    display.drawBitmap(340, 5, gImage_logo, 121, 56, GxEPD_BLACK);

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
    display.setFont(&Meteocons_Regular_50);
    
    // Determine which logo to show based on time
    bool night_time = is_night_time(time);
    if (night_time) {
        // Night time (6 PM to 3 AM) - show logo_R
        display.setCursor(620, 55);
        display.println(logo_L);
    } else {
        // Day time (3 AM to 6 PM) - show logo_L
        display.setCursor(550, 55);
        display.println(logo_R);
    }
  if (number_of_nudge_lines > 0)  // Only proceed if we have messages to display
        {
            switch(number_of_nudge_lines) {
                case 1:
                    display.setFont(&FreeSans20pt7b);
                    drawWrappedText(Nudge_line1, 20, 425, display.width() - 50, "CENTER", 5);
                    break;
                    
                case 2:
                    display.setFont(&FreeSans16pt7b);
                    drawWrappedText(Nudge_line1, 20, 415, display.width() - 50, "CENTER", 5);
                    drawWrappedText(Nudge_line2, 20, 450, display.width() - 50, "CENTER", 5);
                    break;
                    
                case 3:
                    display.setFont(&FreeSans12pt7b);
                    drawWrappedText(Nudge_line1, 20, 400, display.width() - 50, "CENTER", 5);
                    drawWrappedText(Nudge_line2, 20, 430, display.width() - 50, "CENTER", 5);
                    drawWrappedText(Nudge_line3, 20, 460, display.width() - 50, "CENTER", 5);
                    break;
                    
                default:
                    // Fallback case - show just first line if something went wrong
                    display.setFont(&FreeSans20pt7b);
                    drawWrappedText(Nudge_line1, 10, 430, display.width() - 20, "CENTER", 5);
                    break;
            }
        }

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