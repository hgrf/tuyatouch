#pragma once

#ifdef BOARD_WAVESHARE_1_69

#define LCD_DC 4
#define LCD_CS 5
#define LCD_SCK 6
#define LCD_MOSI 7
#define LCD_RST 8
#define LCD_BL 15
#define LCD_WIDTH 240
#define LCD_HEIGHT 280

#define IIC_SDA 11
#define IIC_SCL 10

#define TP_RST 13
#define TP_INT 14

#define BUZZER 33
#define SYS_EN 35
#define VOLTAGE_DIVIDER 1
#define VREF 3.3                 // Power supply voltage of ESP32-S3 (unit: volts)
#define R1 200000.0              // Resistance value of the first resistor (unit: ohms)
#define R2 100000.0              // Resistance value of the second resistor (unit: ohms)

#endif

#ifdef BOARD_WAVESHARE_4_3

#define SD_MOSI 11
#define SD_CLK  12
#define SD_MISO 13
#define SD_SS -1

#define SD_CS 4

#endif