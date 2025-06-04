/*
 * Wemos S2 Mini V1.0 board code is ported from ESP32-gps-oled with
 * changes for using U1UART instead of UART2 serial port for NEO-6M
 * module interface. Pinouts for I2CEXT0_SCL_IN and I2CEXT0_SDA_IN
 * functions are different.
 * 
 * NEO-6M GPS module based 24 h clock with 128x32 oled display.
 * The 107-Arduino-NMEA library is used to parse the incoming NMEA
 * messages to extract hh:mm:ss GMT time from GSS sentence. Correction
 * to local time is made and daylight savings time correction is
 * selected by adding an external jumper.
 * 
 * oled display library u8g2 provides oled driver functions.
 * Time is printed on serial monitor as well.
 *
 * UART0 is used as terminal I/O, set at 115200 baud
 * UART2 is used as GPS module serial interface, set at 9600 baud
 * Hardware:
 *   - ESP32 S2 mini board
 *   - NEO-6M Module
 *
 * Electrical Connection:
 *   - GPS Module VIN <-> 3.3 V
 *   - GPS Module GND <-> GND
 *   - GPS Module TXO <-> (GPIO_18) RxD1, UART serial port 1
 * 
  
  For 0.91" oled display (SSD1306), use u8g2 Olimex library
  and use the applicable constructor. The display uses I2C
  interface, GPIO_33 <-> SDA, GPIO_35 <-> SCL

  Use GPIO_5 as a pseudo ground pin for DST_IN jumper as
  S2 mini board has limited GND connections.
  
 */

 // MIT License


#include <Arduino.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <ArduinoNmeaParser.h>
#include <RTClib.h>       // Required for I2C stuff

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
  This is a page buffer example.    
*/

// Constructor:
//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);



#define DST_IN 4          // GPIO_4 is DST enable input pin
#define P_GND 5           // GPIO_5 is a psuedo ground pin

/*
Function delcarations
*/

void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

/*
Global variables
*/
const int TimeZone = -5;      // Eastern time zone offset
int localtime_hours;          // Local time variable
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);

void setup()
{
  u8g2.begin();  
  Serial.begin(115200);     // Terminal interface
  Serial1.begin(9600);      // NEO-6M serial interface
  pinMode(DST_IN, INPUT_PULLUP);
  pinMode(P_GND, OUTPUT);
  digitalWrite(P_GND, 0);   // Pull pin to GND
  u8g2.firstPage();   // Sign-on message
    do {
      u8g2.setFont(u8g2_font_fub14_tr);   //Free Universal font
      u8g2.drawStr(16,24, "GPS Clock");
    } while (u8g2.nextPage());
  delay(2000);
}

void loop()
{
  while (Serial1.available()) {
  parser.encode((char)Serial1.read());
//  Serial.write(Serial1.read());   // Uncomment for debug only
  }
}

/*
Function definitions
*/

void onRmcUpdate(nmea::RmcData const rmc)
{
  localtime_hours = rmc.time_utc.hour;
  localtime_hours += TimeZone;
    if(localtime_hours < 0)
      localtime_hours += 24;
    if(!digitalRead(DST_IN))
      localtime_hours++;     // Apply daylight savings

  Serial.print("  ");
  Serial.print(localtime_hours);
  Serial.print(":");
  Serial.print(rmc.time_utc.minute);
  Serial.print(":");
  Serial.print(rmc.time_utc.second);
  Serial.println();


// Print time to oled display using u8g2 library functions
  
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_fub20_tn);   //Free Universal font, Good size for 6 digit display
    u8g2.drawStr(4,24, u8g2_u16toa((localtime_hours), 2)); // u8x8 function convert unit16 to ascii
    u8g2.drawStr(36,24,":"); // u8x8 function convert unit16 to ascii
    u8g2.drawStr(50,24, u8g2_u16toa((rmc.time_utc.minute), 2)); // u8x8 function convert unit16 to ascii
    u8g2.drawStr(83,24,":"); // u8x8 function convert unit16 to ascii
    u8g2.drawStr(96,24, u8g2_u16toa((rmc.time_utc.second), 2)); // u8x8 function convert unit16 to ascii
    } while (u8g2.nextPage());
}

void onGgaUpdate(nmea::GgaData const gga)
{
}

