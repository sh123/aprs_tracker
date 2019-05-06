# aprs_tracker
Tiny single sketch Arduino APRS tracker based on modified version of LibAPRS.

Arduino APRS Tracker
====================
![alt text](http://i60.tinypic.com/23h2hd1.jpg)

Introduction
------------
Arudino APRS tracker is based on Arduino UNO with next periherals: 

 * Nokia screen, PCD8544
 * Rotary encoder
 * Micromodem from marqvist - http://unsigned.io/micromodem/
 * GPS receiver, SkyLabs

Requirements
------------
 * Minimum: Arduino UNO with 2k SRAM, 32k FLASH
   * requires some manual libraries optimization, such as buffer size decrease
   * or different screen, PCD8544 requires 500 bytes of RAM for the buffer
 * Recommended: Aruduino MEGA with 8k SRAM
   * needs some tweaks in LibAPRS internals

Dependencies
------------
 * Simple timer - https://github.com/jfturcot/SimpleTimer
 * Rotary (rotary_button branch) - https://github.com/sh123/Rotary/tree/rotary_button
 * Modifed LibAPRS (aprs_tracker branch, check device.h for ports usage) - https://github.com/sh123/LibAPRS/tree/aprs_tracker
 * Tiny GPS - https://github.com/mikalhart/TinyGPS
 * Adafruit PCD8544 - https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library
 * Adafruit GFX - https://github.com/adafruit/Adafruit-GFX-Library
