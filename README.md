# aprs_tracker
Standalone Arduino APRS tracker based on modified version of LibAPRS

Arduino APRS Tracker
====================

Introduction
------------
Arudino APRS tracker is based on Arduino UNO with next periherals: 

 * Nokia screen, PCD8544
 * Rotary controller
 * Micromodem from marqvist - http://unsigned.io/micromodem/
 * GPS receiver, SkyLabs

Requirements
------------
 * Minimum: Arduino UNO with 2k SRAM, 32k FLASH (requires libraries manual optimization)
 * Recommended: Aruduino MEGA with 8k SRAM

Dependencies
------------

 * Simple timer - https://github.com/jfturcot/SimpleTimer
 * Rotary (rotary_button branch) - https://github.com/sh123/Rotary 
 * Modifed LibAPRS (aprs_tracker branch, check device.h for ports usage) - https://github.com/sh123/LibAPRS/tree/aprs_tracker
 * Tiny GPS - https://github.com/mikalhart/TinyGPS
 * Adafruit PCD8544 - https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library
 * Adafruit GFX - https://github.com/adafruit/Adafruit-GFX-Library
