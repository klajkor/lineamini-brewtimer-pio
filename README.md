# Arduino Brew Timer for LaMarzocco Linea Mini espresso machine

### Board: Arduino Pro Mini / Nano

### Extension modules and peripherials used:
- SSD1306 OLED display, I2C - optional, please see source code for feature switch (SSD1306_ENABLED)
- MAX7219 LED matrix display, I2C - optional, please see source code for feature switch (MAX7219_ENABLED)
- 5x7 dot matrix displaying 3x5 digits (controlled by MAX7219)
- Holtek HT16K33 LED matrix display, I2C - optional, please see source code for feature switch (HT16K33_ENABLED)
- ILI9340 TFT display, SPI and 8 bit data - optional, please see source code for feature switch (ILI9340_ENABLED)
- reed switch, sensing magnet valve sc
- INA219 voltage and current sensor, I2C

### Libraries used:
- SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
- LedControl by wayoda - Copyright (c) 2015, Eberhard Fahle
- Adafruit INA219 by Adafruit - Copyright (c) 2012, Adafruit Industries
- HT16K33 - Copyright (c) 2017, lpaseen, Peter Sjoberg <peters-alib AT techwiz.ca>
- MCUFRIEND_kbv - Copyright (c) 2020, David Prentice

### Arduino Pro Mini / Nano pinout connections

I2C bus:
- SCK - pin A5
- SDA - pin A4

Toolchain: VSCode + Platform.IO

Copyright (c) 2020, Robert Klajko
