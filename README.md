# Arduino Brew Timer for LaMarzocco Linea Mini espresso machine

### Board: Arduino Pro Mini / Nano

### Extension modules and peripherials used:
- SSD1306 OLED display, I2C
- MAX7219 LED display, I2C - optional, please see source code for feature switxh (MAX7219_ENABLED)
- 5x7 dot matrix displaying 3x5 digits (controlled by MAX7219)
- reed switch, sensing magnet valve sc

### Libraries used:
- SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
- LedControl by wayoda - Copyright (c) 2015, Eberhard Fahle

### Arduino Pro Mini / Nano pinout connections

I2C bus:
- SCK - pin A5
- SDA - pin A4

Toolchain: VSCode + Platform.IO

Copyright (c) 2019, Robert Klajko
