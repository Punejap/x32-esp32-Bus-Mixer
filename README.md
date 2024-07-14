This project uses FreeRTOS, ESP-IDF, and an ESP32 board to create a stereo channel mixing control for the Behringer x32 digital mixer. The design intention is to allow a musician to quickly edit their in-ear monitoring mix via physical control device. The build requires an ESP32 devkit, 4 rotary encoders, 4 LCD screens, and 4 WS2812b LED rings.

The original design was based off an older ESP32 WROOM D, and is currently being updated for use with an ESP32 WROOM s3. This repository will soon include the schematic and a PCB design done via KiCad.
GPIO assignments can be modified as needed in the main program file.
