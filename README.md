# x32-esp32-Bus-Mixer
This project uses an esp32 to control a single stereo bus of a Behringer x32 digital mixer. The intention of the design is to allow a musician to quickly edit their in-ear monitoring mix via simple, physical control interface. The project requires an ESP32 dev-kit, 4 rotary controllers, 4 LCD screens, and 4 LED rings. It uses FreeRTOS and ESP-IDF.

The current build is centered around an older ESP32 WROOM 32D model. It is currently being updated to an ESP32 WROOM S3. The schematic and PCB design (done via KiCad) will be included in the near future. GPIO assignments are defined in the main program and can be changed as needed.   
