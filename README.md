# 2DT304-life-buoy
heltec v2 board
https://escapequotes.net/esp32-lora-heltec-v2-with-display-pinout-diagram/



SETUP
Need usb drivers to flash
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads
device manager fix

Folder:
Components
Links to githubs to get in the components folder

senor lib:
https://github.com/sparkfun/SparkFun_Environmental_Combo_Breakout_ENS160_BME280_QWIIC

Libraries
--------------
* **[Arduino Lirary](https://github.com/espressif/arduino-esp32)** - Arduino Library
* **[ENS160 Library](https://github.com/sparkfun/SparkFun_Indoor_Air_Quality_Sensor-ENS160_Arduino_Library)** - Arduino library for the ENS160
* **[BME280 Library](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library)** - Arduino library for the BME280
* **[ttn-esp Library](https://github.com/manuelbl/ttn-esp32)** - The things network Library for esp32

in BME280 FOLDER
CMakeLists.txt


in ENS160 Folder
CMakeLists.txt




Lorawan > 
> Menu
> ESP-IDF: SDK Configuration editor (Menuconfig)

 change Component config the things network ttn LoRa frequency/ region to Europe (868 MHz)
