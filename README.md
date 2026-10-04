# E-Ink Picture Frame

A battery-powered **color e-paper picture frame** that rotates itself between portrait and landscape. Pictures are uploaded and managed in a web interface hosted by the frame itself.

## Features

- **7.3" Spectra 6 e-paper** (GDEP073E01, 800×480, 6 colors). It keeps the image without using power.
- **Motorized rotation:** a 28BYJ-48 stepper with a 40:1 worm gear turns the frame by 90°. An IMU (MPU-9250) detects the orientation and fine-tunes the position.
- **Built-in web app:** the frame opens its own WiFi access point (`http://192.168.4.1`) where you can:
  - upload and delete pictures (thumbnails are generated automatically)
  - create playlists and pick the current picture
  - set the clock, change settings, calibrate the motor
- **Deep sleep:** the DS3231 RTC wakes the frame to change the picture. A button wakes it up to start the WiFi interface.
- Battery monitoring with a low-battery indicator on the picture, plus an RGB status LED.
- Images, playlists and settings are stored on a microSD card.

## Hardware

| Part | Details |
|---|---|
| MCU | ESP32-S3 (with PSRAM) |
| Display | GDEP073E01 via DESPI-C02 adapter (SPI) |
| Storage | microSD (shares the SPI bus) |
| RTC / IMU | DS3231 + MPU-9250 (I2C) |
| Motor | 28BYJ-48 + ULN2003, worm gear 40:1 |
| Power | Li-ion battery |
| PCB | Custom KiCad board (`Ki-Cad/EInk-Rahmen`) |

All pins are defined in `config.h`.

## Repository structure

```
Arduino/
  eink_frame_V11/       Latest firmware (config.h, display/motor modules, web UI)
  eink_frame_V2..V10/   Earlier versions / milestones
  eink_frame_V10_WOmotor/  Variant without rotation motor
  Test/                 Hardware tests (display, SD, I2C scanner, LED, stepper)
3D Files/Frame.f3z      Fusion 360 frame model
Ki-Cad/EInk-Rahmen/     Schematic and PCB
```

## Getting started

1. Install these libraries: **GxEPD2**, **ESPAsyncWebServer** + **AsyncTCP** (mathieucarbou), **RTClib**, **AccelStepper**.
2. Board: *ESP32S3 Dev Module* with PSRAM enabled.
3. Copy `eink_frame.html` to the SD card as `/index.html` and create an `/images` folder.
4. Adjust the WiFi name/password in `config.h`, then upload `eink_frame_V11.ino`.
5. Press the WiFi button, connect to the frame's network and open `http://192.168.4.1`.
