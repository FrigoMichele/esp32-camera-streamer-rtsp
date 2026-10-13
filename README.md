# ESP32-P4 OV5647 camera to H.264

Minimal camera capture based on ESP-IDF's
[`examples/peripherals/camera`](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/camera).
It is configured for the Raspberry Pi Camera v1.3, which uses the OV5647
sensor and a two-lane MIPI-CSI2 interface.

The application:

1. Detects and configures the OV5647 over I2C/SCCB.
2. Captures 800x640 RAW8 frames over MIPI-CSI2.
3. Uses the ESP32-P4 ISP to convert them to YUV420.
4. Encodes each frame with the ESP32-P4 hardware H.264 encoder.
5. Delivers encoded data to `handle_encoded_frame()`.

## Build

```sh
idf.py set-target esp32p4
idf.py build flash monitor
```

## RTSP server

The application starts an H.264 RTSP server on TCP port `8554`:

```text
rtsp://<device-ip>:8554/live
```

The server uses Espressif's `esp_media_protocols` component. Encoded frames
are copied from the H.264 encoder into the RTSP callback buffer. Use VLC or
`ffplay` to open the URL.

This project does not configure a network interface because ESP32-P4 boards
have board-specific Ethernet/PHY wiring and no integrated Wi-Fi. Initialize
Ethernet or an external Wi-Fi interface before `start_rtsp_server()` in
`main/main.c`.

## Hardware configuration

The default configuration follows the ESP-IDF camera example:

| Signal | Default GPIO |
|---|---:|
| Camera SDA | 7 |
| Camera SCL | 8 |
| MIPI PHY LDO | channel 3, 2500 mV |

MIPI CSI clock/data pins are fixed by the ESP32-P4 board's CSI connector and
are not GPIO-configurable in this application. Verify the pinout and the LDO
routing for your specific ESP32-P4 board before connecting the camera.

The Raspberry Pi camera is connected to the ESP32-P4 board's MIPI camera
port. The port provides the CSI clock/data lanes; only the SCCB/I2C pins and
MIPI PHY supply are configured by this application. Verify the board's camera
port pinout and voltage levels before connecting the module.

Change GPIOs, resolution, bitrate, and other options with:

```sh
idf.py menuconfig
```

The selected sensor mode is fixed at
`MIPI_2lane_24Minput_RAW8_800x640_50fps`; keep the configured resolution at
800x640 unless you also change the sensor format string in `main/main.c`.

Set **Number of frames to capture** to zero for continuous capture.
`handle_encoded_frame()` publishes each encoded access unit to the RTSP
server; the data is in `frame->raw_data.buffer` and has `frame->length` bytes.
