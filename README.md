# ESP32-P4 H.264 hardware encoder

Minimal encoder-only adaptation of ESP-IDF's
[`examples/peripherals/h264`](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/h264).
The example generates test frames, encodes them with the ESP32-P4 hardware
encoder, and passes each encoded access unit to `handle_encoded_frame()`.

## Build

```sh
idf.py set-target esp32p4
idf.py build
```

The project is configured exclusively for ESP32-P4 hardware encoding and uses
PSRAM for the input and output buffers. Configure resolution, frame rate,
bitrate, GOP, and QP under **H.264 Hardware Encoder** in menuconfig.

Replace `fill_source_frame()` with camera input and send
`frame->raw_data.buffer` (`frame->length` bytes) from
`handle_encoded_frame()` to the RTSP transport.
