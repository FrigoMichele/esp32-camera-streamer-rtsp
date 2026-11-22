# ESP32-P4 MIPI CSI-2 capture

`app_main()` in `main/main.c` blocks on the ready queue and processes frames as they arrive. There is no separate camera task. The MIPI component uses CSI driver callbacks to hand off three PSRAM DMA buffers. If all buffers are busy, the driver captures into its private discard buffer. Use frame data before calling `mipi_camera_release_frame()`.

Use ESP-IDF **v5.5.2**, target `esp32p4`, with PSRAM enabled. `main/main.c` uses the `test/rtsp-streamer` wiring for a Raspberry Pi Camera Rev 1.3 (OV5647): SDA GPIO 7, SCL GPIO 8, and CSI PHY LDO channel 3 at 2.5 V. Verify your board and ribbon adapter match this wiring before connecting the camera. The app selects 800×640 RAW8, starts CSI and the ISP, then enables sensor streaming. The ISP converts RAW8 to YUV420 in hardware (768,000 bytes per 800×640 frame). The buffer uses the native ISP layout; do not assume I420 or NV12. The app logs a warning if no frame arrives within two seconds.

A DMA completion indicates that the driver finished the configured transfer; it does not verify sensor frame integrity.
