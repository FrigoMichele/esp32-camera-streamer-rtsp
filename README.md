# ESP32-P4 MIPI CSI-2 capture and H.264 encoding

Use ESP-IDF **v5.5.2**, target `esp32p4`, with PSRAM enabled. `main/main.c` uses the `test/rtsp-streamer` wiring for a Raspberry Pi Camera Rev 1.3 (OV5647): SDA GPIO 7, SCL GPIO 8, and CSI PHY LDO channel 3 at 2.5 V. Verify your board and ribbon adapter match before connecting the camera.

`MIPI CSI → ISP → camera ready queue → encoder task → H.264 output queue → app_main → RTSP`

The encoder task in `main/main.c` configures the sensor for 800×640 RAW8, starts CSI and the ISP, opens the hardware encoder, then enables streaming. CSI callbacks hand off descriptors for three PSRAM DMA buffers. `main/h264_encoder.c` passes the original DMA pointer to the blocking hardware encode call; there is no raw copy, conversion, or encoding in an ISR. The task returns the input buffer to CSI only after completion (on codec error it closes the encoder before returning buffers and stopping).

Two separate 64-byte-aligned PSRAM slots hold encoded output. Each has 768,000 bytes of capacity; only `h264_frame_t.size` bytes contain the encoded frame. `app_main()` logs frame timing and offers each encoded frame to RTSP without waiting. If RTSP does not accept the frame, main releases it immediately. If no encoder output slot is free, the encoder waits **before** acquiring another camera buffer. CSI continues capturing and uses its private discard buffer when the camera pool is exhausted.

## Viewing the RTSP stream

Set the static IPv4 address, subnet mask, and gateway in `idf.py menuconfig` → **RTSP Ethernet settings**. DHCP is disabled. The defaults are `192.168.1.123`, `255.255.255.0`, and `0.0.0.0` (no gateway). Choose an unused address on your LAN, or connect directly and set your computer to a different address on the same subnet.

Connect the Function EV Board's onboard RJ45 port. Once the link is up, the `rtsp_connection` task logs the static IP and starts a video-only server using `espressif/esp_media_protocols` 1.0.3. Use the URL printed in the serial log, for example:

```sh
ffplay -rtsp_transport udp rtsp://192.168.1.123:8554/live
```

**RTSP Ethernet settings → Recommended RTP transport** defaults to UDP and controls the `ffplay` command printed at startup. The server does not force the transport; the client selects it in RTSP `SETUP`.

The server is unauthenticated; use it only on a trusted LAN. The RTSP handoff holds at most one queued encoded frame and never waits in the library callback. New sessions and interrupted streams resume at an IDR. RTSP copies encoded bytes into the library's staging buffer; raw camera frames remain zero-copy.

## Why this zero-copy input is compatible

- IDF v5.5.2 `hal/isp_types.h` and the P4 `hal/isp_ll.h` select ISP output type 3 for `ISP_COLOR_YUV420`. The [ESP32-P4 TRM](https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf), table **ISP Output Pixel Layout Format**, gives the byte order: `U Y Y` groups on odd rows, `V Y Y` on even rows. `hal/mipi_csi_hal.c` passes `byte_swap_en` to the bridge; this app leaves it disabled. The CSI driver (`esp_cam_ctlr_csi.c`) sizes the transfer from the output bit depth and invalidates the completed DMA buffer before the callback.
- `espressif/esp_h264` **1.4.1** `interface/include/esp_h264_types.h` defines exactly this packed order as `ESP_H264_RAW_FMT_O_UYY_E_VYY`. It is **not** NV12 or I420. `hw/src/esp_h264_enc_hw_param.c` places the input address directly in the YUV DMA descriptor; `hw/src/esp_h264_enc_single_hw.c` waits for the frame-done interrupt and performs cache maintenance.
- The 800×640 frame needs no 16×16 macroblock padding. Its 768,000 bytes and 64-byte-aligned PSRAM address meet the encoder's length/alignment requirements. The encoder rejects unverified layouts, size mismatches, or unsuitable alignment; it does not repack them.

To check on a board, run `idf.py flash monitor`. Look for H.264 frames with nonzero, variable byte counts and capture intervals. For an end-to-end image check, save the `size` valid bytes of consecutive output frames starting at an IDR, then decode the resulting H.264 stream and verify geometry and colors against a known scene. DMA completion alone does not verify image integrity.
