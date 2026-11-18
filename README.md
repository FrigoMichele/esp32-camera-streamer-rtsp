# ESP32-P4 OV5647 camera to H.264

Minimal camera capture based on ESP-IDF's
[`examples/peripherals/camera`](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/camera).
It is configured for the Raspberry Pi Camera v1.3, which uses the OV5647
sensor and a two-lane MIPI-CSI2 interface.

The application:

1. Detects and configures the OV5647 over I2C/SCCB.
2. Captures 1920x1080 RAW10 frames at 30 FPS over MIPI-CSI2.
3. Uses the ESP32-P4 ISP to convert them to YUV420.
4. Encodes each frame with the ESP32-P4 hardware H.264 encoder.
5. Publishes encoded H.264 access units to the RTSP module.

## Build

```sh
idf.py set-target esp32p4
idf.py build flash monitor
```

## RTSP server

The application starts an H.264 RTSP server on TCP port `8554`:

```text
rtsp://192.168.2.5:8554/live
```

The server uses Espressif's `esp_media_protocols` component. Encoded frames
are copied from the H.264 encoder into the RTSP callback buffer. Use VLC or
`ffplay` to open the URL.

The application passes the endpoint and static Ethernet settings through
`rtsp_config.network`; the RTSP module then configures the internal EMAC and
IP101 PHY on the ESP32-P4 Function EV Board:

| Setting | Value |
|---|---|
| Address | `192.168.2.5` |
| Netmask | `255.255.255.0` |
| Gateway | `192.168.2.1` |
| PHY address | `1` |
| PHY reset | GPIO `51` |
| RMII clock | GPIO `50` (external PHY clock) |

The board's Ethernet cable and playback computer must be on the
`192.168.2.0/24` network. ESP32-P4 has no integrated Wi-Fi; use a separate
Wi-Fi interface or adjust the board-specific Ethernet initialization if your
hardware is not the Function EV Board.

## Low-latency playback and encoding

For video-only live preview, use video-master sync **and untimed presentation**
so FFplay can catch up after a display/window stall:

```sh
ffplay -rtsp_transport udp -fflags nobuffer -flags low_delay -framedrop -sync video -vf "setpts=N" -max_delay 0 -reorder_queue_size 0 -analyzeduration 0 -probesize 32768 rtsp://192.168.2.5:8554/live
```

`setpts=N` assigns successive decoded pictures one timebase tick apart, rather
than replaying a backlog at the source's presentation cadence. The live input
normally determines display cadence; after a stall the receiver can drain its
queue rapidly. This deliberately changes playback timestamps: use it only for
video-only live viewing, not A/V synchronization or timestamp-accurate playback.
Without this filter, `-sync video` reduces initial buffering but can preserve
newly accumulated delay after a stall. Firmware/RTP timestamps are unchanged.

Firmware defaults (configurable in `idf.py menuconfig`):

- Encoder target **10 FPS**, GOP **10**, bitrate **1,000,000 bit/s**. The sensor
  remains in its 30-FPS mode; actual encoded throughput is reported in the log.
- Rate-control QP range **26..51**, rather than fixed QP. Minimum QP must be
  strictly less than maximum QP; high QP reduces image quality to help meet the
  bitrate target. This is rate control, not a strict bandwidth cap.
- `CONFIG_RTSP_SEND_POLL_HZ=200`: poll for ready frames every **5 ms**, independent
  of the encoder FPS. This maps to the RTSP library's maximum-FPS timer field;
  it does not advertise a 200-FPS H.264 stream or change SPS/PTS. Empty polls
  return immediately and send nothing. Busy timer/network work can add delay.

Once-per-second logs report pipeline timing, measured encoded kbit/s, largest
access unit (`max_AU`), and frame age at the RTSP callback during playback.
The `trace:` line breaks down the slowest handed-off frame in each interval,
keyed by its original `pts_ms`; `cb_us` is **before** RTP send, not display time.
For a persistent ~700 ms, [collect the matching FFplay log and optical measurement](docs/latency-analysis.md#isolating-a-steady-700-ms)
before tuning more flags. `tools/ffplay_timing.py` summarizes the receiver log offline.

## Hardware configuration

The camera defaults follow the ESP-IDF camera example; Ethernet uses the Function EV Board pinout:

| Signal | Default GPIO |
|---|---:|
| Camera SDA | 7 |
| Camera SCL | 8 |
| MIPI PHY LDO | channel 3, 2500 mV |
| Ethernet MDC | 31 |
| Ethernet MDIO | 52 |
| RMII TXEN | 49 |
| RMII TXD0/TXD1 | 34/35 |
| RMII RXDV | 28 |
| RMII RXD0/RXD1 | 29/30 |

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

The application currently passes
`MIPI_2lane_24Minput_RAW10_1920x1080_30fps` as
`camera_config.format_name` in `main/main.c`. Keep the configured resolution
matched to the selected sensor format.

Set **Number of frames to capture** to zero for continuous capture.

## Module layout

- `camera.h/.c`: OV5647, unsynchronized MIPI-CSI capture and all ISP
  correction (demosaic, denoising, gamma, color correction matrix and color
  controls).
- `encoder.h/.c`: opaque hardware H.264 encoder and reusable output buffer.
- `rtsp.h/.c`: Ethernet initialization, static IP configuration and RTSP
  server. `rtsp_publish()` copies each encoded frame before returning.
- `main.c`: connects the three minimal module interfaces.

Each module exposes an opaque handle and returns `esp_err_t` for operations;
implementation-specific ESP-IDF handles remain private to its `.c` file.

## TODO

- Reintroduce and test camera completion, buffer ownership, RTSP hand-off and
  Ethernet readiness separately, step by step.
- Measure actual frame cadence, PSRAM headroom and reconnect/slow-client behavior.
- Verify the ISP transform and, more generally, the camera processing and
  calibration for this OV5647 camera.
