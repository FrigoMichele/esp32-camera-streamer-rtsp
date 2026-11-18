# Glass-to-glass latency investigation

## Isolating a steady 700 ms

At the measured 10 FPS, 700 ms spans **seven picture periods**. That does not
mean there is a seven-picture FIFO. A constant offset, delay that grows after a
stall, and waiting for the first IDR need different tests. The FFplay workaround
below addresses the second case; it does not establish the cause of the first.

### What is already accounted for?

- **Board, before the new trace.** In the saved serial run (board log time
  295–314 s), the medians of the bucket averages were 64.1 ms in `camera_frame`,
  35.5 ms in `encoder_encode`, and 0.13 ms in `rtsp_publish`. Request-to-callback
  bucket averages were 102.2–104.9 ms; the largest individual age in that window
  was 115.7 ms. One earlier join reached 291.8 ms. These are *request* ages:
  the sensor is free running, one raw buffer is requested at a time, the encoder
  returns the current AU synchronously, and RTSP has one replaceable slot.
- **Receiver, new direct RTSP/UDP tests.** Two 10-second sessions with the live
  camera, FFplay 9.0.2, dummy/software renderer, same low-delay flags, and
  `showinfo` **before** any `setpts`. Matched by exact original PTS; first ten
  matched pictures excluded in each run. Times are PC log intervals in ms:

| Interval (median / p95) | `-sync video` (83 pictures) | Also `setpts=N` (76 pictures) |
| --- | ---: | ---: |
| Last `ff_read_packet` fragment → `read_frame_internal` assembled AU | 99 / 105 | 99 / 101 |
| Assembled AU → `showinfo` decoded picture | 2 / 2 | 3 / 5 |

The trace repeatedly shows the **first fragment of picture N+1 releasing the
parsed packet for N**. That is about one source period *before decoding*, not
100 ms of decoder computation. Rewriting timestamps in a decoded-frame filter
cannot remove this wait. Conversely, `ff_read_packet` is a user-space read,
not NIC arrival; `showinfo` is **not** presentation. The serial and player
measurements were not the same optical run. They account for roughly two
periods in those tests, not the other ~500 ms in the reported observation.

### 1. Collect a matched sender and receiver trace

Build/flash the diagnostic firmware when ready (`idf.py -p PORT flash monitor`).
Save the serial output including SETUP/PLAY, and start **one** viewer. No FPS,
GOP, RC, packet, PTS, buffer policy or player default is changed by this logger.
Once per pipeline report, `callback:` gives the count, average/max request age,
and unread AUs `overwritten` since the last report. `n=0` with overwrites is
normal without a viewer; during PLAY, overwrites mean some **pictures never
reached the callback**, not that UDP packets were lost.

`trace:` describes **one actual handed-off picture**, the one with maximum
request age in that interval. `id` is its producer frame number, `pts_ms` its
unchanged encoder PTS, `bytes` its encoded size. All five durations below are
in microseconds on one ESP clock:

| Field | From → to |
| --- | --- |
| `cap_us` | Capture request → raw buffer returned (includes CSI wait, task wakeup and cache sync; **not** exposure time) |
| `enc_us` | Raw buffer returned → synchronous encoder returned |
| `pub_us` | Encoder returned → slot published (includes publisher mutex wait and copy) |
| `slot_us` | Published → callback has the mutex and starts copying (includes polling/scheduling) |
| `copy_us` | Callback starts copy → bytes/PTS ready for the library |

Their sum / 1000 is the reported `max` in ms (before rounding). `cb_us` is the
absolute ESP monotonic time at the end of `copy_us`, **before RTP send**. No
logging in the CSI ISR, per frame, on empty polls, or under the frame mutex.
The publisher prints the snapshot, not the timer callback. If the mutex is busy
at report time, the counters carry over rather than making either task wait
to report. UART output still costs time between captures: compare `pipeline:`
FPS and the optical result with the uninstrumented run.

On the PC, run this in a shell with raw stderr redirection (e.g. `cmd.exe` or
Bash). Reproduce the lag, quit with **q**, then summarize the log offline:

```sh
ffplay -hide_banner -nostats -loglevel datetime+debug -fdebug ts -rtsp_transport udp -fflags nobuffer -flags low_delay -framedrop -sync video -vf "showinfo=checksum=0,setpts=N" -max_delay 0 -reorder_queue_size 0 -analyzeduration 0 -probesize 32768 rtsp://192.168.2.5:8554/live 2>player.log
python tools/ffplay_timing.py player.log --skip 10
```

One session per file. Log to disk, not a scrolling terminal; omit `--skip` to
include acquisition. For the A/B, remove **only** `,setpts=N`. The script joins
the last PTS-labelled fragment read, parser output, and filter log of the same
picture in the `1/90000` time base. Its maximum includes the offending PTS.
An AU → filter spike can include **backpressure from a full picture queue**,
not just slow decoding. Probe size and waiting for SPS/PPS mainly affect joining.

### 2. Split the optical number on one clock (no clock sync required)

Show a large **local PC clock with milliseconds** next to FFplay and point the
OV5647 at the clock only. For a local clock page, save/open this HTML on the
playback PC:

```html
<pre id="clock" style="font: bold 7vw monospace"></pre><script>
function tick() { const d = new Date(); document.getElementById('clock').textContent =
  d.toTimeString().slice(0,8) + '.' + String(d.getMilliseconds()).padStart(3,'0');
  requestAnimationFrame(tick); } tick();
</script>
```

For this test, insert `drawtext` **between** `showinfo` and `setpts` in the
command above. Example filter on this Windows PC (substitute a local TTF on
other hosts):

```text
-vf "showinfo=checksum=0,drawtext=fontfile='C\:/Windows/Fonts/consola.ttf':text='PTS %{pts}':fontsize=40:fontcolor=white:box=1:boxcolor=black,setpts=N"
```

Film the clock and player together at 120/240 FPS. On the **first appearance**
of a new overlaid PTS, read the clock inside the camera image (`S`), the live
clock beside it (`V`), and the PC `datetime` on the `showinfo` line with that
`pts_time` (`D`). All three are on the **same PC clock**:

- `V − S`: optical clock difference. A random still can include an extra
  displayed-picture hold (up to ~100 ms at 10 FPS during normal playback).
- `D − S`: source-clock drawing/scanout + camera + encoder + delivery + decode.
- `V − D`: delay **after the filter relative to the live clock**. A large
  positive value implicates player scheduling/queue, renderer or display. The
  reference clock itself takes time to appear: shared panel delay can cancel
  here and appear in `D − S` instead. Use a known low-lag reference display if
  that distinction matters. Allow for refresh/scanout and rolling shutter;
  these are estimates, not exposure or vsync timestamps.

Repeat after warmup and after 60 s, before/after a window stall and after a
player restart. Use multiple transitions (median and p95). As an independent
check, film a physical blinking LED and its preview in one shot: number of
phone-frame intervals between the edges / phone FPS is real glass-to-glass time.
This avoids a slow source display, remote desktop, or unsynchronized stopwatches.

### 3. If the missing time is before `showinfo`, bound the wire interval

Capture on the **viewing PC's Ethernet adapter**, starting before SETUP. In
Wireshark decode TCP 8554 as RTSP and the negotiated video UDP port as RTP if
needed. Check a single SSRC, sequence continuity and marker bits. For a complete,
in-order picture, record the PC time of its last (marker) packet, `M`. First to
last packet is only burst duration, not one-way delay.

- Match `trace.pts_ms` to the H264/90000 RTP timestamp: this library scales ms
  by 90, modulo 2^32. Verify the match over several samples (PTS steps and AU
  sizes); account for any session offset. FFplay rebases its PTS: for this
  stream use the RTP timestamp corresponding to `ff_read_packet pts=0` as the
  origin, **not** the first *decoded* IDR. Never use timestamps after `setpts`.
- **Bound the clock offset instead of subtracting PC time from ESP uptime.**
  In this server the serial order is received SETUP → `RTSP state: 5 event_us=B`
  → TX SETUP 200. Verify that order for the session. With PC times `H0`, `H1`
  for that SETUP request/reply (same CSeq), the offset `PC − ESP` lies between
  `L=H0−B` and `U=H1−B` in common units. For a matched handoff `C=cb_us`, the
  handoff → last packet interval is **between `M−C−U` and `M−C−L`**. This
  gives a range, not half-RTT accuracy. Use a short run, allow for clock drift,
  and reject clock steps, ambiguous matches or a wide handshake bracket. Serial
  *arrival* time isn't needed. Without a clock anchor only *changes* in delay
  across frames can be compared, not a constant one-way offset.
- `ff_read_packet` (last fragment for that PTS) minus `M`, then the script's two
  intervals, localize **host socket/read backlog vs parser vs decoder/queue**.
  Use the PC wall clock for both (Wireshark time of day including ms).

**Choose the fix from the first large interval.** Large `cap/enc/pub/slot`:
inspect that board stage, not FFplay. Large handoff → marker or missing packets:
inspect AU bursts, link and sender/receiver congestion. A clean small wire
interval but late reads implicates the host reader/socket queue. A repeatable
~100 ms in the parser is a reason to A/B a player with RTP-marker-aware AU
assembly; do **not** disable H.264 parsing and feed FU-A fragments as pictures.
If `V−D` holds most of the 700 ms, A/B a different renderer/display or a viewer
that keeps the newest **decoded** picture; don't drop encoded P references.
No player flag or firmware fix is justified for the remaining delay until the
large interval appears in the same optical run.

## Earlier receiver findings

There is a demonstrated receiver-side buffering mechanism in the installed
FFplay 9.0.2. For this video-only stream, its default audio-master setting falls
back to an external clock. FFplay deliberately slows that clock when its
compressed-packet queue is small, building a reserve of frames. At 10 FPS,
each buffered frame costs 100 ms. `-fflags nobuffer` does not remove this clock
controller, the decoder queue, or the display queue.

Changing only the sync mode to `-sync video` substantially reduced initial
receiver queueing in short tests, but a later 60-second test reproduced a
persistent backlog after a display-thread stall even with this option. The
player can re-anchor its display schedule and continue replaying old pictures
at source speed instead of returning to the live edge. See the accumulation
follow-up below; `-sync video` alone is not a bounded-latency guarantee.

For video-only live preview, use untimed presentation as well:

```sh
ffplay -rtsp_transport udp -fflags nobuffer -flags low_delay -framedrop -sync video -vf "setpts=N" -max_delay 0 -reorder_queue_size 0 -analyzeduration 0 -probesize 32768 rtsp://192.168.2.5:8554/live
```

The filter assigns consecutive decoded pictures one timebase tick apart, so
backlogged pictures do not each incur another nominal source-frame period.
This deliberately changes presentation timestamps: it is a video-only live
preview workaround, not appropriate for preserving A/V or source timing. It
does not modify firmware/RTP timestamps or guarantee zero glass-to-glass delay.

`-sync ext` is not the right recommendation here: the default already falls
back to that clock when audio is absent. Zero RTP reordering delay trades
robustness to reordered/lost packets for latency.

The original evidence section describes the pre-fix baseline. The accumulation
follow-up separately records measurements after flashing the firmware fixes.

## Follow-up firmware changes

- Replaced fixed QP with `CONFIG_H264_ENCODER_QP_MIN=26` and
  `CONFIG_H264_ENCODER_QP_MAX=51`. The encoder rejects equal/reversed bounds,
  ensuring the component enables bitrate control. Target remains 1 Mbit/s;
  actual bitrate depends on the scene, QP limits and achieved frame cadence.
- Added `CONFIG_RTSP_SEND_POLL_HZ=200` and `rtsp_config.send_poll_hz`. The
  library's maximum-FPS timer field now polls every 5 ms instead of using the
  encoder's 100-ms nominal video period. Encoder/SPS FPS and GOP remain 10.
  Empty callbacks return immediately; this is not 200-FPS video. Timer dispatch
  and sending work can still add delay beyond the nominal polling period.
- Added measured encoded kbit/s and maximum access-unit size to the existing
  once-per-second pipeline summary, plus startup logs for RC bounds and polling.

The freshness trace below records the investigated baseline; line numbers may
have shifted after these changes.

## Accumulation follow-up on the updated firmware

A 60-second test relayed original camera RTP packets to FFplay with `-sync
video` while independently reading passive serial diagnostics. Results:

- Firmware callback capture-request age remained around 101–103 ms per bucket.
- Publish-to-callback wait remained about 1–3 ms.
- 602 picture timestamps, no missing/reordered RTP packets; packet bursts
  averaged 1.87 ms, with a maximum of 7.27 ms.
- Relative RTP/arrival timing showed no accumulating clock drift. An initial
  queued picture accounted for a roughly 76-ms starting offset.
- Receiver playback-clock age averaged about 159 ms for the first 20 seconds.
- At roughly 25.7 seconds, playback/status updates stalled for about 2.8 seconds.
  The decoder filled the picture queue and then blocked, while RTP kept arriving.
- After rendering resumed, the receiver stayed roughly 2.97 seconds behind for
  the remaining 30 seconds, with a compressed-packet backlog and zero reported
  dropped frames. Firmware callback age did not grow.

This reproduces the reported symptom and locates the growing backlog downstream
of firmware capture/handoff in this test. Restarting the player discards that
backlog. The cause of the display stall itself was not identified; window
operations, scheduling or renderer stalls are possible, not established causes.
The playback-clock figures are receiver estimates, not monitor scanout times.

A separate controlled test used a synthetic 10-FPS H.264/RTP stream and the same
FFplay binary with a dummy/software renderer. Only the diagnostic child's main
thread was stalled for 1.5 seconds; other processes were not suspended:

| Recovery observation | `-sync video` | Same, plus `-vf "setpts=N"` |
| --- | ---: | ---: |
| RTP-complete -> decoded filter before stall | ~100 ms | ~100 ms |
| Same delay after resuming | Stayed ~1,300 ms | Returned to ~100 ms in next second |

`showinfo` was placed before `setpts` so the measurements used original RTP
presentation timestamps, not rewritten timestamps. The untimed workaround was
validated in this controlled receiver test; a new optical measurement on the
camera is still needed. No further firmware behavior was changed for this fix.

## Evidence from the live stream (original baseline)

### Direct UDP metadata capture

An eight-second RTSP/UDP probe received original RTP packets without decoding
or retaining image data. It primed the outbound UDP flow to permit replies
through the PC's stateful firewall; no firewall settings were changed.

Representative baseline:

- 80 frame timestamps, 14,694 RTP packets.
- No missing sequence numbers and no reordered packets.
- Frame arrival interval: 97.86 / 99.98 / 102.64 ms (min / average / max).
- RTP timestamp increment: 99 / 99.99 / 101 ms, using H264/90000.
- First-to-last RTP packet of a frame: 28.25 / 29.62 / 35.97 ms.
- Arrival time minus RTP clock, relative to the first frame: within about 3 ms.
- Exactly one RTP marker per frame, and IDRs spaced ten frames apart.
- Frame sizes about 217–248 KB including RTP headers; about 17.7 Mbit/s.
  Other samples were around 19–20 Mbit/s, with packet bursts up to 43 ms.

This rules against accumulating network backlog in these samples and confirms
correct relative RTP timing. It does NOT, by itself, rule out a constant
upstream image-age offset: sender and PC clocks were not synchronized.

`ffprobe` also confirmed Constrained Baseline H.264, 1920x1080, 10 FPS, and
`has_b_frames=0`. A headless FFmpeg decode test with low-delay flags reported
approximately 7–10 ms decoding after demux output, not one second. That is
not a measurement of demux/parser delay or glass-to-glass latency.

### FFplay buffering comparison

For a receiver-side measurement, the probe forwarded the original RTP packets
immediately over loopback to FFplay, with the same H.264 payload/timestamps.
Packet completion times were compared with timestamped FFplay `showinfo` output
and its printed playback clock. The SDP receiver used the same low-buffer
options; changing sync mode was the relevant A/B difference.

| Receiver-side interval | Default sync | `-sync video` |
| --- | ---: | ---: |
| Complete RTP frame -> decoded filter, average | 153 ms | 76 ms |
| Complete RTP frame -> printed playback clock, average | 448 ms | 124 ms |
| Complete RTP frame -> printed playback clock, maximum | 706 ms | 138 ms |

Playback-clock estimates are approximate: status is sampled, the frame clock
is interpolated between pictures, and this does not timestamp monitor scanout.
The tests were separate eight-second samples, not identical scene recordings.

A second comparison used **direct RTSP/UDP**, not the relay. Each player ran
for nine seconds with the same flags and `showinfo`; only sync mode differed:

| Direct RTSP receiver observation | Default sync | `-sync video` |
| --- | ---: | ---: |
| Compressed packet queue, average | 140 KB | 0 KB in status samples |
| Compressed packet queue, maximum | 749 KB | 0 KB in status samples |
| Latest filter PTS ahead of playback clock, average | 210 ms | -38 ms |
| Same PTS difference, maximum | 362 ms | 19 ms |
| Reported dropped frames at end | 0 | 0 |

The negative PTS difference means the interpolated playback clock had advanced
past the most recently decoded picture; it is not negative physical latency.
Together these tests demonstrate extra player queueing even with `nobuffer`
and `low_delay`, and its reduction with video-master sync.

### Why FFplay does this

The matching upstream [FFplay n9.0.2 source](https://github.com/FFmpeg/FFmpeg/blob/n9.0.2/fftools/ffplay.c)
contains:

- Default `av_sync_type = AV_SYNC_AUDIO_MASTER`.
- `get_master_sync_type()`: no audio -> `AV_SYNC_EXTERNAL_CLOCK`.
- `check_external_clock_speed()`: when the video packet queue has <=2 packets,
  decrease external-clock speed, down to 0.900x. At >10 packets it can speed up;
  in between, speed returns gradually to 1.0x.
- `video_refresh()`: runs this speed controller for realtime external-clock
  playback and schedules display against the chosen master clock.
- A three-entry decoded-picture queue, plus a separate compressed-packet queue.

The slowdown can add about 100 ms of lag per second while at 0.9x. Once there
is a reserve, FFplay can continue playing smoothly but behind the live edge.
`-framedrop` does not guarantee catch-up to real capture time: it compares with
its playback clock, which can itself be behind live video.

Do not mistake `MIN_FRAMES=25` for a mandatory startup prebuffer. In this code
it controls when the demux thread stops filling queues, not a requirement to
collect 25 frames before displaying anything.

## Firmware trace: freshness, not just throughput

### 1. Sensor / CSI / ISP

Relevant application code: `main/camera.c:520`, `main/camera.c:608`.

- The sensor remains in its 1920x1080 **30-FPS** mode. The encoder and RTSP
  target of 10 FPS does not reprogram sensor timing.
- One application raw buffer; one queued capture request at most.
- A capture is submitted only after encoding the preceding buffer has finished.
- Completion is signalled by a binary semaphore after that requested DMA frame.
- The ESP-IDF CSI driver has one internal backup/discard buffer. When there is
  no application request, subsequent sensor frames overwrite this discard
  buffer; it is not a FIFO later replayed to the application.
- The ISP corrections used here are spatial/line processing, not a configured
  temporal queue of many pictures.

The CSI implementation was checked in
`$IDF_PATH/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c`, especially
`csi_dma_trans_done_callback()` and `s_ctlr_csi_receive()`.

The roughly 58.5-ms `camera_frame()` duration includes waiting for the CSI
request to become active and for the requested frame to finish. It is not
proof of a 58.5-ms-old picture: some of that time precedes capture of the
returned image. Conversely, it does not measure the preceding sensor exposure.

No multi-frame camera FIFO was found that would explain one second of age.

### 2. Encoder

Relevant code: `main/encoder.c:139`, `main/encoder.c:150`,
`managed_components/espressif__esp_h264/hw/src/esp_h264_enc_single_hw.c`.

- Hardware encoding is synchronous and produces the input picture on return.
- Output PTS/DTS are copied from the input PTS; no future input is awaited.
- Pictures are IDR/P, not B frames. SPS uses one reference frame and POC type 2.
- SPS/PPS are prepended at each IDR; the encoder does not wait for a whole GOP
  before returning output.
- The PTS originates at encode submission, not sensor exposure. Changing its
  epoch will not remove a real queue. The measured relative RTP clock was sound.

**Baseline configuration issue (fixed above):**
`main/encoder.c` set `qp_min == qp_max == 26`. In
`hw/src/esp_h264_enc_hw_param.c:367`, the component creates a rate controller
only when `qp_min < qp_max`. Therefore the configured 1,000,000-bit/s target was
not enforced in that fixed-QP mode. The measured 17–20 Mbit/s is real, not a
1-Mbit/s stream with an unusually large network buffer.

This costs tens of milliseconds per frame for packet transmission and makes
loss/congestion more likely. It does not explain a clean, fixed one-second
FIFO in the measured Ethernet samples.

### 3. Application RTSP handoff

Relevant code: `main/rtsp.c:290`, `main/rtsp.c:456`.

- One latest-frame slot, protected only during copies.
- Publishing replaces an unread access unit; no encoded-frame FIFO accumulates.
- The callback consumes a pending access unit once, otherwise returns zero bytes.
- `max_frame_size` / `video_info.len` are byte-capacity limits, not a target fill
  level or number of buffered pictures. Shrinking them blindly risks overflow.

Overwriting encoded P frames can break downstream references until an IDR;
a low-latency design must prefer discarding raw pictures **before encoding**,
not arbitrarily dropping already-encoded reference pictures.

### 4. Espressif RTSP library and network

The protocol component supplies a precompiled archive, not RTSP server source:
`managed_components/espressif__esp_media_protocols/lib/esp32p4/libesp_media_protocols.a`.
The server/RTP functions were inspected using RISC-V disassembly and symbols.

- `_rtsp_server_task` starts an `esp_timer` at `1000000 / video_info.fps` us.
  With the baseline value 10, this was a **100-ms polling interval**; the
  follow-up maximum/poll rate is 200, giving a 5-ms nominal interval.
- `_rtp_video` calls our callback, then calls `esp_rtp_send()` with the returned
  picture. It does not first accumulate ten pictures or wait for a full GOP.
- Callback PTS is interpreted in milliseconds and converted into the RTP 90-kHz
  timebase. This agrees with measured 9,000-tick steps.
- `esp_rtp_send()` packetizes the current access unit and writes the packets in
  the same call. Its temporary packet arrays describe the current picture,
  not a one-second video FIFO.
- The timer can add nearly 100 ms of phase-dependent handoff delay under normal
  operation; it is not a one-second buffer. Long callback work can delay timers.
- `.trans = RTSP_TRANSPORT_TCP` is documented as a **client** default. In server
  mode the SETUP negotiation selected UDP, verified by the live response. RTSP
  control still using TCP is normal.

The existing callback-age summary stops before library packetization/network
transmission. It cannot alone measure complete sender-to-monitor latency.

The board serial port was already open by another process, so fresh callback
age logs could not be collected without disturbing that process.

## Startup delay is a different issue

The live SDP lacks `sprop-parameter-sets`. A viewer joining between IDRs sees
P frames without SPS/PPS until the next IDR, explaining the initial PPS errors.
With GOP=10 at actual 10 FPS, that wait can approach one second. Aggressive
probing can finish before dimensions/framerate are known.

That delays initial acquisition, not every subsequent motion event. Reducing
GOP or requesting an IDR on PLAY helps startup/recovery, but does not inherently
remove steady-state buffering. A very small GOP can increase bitrate further.

## Prioritized latency strategy from the baseline (items 2–3 now implemented)

1. **Use video-master FFplay sync first and repeat the optical test.** This
   directly addresses demonstrated receiver queueing, without changing capture
   FPS, resolution, or firmware. Measure after startup as well as immediately.
2. **Allow rate control to work.** Expose a QP range instead of pinning min/max
   to the same value; choose bounds/bitrate with image-quality testing. Log
   encoded bytes/s and maximum picture size, not just the target bitrate.
3. **Decouple RTSP polling interval from nominal encoder FPS.** Prefer a sender
   that is notified when a frame is ready, or a separately configurable faster
   poll rate. The current public API calls this field maximum video FPS; do not
   blindly change encoder/SPS FPS merely to make the library poll faster.
   Do not block inside the shared ESP timer callback waiting for a frame.
4. **Only then consider latest-raw-frame capture overlap.** Properly owned raw
   buffers can keep the sensor running and select a fresh completed picture
   for encoding. This should be a latest-frame policy, not a deeper FIFO.
5. **Force a fresh IDR for new sessions** if rapid first-picture acquisition is
   needed. Preserve SPS/PPS/IDR continuity; do not implement an encoded-frame
   drop policy that silently loses references.

A useful latency budget is:

```text
sensor exposure/readout and capture phase
+ encoding current picture
+ publish/callback copies and polling wait
+ RTP packetization and last-packet arrival
+ receiver reassembly/parser and decoding
+ receiver scheduling/queues
+ display scanout / measurement-display delay
```

The measured ~100-ms capture/encode/publish total is only part of this sum.
There is no evidence justifying dismissing a one-second optical observation
because the producer makes ten frames per second. The receiver tests instead
show where several hundred milliseconds can be introduced and removed.
