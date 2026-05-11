# ESP32 Audio Streamer

An ESP32 firmware for turning an I2S microphone into a small network audio source.

It was built for always-on outdoor/bird-audio use, especially with BirdNET-Go, but it also works with common players such as VLC and ffplay.

Tested on a **Wemos D1 Mini32**. Other ESP32 boards should work if the I2S pins are adjusted.

## What it does

- Captures mono audio from an I2S MEMS microphone
- Streams live audio over:
  - HTTP: `http://<device-ip>:81/stream.wav`
  - RTSP/TCP: `rtsp://<device-ip>:8554/audio/`
- Provides a browser UI at `http://<device-ip>/`
- Lets you configure Wi-Fi from a setup access point on first boot
- Includes BirdNET-Go-friendly audio profiles:
  - **Quality — 48 kHz**
  - **Stability — 24 kHz**
- Can optionally sleep at night and wake at civil dawn

## Hardware

Default I2S wiring:

| I2S signal | ESP32 GPIO |
|---|---:|
| WS / LRCLK | 25 |
| BCK / SCK | 33 |
| SD / DOUT | 32 |

Default capture channel is **right**. You can change pins/channel in `local_env.ini`.

## Quick start

### 1. Install PlatformIO

Use either the PlatformIO VS Code extension or the CLI.

### 2. Create local config

```bash
cp local_env.ini.example local_env.ini
```

Edit `local_env.ini` if needed for:

- I2S pins
- location/timezone
- deep sleep behavior
- optional Wi-Fi seed credentials

Wi-Fi credentials do **not** need to be hardcoded; the device can be configured from the setup web page.

### 3. Flash the ESP32

```bash
pio run --target upload
```

### 4. Watch the serial log

```bash
pio device monitor -b 115200
```

### 5. Configure Wi-Fi

On first boot, if no saved Wi-Fi credentials exist, the ESP32 starts a setup access point.

Connect to:

```text
ESP32-Audio-Setup-XXXXXX
```

Then open:

```text
http://192.168.4.1/
```

Save your Wi-Fi credentials. The page and serial log will show the device IP after it connects.

### 6. Open the dashboard

```text
http://<device-ip>/
```

From the UI you can:

- See stream status
- Play the HTTP stream
- Change audio profile
- Change HPF / gain conversion settings
- Restart audio
- Manage Wi-Fi
- View logs and system status

## Stream URLs

Recommended URLs:

| Use | URL |
|---|---|
| Browser / simple HTTP client | `http://<device-ip>:81/stream.wav` |
| Raw PCM HTTP client | `http://<device-ip>:81/stream.pcm` |
| BirdNET-Go / VLC / ffplay | `rtsp://<device-ip>:8554/audio/` |

RTSP is **TCP/interleaved only**. If a client first tries UDP, the firmware rejects that setup and waits for a TCP RTSP setup.

Only **one stream client** can be active at a time.

## BirdNET-Go recommendation

Start with:

- URL: `rtsp://<device-ip>:8554/audio/`
- Audio profile: **Stability — 24 kHz** if Wi-Fi is marginal
- Audio profile: **Quality — 48 kHz** if the stream is stable

The profile can be changed from the **Audio** page. After changing profile, click **Restart Audio**.

## Useful commands

Build:

```bash
pio run
```

Upload:

```bash
pio run --target upload
```

Monitor:

```bash
pio device monitor -b 115200
```

Test RTSP with ffplay:

```bash
ffplay -rtsp_transport tcp rtsp://<device-ip>:8554/audio/
```

Test HTTP WAV:

```bash
ffplay http://<device-ip>:81/stream.wav
```

## More documentation

Detailed information has been moved out of this README:

- [Streaming details](docs/streaming.md)
- [Configuration reference](docs/configuration.md)
- [Web UI and API reference](docs/api.md)
- [Architecture notes](docs/architecture.md)

## Acknowledgements

This project was influenced by BirdNET-Go ESP32 microphone work from the community, especially:

- [birdnetgo-esp32-rtsp-mic](https://github.com/Sukecz/birdnetgo-esp32-rtsp-mic)
- [birdnetgo-m5stack-atom-echo-rtsp-mic](https://github.com/stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic)
