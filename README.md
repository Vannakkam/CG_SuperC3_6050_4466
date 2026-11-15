# ESP32-C3 SuperMini — 8 kHz microphone / 200 Hz IMU / 50 ms BLE batching

## Files
- `ESP32C3_8k_50ms_Optimized/ESP32C3_8k_50ms_Optimized.ino` — firmware.
- `index.html` — matched Web Bluetooth dashboard with raw WAV, speech-denoised WAV, heart WAV, two CSV downloads and live graphs.

## Hardware
- MAX4466 VCC -> 3.3 V, GND -> GND, OUT -> GPIO0.
- MPU6050 VCC -> 3.3 V, GND -> GND, SDA -> GPIO8, SCL -> GPIO9; 7-bit address 0x68.
- 100 nF and 1 uF across microphone VCC/GND in **parallel**, near board.
- GPIO8/GPIO9 are ESP32-C3 strapping pins; wiring/pull-ups may affect boot.

## Required software
- Arduino-ESP32 **3.x** board core and **NimBLE-Arduino 2.x**, **Adafruit MPU6050**, **Adafruit Unified Sensor**, **Adafruit BusIO**.
- Chrome or Edge with Web Bluetooth; serve HTML from http://localhost or HTTPS. Avoid opening file:// directly.

## Acquisition and radio
- ADC continuous DMA at **24,000 conversions/s**, 31-tap Hamming-window low-pass ~3.25 kHz, decimated 3:1 to **8,000 samples/s**.
- MPU6050 sampled at nominal **200 Hz** (software scheduling can jitter).
- Nominal audio data **16 KB/s** (12-bit measurements carried in 16-bit words) plus IMU and BLE overhead.
- At **50 ms**, 400 microphone samples have accrued, needing >=4 BLE notifications with 108 samples per notification (at MTU 247). This schedules application sends in short bursts; it **does not** change BLE connection-event interval or RF duty-cycle guarantee.
- BLE needs MTU **>=100** to start. Prefer 247. If recording never starts, check adapter negotiated MTU.
- Queue = 8192 microphone samples (~1.024 s), 256 IMU samples (~1.28 s). Overflows are counted and sequence gaps show missing received samples. BLE notifications are **not acknowledged per sample**; no guaranteed lossless transport. Driver DMA overruns are reported as an acquisition error but cannot necessarily be counted exactly.
- Microphone sample timestamps are nominal 125 us increments. ADC filter causes group delay ~0.625 ms; browser time synchronization is approximate.
- No microcontroller flash or SD writes. Browser keeps received data in memory until page refresh, so download after each test.

## Usage
1. Open `.ino` in Arduino IDE; choose an ESP32-C3 SuperMini-compatible board and upload.
2. Open Serial Monitor 115200. Expected line: `BLE ready: C3-IMU-MIC | MIC 8000 Hz ...`. Close Monitor if needed.
3. From the directory containing `index.html`: `python -m http.server 8000`.
4. In Chrome/Edge open `http://localhost:8000/index.html`; click Pair and choose `C3-IMU-MIC`.
5. Click Start, collect 5-10 s, Stop, wait for `Recording finished` in the event log, then Download Raw WAV and CSVs.
6. For a 5 s recording, expect near 40,000 microphone samples and 1,000 IMU samples, depending on button/clock timing. Check browser sequence gaps AND ESP32 queue drops in the event log.

## Noise experiment (same hardware and location)
Record **three** similar 5 s trials:
A. Previous BLE-off 8 kHz diagnostic WAV — baseline.
B. Existing 16 kHz continuous BLE firmware WAV — noisy reference.
C. This 8 kHz, 50 ms BLE-burst firmware WAV — optimized trial.
Compare raw WAV recordings at matched gain; **do not use normalized/denoised exports to judge interference severity**. A reduction in buzzing does not prove whether source was RF supply coupling vs firmware/ADC contention; both can change with notification load.

## Important limitations
- The acquisition, transport and JS integration have not been verified on physical hardware. The JS source passes Node syntax checks, but no Arduino compiler toolchain is installed in this environment.
- The 'Raw WAV' is raw with respect to speech/heart denoising, but contains the required 24k-to-8k anti-alias FIR output and a browser-side DC removal/normalization for listening. The microphone CSV retains integer filtered ADC counts. For exact unmodified 24k ADC conversions, use a different capture path.
- ADC conversion timing on ESP32-C3 and BLE notification rates depend on ESP-IDF core, connection parameters and PC/phone BLE stack. Sample output, packet spacing and phase jitter should be measured.
- If 50ms bursts do not help, trial transmission of dummy notifications while buffering real mic samples locally; radio-current coupling remains plausible.
