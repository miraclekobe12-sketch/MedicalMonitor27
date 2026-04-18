# MAX30102 Vital Monitor — QNX RTOS

Real-time heart-rate (BPM) and blood-oxygen (SpO₂) monitor built for
**QNX Neutrino RTOS**, demonstrated on an embedded board at the
**Q-eHACK** hackathon.

---

## File Structure

```
.
├── vitals.h                  # Shared data contract (vital_data_t, DCMD_*)
├── vital_resmgr.c            # Resource Manager — I2C driver + algorithms
├── max30102_dashboard.c      # Display application — fetches from /dev/vital_sensor
├── Makefile                  # Build + deploy targets
└── README.md                 
```

---

## Building

```bash
make
```

Requires QNX SDP 8.0 with `qcc` in PATH. Tested on `aarch64` target.

---

## Running

On the QNX target board:

```bash
# 1. Start the Resource Manager (background)
./vital_resmgr &

# 2. Start the Dashboard
./max30102_dashboard
```

Press `Ctrl+C` on either process to shut down cleanly.

---

## Hardware

| Component | Detail |
|---|---|
| Sensor | MAX30102 pulse-oximeter (I2C addr `0x57`) |
| Bus | `/dev/i2c1` |
| Sample rate | 50 Hz (18-bit ADC, 411 µs pulse width) |
| LED current | ~7.2 mA (RED + IR) |
| Display | Any QNX Screen-compatible framebuffer |

---

## Algorithms

### Heart Rate (BPM)
- DC component removed with a first-order IIR filter (`α = 0.05`)
- Peak detection on the AC-coupled IR signal
- Instantaneous BPM averaged over the last 8 peaks

### SpO₂
- AC/DC ratio computed over a 100-sample window
- `SpO2 = 104 − 17 × R`  where  `R = (AC_red/DC_red) / (AC_ir/DC_ir)`
- Clamped to `[70, 100]%`

---

## Real-Time Design Decisions

- **No `usleep()`** in any periodic loop — all timing via `timer_create` + `MsgReceivePulse` for deterministic, drift-free cadence.
- **Two separate processes** — driver and UI — aligned with QNX microkernel philosophy (fault isolation).
- **`devctl()` over raw `read()`** — typed, versioned IPC that won't break if `vital_data_t` grows.
- **Double-buffered display** — eliminates tearing without vsync dependency.

---
