# VIO-Lite

A lightweight Rust/OpenCV visual-odometry prototype. It reads frames from the
default camera, detects ORB features, tracks them between frames with
Lucas–Kanade optical flow, estimates relative camera motion, and displays a
top-down path visualization.

An optional MPU6050/ESP32 sketch streams IMU samples over serial. The Rust app
currently reports those samples for monitoring; it does **not** fuse IMU data
into the visual pose estimate. This is a prototype, not a complete visual-
inertial odometry system.

## What it does

- Opens camera device `0` and displays the tracked image.
- Uses ORB to seed feature points, then tracks them between frames.
- Estimates relative motion with an essential matrix and accumulates a
  monocular visual-odometry path.
- Displays the estimated path in a second, top-down window.
- Optionally reads MPU6050 samples from a serial port and prints a throttled
  one-second summary.

Monocular translation has unknown scale, and the camera intrinsics in the code
are placeholders. The plotted path is therefore an approximate visualization,
not a metric trajectory.

## Requirements

- Rust and Cargo (install with [rustup](https://rustup.rs/)).
- OpenCV development libraries, including the `calib3d`, `features2d`,
  `highgui`, `imgproc`, `video`, and `videoio` modules.
- `pkg-config` and a compatible Clang/libclang installation for the Rust
  OpenCV bindings.
- A connected camera for the visual-odometry demo.
- Optional: an ESP32 with an MPU6050 sensor for IMU serial readings.

### macOS

Install OpenCV and build tools:

```bash
xcode-select --install
brew install opencv pkg-config llvm
```

If OpenCV or libclang is not found during the build, configure the paths for
your Homebrew installation. For Apple Silicon, Homebrew is usually under
`/opt/homebrew`; for Intel Macs it is usually under `/usr/local`.

```bash
export PKG_CONFIG_PATH="$(brew --prefix opencv)/lib/pkgconfig:$PKG_CONFIG_PATH"
export LIBCLANG_PATH="$(brew --prefix llvm)/lib"
```

### Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y build-essential pkg-config clang libclang-dev libopencv-dev
```

### Windows

Install Rust with rustup, OpenCV development libraries, and LLVM/Clang. Configure
the OpenCV and libclang environment paths for your installation before building.
The [`opencv` crate setup guide](https://docs.rs/opencv/latest/opencv/) has
platform-specific configuration details.

## Build and run

From the repository root:

```bash
cargo build
cargo run
```

The program uses camera index `0` by default and opens two windows:

- **Camera VIO Frontend** — camera image with tracked points drawn in green.
- **Path (top-down x-z)** — accumulated approximate camera path.

Move the camera through a well-lit, textured scene for useful feature tracking.
Press **Esc** or **q** while a display window is focused to quit.

If the default camera cannot be opened, connect a camera or update the camera
index in `src/main.rs`.

## Optional IMU setup

The firmware sketch is in
[`IMU/IMU_6050/IMU_6050.ino`](IMU/IMU_6050/IMU_6050.ino). It is written for an
ESP32-style Arduino environment and uses the Adafruit MPU6050 and Adafruit
Unified Sensor libraries. In the sketch, check the board's I2C SDA/SCL pins and
install those libraries through the Arduino Library Manager before uploading.

The sketch emits IMU records at **921600 baud**. Connect the board and set
`IMU_PORT` to its serial device before starting the Rust application:

```bash
# macOS / Linux example
IMU_PORT=/dev/ttyUSB0 cargo run
```

On macOS, the device may instead be named `/dev/cu.usbmodem...` or
`/dev/cu.usbserial...`. On Windows, set the variable in PowerShell:

```powershell
$env:IMU_PORT = "COM3"
cargo run
```

If `IMU_PORT` is not set, the app defaults to `/dev/cu.usbmodem1101`. If the
serial port cannot be opened, an error is printed and the camera demo continues
without IMU readings.

## Project layout

```text
.
├── Cargo.toml
├── Cargo.lock
├── src/
│   └── main.rs
├── IMU/
│   └── IMU_6050/
│       └── IMU_6050.ino
└── README.md
```

## Current status

Implemented: camera capture, ORB feature initialization, optical-flow tracking,
relative pose estimation, path visualization, and optional IMU serial
monitoring.

Not yet implemented: IMU/vision sensor fusion, camera calibration, metric-scale
recovery, and robust long-term tracking.
