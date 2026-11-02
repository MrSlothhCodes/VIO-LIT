# Camera ORB

A small Rust project that opens the default webcam and overlays ORB (Oriented FAST and Rotated BRIEF) keypoints in real time.
# To do
Integrate IMU
Kalman Filter
Micro Ros for Comms
## Overview

This project demonstrates:

- opening a camera with OpenCV
- converting frames to grayscale
- detecting ORB features
- drawing detected keypoints back onto the live image
- displaying the result in a window

It is a lightweight example for learning OpenCV + Rust integration and computer vision feature detection.

## Project structure

```text
.
├── Cargo.toml
├── src/
│   └── main.rs
└── README.md
```

## Requirements

Before running the project, install the required OpenCV development libraries and tools.

### macOS

```bash
brew install opencv
```

If `pkg-config` cannot find OpenCV, set the path:

```bash
export PKG_CONFIG_PATH="$(brew --prefix opencv)/lib/pkgconfig:$PKG_CONFIG_PATH"
```

### Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y libopencv-dev pkg-config
```

### Windows

Install OpenCV and ensure the OpenCV build environment is available to the Rust toolchain. See the `opencv-rust` setup documentation for the correct configuration for your toolchain.

## Build and run

From the project root:

```bash
cargo build
cargo run
```

The program will open the default webcam and show a window named `Camera ORB`.

- Press `Esc` to exit the window.
- The green points are the detected ORB keypoints.

## Dependencies

The project uses:

- Rust
- OpenCV via `opencv-rust`
- `clang-runtime` feature enabled in Cargo

## Notes

The example is intentionally simple and is intended for experimentation, learning, and prototyping. For production computer vision pipelines, you would typically add:

- camera calibration
- feature matching between frames
- motion estimation or pose estimation
- robust tracking and filtering

## License

This project is provided as-is for educational and experimental use.
