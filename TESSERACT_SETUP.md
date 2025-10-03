# Tesseract Setup

## Installation and Building

### Package setup
```bash
mkdir -p ws_tesseract/src

vcs import ws_tesseract/src < ws_tesseract/src/dependencies.repos # this will take a while

rosdep install --from-paths src -iry
```

### Building
```bash
colcon build --symlink-install --cmake-args -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF -DBUILD_STUDIO=OFF
```