# 55Knights-UAS4STEM
55 Knights UAS4STEM

## Project Overview

shi tuf boi uses yolo / hef modesl to track shi and lower drone on it, w sigma boi twin

## Features

- Hailo shi
- trackin
- MAVLINK

## Usage
### Basic Operation
```bash
python main.py
```

### Tracking Mode
```bash
python main.py --track --target QR
```

### With Custom Model
```bash
python main.py --model /path/to/model.hef --labels /path/to/labels.txt
```

### With MAVLink Connection
```bash
python main.py --track --target person --mavlink /dev/ttyAMA0 --baud 57600
```

### Custom Lowering Speed
```bash
python main.py --track --target Truck --lower_speed 0.03
```