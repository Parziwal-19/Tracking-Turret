# Tracking-Turret - Raspberry Pi 5 & Python 3 Version

A motion tracking turret updated for modern Raspberry Pi 5 and Python 3 compatibility.

**Original project**: https://www.youtube.com/watch?v=HoRPWUl_sF8

## Key Updates for Pi 5 & Python 3

- ✅ **Python 3.x compatibility** (tested with Python 3.9+)
- ✅ **Raspberry Pi 5 support** with updated GPIO libraries
- ✅ **Modern CircuitPython libraries** for motor control
- ✅ **OpenCV 4.x compatibility** with updated API calls
- ✅ **Improved error handling** and fallback options
- ✅ **Type hints and modern Python syntax**

## Hardware Requirements

- **Raspberry Pi 5** (or Pi 4 for backward compatibility)
- **Adafruit Motor HAT** or compatible stepper motor controller
- **2x Stepper Motors** (200 steps/revolution recommended)
- **USB Camera** or Pi Camera module
- **Relay module** (for firing mechanism)
- **Power supply** appropriate for your motors

## Installation Guide

### 1. Update System

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Python 3 and pip (if not already installed)

```bash
sudo apt install python3 python3-pip python3-venv -y
```

### 3. Enable I2C

For Raspberry Pi 5, enable I2C through raspi-config:

```bash
sudo raspi-config
```

Navigate to: `Interface Options` → `I2C` → `Enable`

Or manually add to `/boot/firmware/config.txt`:
```
dtparam=i2c_arm=on
```

### 4. Create Virtual Environment

```bash
python3 -m venv turret-env
source turret-env/bin/activate
```

### 5. Install Dependencies

```bash
# Clone the repository
git clone https://github.com/HackerHouseYT/Tracking-Turret.git
cd Tracking-Turret

# Install required packages
pip install -r requirements_pi5.txt
```

### 6. Alternative Installation Methods

#### Option A: CircuitPython Libraries (Recommended for Pi 5)
```bash
pip install adafruit-circuitpython-motorkit
pip install adafruit-circuitpython-motor
```

#### Option B: Legacy Adafruit MotorHAT (if Option A doesn't work)
```bash
pip install git+https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library
```

#### Option C: GPIO Library Options
```bash
# Try RPi.GPIO first
pip install RPi.GPIO

# If RPi.GPIO doesn't work on Pi 5, try lgpio
pip install lgpio
```

### 7. Install OpenCV

#### Method 1: Using pip (easiest)
```bash
pip install opencv-python imutils
```

#### Method 2: Build from source (for better performance)
Follow the official OpenCV installation guide for Raspberry Pi 5.

## Configuration

### Setting Parameters

Edit the parameters at the top of `turret_pi5.py`:

```python
### User Parameters ###

MOTOR_X_REVERSED = False    # Set to True if X-axis motor moves in wrong direction
MOTOR_Y_REVERSED = False    # Set to True if Y-axis motor moves in wrong direction

MAX_STEPS_X = 30           # Maximum steps for X-axis movement
MAX_STEPS_Y = 15           # Maximum steps for Y-axis movement

RELAY_PIN = 22             # GPIO pin for relay control

#######################
```

### Hardware Connections

- **Stepper Motor 1 (X-axis)**: Connect to Motor HAT port M1/M2
- **Stepper Motor 2 (Y-axis)**: Connect to Motor HAT port M3/M4
- **Relay**: Connect to GPIO pin 22 (or change `RELAY_PIN` parameter)
- **Camera**: USB camera on port 0, or configure camera port in code

## Usage

### Run the Program

```bash
# Activate virtual environment
source turret-env/bin/activate

# Run the turret program
python3 turret_pi5.py
```

### Operation Modes

1. **Motion Detection Mode**: Automatically tracks and follows detected motion
2. **Interactive Mode**: Manual control using keyboard commands

### Controls

**Interactive Mode:**
- `w` / `s`: Tilt up/down
- `a` / `d`: Pan left/right  
- `Enter`: Fire (in non-friendly mode)
- `q`: Quit

**Calibration Mode:**
- Follow on-screen prompts to align turret with camera

## Troubleshooting

### Common Issues

#### GPIO Permission Errors
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout and login again
```

#### Camera Not Found
```bash
# Check camera connection
lsusb  # For USB cameras
# or
vcgencmd get_camera  # For Pi camera
```

#### Motor HAT Not Detected
```bash
# Check I2C connection
sudo i2cdetect -y 1
# Should show device at address 0x60
```

#### Import Errors
```bash
# Make sure virtual environment is activated
source turret-env/bin/activate

# Reinstall dependencies
pip install -r requirements_pi5.txt
```

### Library Compatibility

The code automatically detects and uses the best available libraries:

1. **Motor Control**: CircuitPython libraries → Legacy Adafruit MotorHAT
2. **GPIO Control**: RPi.GPIO → lgpio (for Pi 5)
3. **OpenCV**: Automatically handles OpenCV 4.x API changes

## Differences from Original

### Python 3 Updates
- `print` statements → `print()` functions
- `raw_input()` → `input()`
- Integer division fixes (`//` instead of `/`)
- f-string formatting for better readability

### Library Updates
- Modern CircuitPython motor libraries
- OpenCV 4.x compatibility
- Improved error handling with try/except blocks
- Threading improvements

### Raspberry Pi 5 Compatibility
- Alternative GPIO libraries (lgpio fallback)
- Updated I2C configuration paths
- Modern dependency management

## Safety Notes

⚠️ **Important Safety Information:**

- This turret is designed for educational purposes
- **Always use in "friendly mode"** for testing
- Ensure safe environment when testing firing mechanisms
- Follow local laws and regulations regarding projectile devices
- Use appropriate eye protection during testing

## Contributing

Feel free to submit issues and enhancement requests for the Pi 5 version!

## License

Same as original project - check main repository for license information.

