# IoT Trend Demo: Edge AI with TensorFlow Lite

An ESP32-based demo showcasing edge AI with TensorFlow Lite for gesture recognition, integrated with AWS IoT Core.

## Prerequisites
- ESP32 DevKit
- Arduino IDE, `PubSubClient`, TensorFlow Lite for Microcontrollers
- AWS IoT Core (certs, endpoint)
- Accelerometer (e.g., MPU-6050) or simulated data

## Setup
1. **Model**: Train a gesture recognition model in TensorFlow, convert to .tflite, and generate `model.h` using `xxd -i`.
2. **Firmware**: Update `esp32_ai.ino` with WiFi/AWS details, flash to ESP32.
3. **Hardware**: Connect accelerometer (e.g., MPU-6050 SDA to GPIO 21, SCL to GPIO 22).
4. **Test**: Monitor `iot/edge_ai/data` in AWS IoT Core for gesture predictions.

## Demo
- Recognizes gestures (e.g., "Wave") at the edge, publishes to AWS.
- [Demo Video](demo.mp4) <!-- Add after testing -->
