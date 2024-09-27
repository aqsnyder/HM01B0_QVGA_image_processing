#include <Wire.h>

// Pin definitions for Raspberry Pi Pico
#define PIN_LED         25  // Onboard LED

// Camera pins (bottom row of HM01B0)
#define PIN_CAM_SIOD    4   // SDA (I2C0 SDA)
#define PIN_CAM_SIOC    5   // SCL (I2C0 SCL)
#define PIN_CAM_VSYNC   16  // VSYNC
#define PIN_CAM_HREF    15  // HREF
#define PIN_CAM_PCLK    14  // PCLK
#define PIN_CAM_D0      6   // D0 (moved from GPIO 5 to GPIO 6)

// Image dimensions
const int width = 96;
const int height = 96;
const int image_size = (width * height) / 8;  // For 1-bit per pixel
uint8_t image[image_size];

uint8_t header[2] = {0x55, 0xAA};

// I2C address of the HM01B0 camera (confirm from datasheet)
#define HM01B0_I2C_ADDR  0x24  // 7-bit address (make sure this is correct)

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to connect

  pinMode(PIN_LED, OUTPUT);

  // Set pin modes for camera signals
  pinMode(PIN_CAM_VSYNC, INPUT);
  pinMode(PIN_CAM_HREF, INPUT);
  pinMode(PIN_CAM_PCLK, INPUT);
  pinMode(PIN_CAM_D0, INPUT);

  // Initialize I2C on default pins (GPIO 4 and GPIO 5)
  Wire.begin();

  Serial.println("Initializing camera...");

  // Test I2C communication with the camera
  uint8_t chip_id = read_camera_register(0x0001);  // Replace 0x0001 with actual chip ID register
  Serial.print("Camera Chip ID: 0x");
  Serial.println(chip_id, HEX);

  // Configure camera
  configure_camera();

  Serial.println("Setup complete.");
}

void configure_camera() {
  // Reset the camera
  write_camera_register(0x0103, 0x01);  // Software reset
  delay(10);

  // Wait for camera to finish reset
  while (read_camera_register(0x0100) != 0x00);

  // Configure camera registers for 1-bit mode
  // Replace the following with actual register configurations based on HM01B0 datasheet
  // The addresses and values below are placeholders and need to be updated accordingly

  // Put camera in standby mode before configuration
  write_camera_register(0x0100, 0x00);  // Mode select: standby

  // Configure registers for single-bit mode
  // You must refer to the HM01B0 datasheet to get the correct register addresses and values
  // For example:
  write_camera_register(0x1000, 0x01);  // Example register (placeholder)
  write_camera_register(0x1001, 0x43);  // Example value to set 1-bit mode (placeholder)

  // Start the camera streaming
  write_camera_register(0x0100, 0x01);  // Mode select: streaming
}

void write_camera_register(uint16_t reg, uint8_t value) {
  Wire.beginTransmission(HM01B0_I2C_ADDR);
  Wire.write((reg >> 8) & 0xFF);  // High byte of register address
  Wire.write(reg & 0xFF);         // Low byte of register address
  Wire.write(value);              // Value to write
  Wire.endTransmission();
}

uint8_t read_camera_register(uint16_t reg) {
  Wire.beginTransmission(HM01B0_I2C_ADDR);
  Wire.write((reg >> 8) & 0xFF);  // High byte of register address
  Wire.write(reg & 0xFF);         // Low byte of register address
  Wire.endTransmission(false);

  Wire.requestFrom(HM01B0_I2C_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void capture_image(uint8_t *image_buffer) {
  // Clear image buffer
  memset(image_buffer, 0, image_size);

  Serial.println("Starting image capture...");

  // Wait for VSYNC to go high
  Serial.println("Waiting for VSYNC HIGH...");
  while (digitalRead(PIN_CAM_VSYNC) == LOW);
  Serial.println("VSYNC HIGH detected.");

  // Wait for VSYNC to go low
  Serial.println("Waiting for VSYNC LOW...");
  while (digitalRead(PIN_CAM_VSYNC) == HIGH);
  Serial.println("VSYNC LOW detected.");

  for (int y = 0; y < height; y++) {
    Serial.print("Capturing line ");
    Serial.println(y);

    // Wait for HREF to go high (start of line)
    while (digitalRead(PIN_CAM_HREF) == LOW);

    for (int x = 0; x < width; x++) {
      // Wait for PCLK rising edge
      while (digitalRead(PIN_CAM_PCLK) == LOW);
      // Read D0
      int bit = digitalRead(PIN_CAM_D0);

      // Wait for PCLK falling edge
      while (digitalRead(PIN_CAM_PCLK) == HIGH);

      // Calculate byte and bit positions
      int byteIndex = (y * width + x) / 8;
      int bitIndex = 7 - (x % 8);

      // Store bit in image buffer
      if (bit) {
        image_buffer[byteIndex] |= (1 << bitIndex);
      }
    }

    // Wait for HREF to go low (end of line)
    while (digitalRead(PIN_CAM_HREF) == HIGH);
  }

  Serial.println("Image capture completed.");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print("Received command: ");
    Serial.println(cmd);
    if (cmd == 'C') {
      Serial.println("Capture command received.");
      digitalWrite(PIN_LED, HIGH);  // Turn on LED to indicate capturing

      // Capture image
      capture_image(image);
      Serial.println("Image captured.");

      // Send header
      Serial.write(header, 2);
      Serial.println("Header sent.");

      // Send image data in chunks
      const int chunkSize = 64;
      for (int i = 0; i < sizeof(image); i += chunkSize) {
        int remaining = sizeof(image) - i;
        int sendSize = (remaining < chunkSize) ? remaining : chunkSize;
        Serial.write(&image[i], sendSize);
        delay(10);
      }
      Serial.println("Image data sent in chunks.");

      digitalWrite(PIN_LED, LOW);  // Turn off LED after capturing
      Serial.println("Capture complete.");
    } else {
      Serial.println("Unknown command.");
    }
  }
}
