#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33;
#define TA_SHIFT 8
float mlx90640To[768];
paramsMLX90640 mlx90640;

// Standard temperature threshold settings
float temperatureThreshold = 40.0;     // Threshold in degrees Celsius
int minPixelsAboveThreshold = 5;       // Minimum number of pixels that must exceed threshold

// Extreme temperature threshold settings (for single/few pixel detection)
float extremeTemperatureThreshold = 60.0;  // Threshold for extreme heat in degrees Celsius
int minPixelsAboveExtremeThreshold = 1;    // Even a single pixel this hot is significant

// Track state to avoid spamming
bool lastFireDetected = false;      
unsigned long lastSend = 0;

void setup() {
  Wire.begin();
  Wire.setClock(1000000);           // Max I2C speed (1MHz)
  Serial.begin(115200);             // Lower baud rate is fine for boolean data
  
  if (!isConnected()) {
    Serial.println("FIRE:ERROR:MLX90640 not detected!");
    while (1);
  }
  
  uint16_t eeMLX90640[832];
  int status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  status |= MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  
  if (status != 0) 
    Serial.println("FIRE:ERROR:EEPROM Error");
  else
    Serial.println("FIRE:INFO:MLX90640 initialized");
    
  // Set to 16Hz without SynchFrame
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); // 16Hz
  MLX90640_SetResolution(MLX90640_address, 0x03);  // 19-bit resolution
}

void loop() {
  uint16_t mlx90640Frame[834];
  
  // Get frame data
  int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
  if (status < 0) {
    Serial.println("FIRE:ERROR:Frame acquisition failed");
    delay(500);
    return;
  }
  
  // Calculate temperatures
  float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
  MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95, Ta - TA_SHIFT, mlx90640To);
  
  // Track various temperature metrics
  int pixelsAboveThreshold = 0;
  int pixelsAboveExtremeThreshold = 0;
  float maxTemp = -100.0;
  float avgTemp = 0.0;
  float top3Temps[3] = {-100.0, -100.0, -100.0}; // Track top 3 hottest pixels
  int hotPixelX = -1, hotPixelY = -1; // Position of hottest pixel
  
  // Process the thermal array
  for (int i = 0; i < 768; i++) {
    float temp = mlx90640To[i];
    avgTemp += temp;
    
    // Track max temperature
    if (temp > maxTemp) {
      maxTemp = temp;
      hotPixelX = i % 32; // Assuming 32x24 resolution
      hotPixelY = i / 32;
    }
    
    // Track top 3 temperatures
    if (temp > top3Temps[0]) {
      top3Temps[2] = top3Temps[1];
      top3Temps[1] = top3Temps[0];
      top3Temps[0] = temp;
    } else if (temp > top3Temps[1]) {
      top3Temps[2] = top3Temps[1];
      top3Temps[1] = temp;
    } else if (temp > top3Temps[2]) {
      top3Temps[2] = temp;
    }
    
    // Count pixels above thresholds
    if (temp > temperatureThreshold) {
      pixelsAboveThreshold++;
    }
    
    if (temp > extremeTemperatureThreshold) {
      pixelsAboveExtremeThreshold++;
    }
  }
  avgTemp /= 768;
  
  // Determine if fire is detected using dual-criteria approach
  bool fireDetectedNormal = (pixelsAboveThreshold >= minPixelsAboveThreshold);
  bool fireDetectedExtreme = (pixelsAboveExtremeThreshold >= minPixelsAboveExtremeThreshold);
  bool fireDetected = fireDetectedNormal || fireDetectedExtreme;
  
  // Only send updates when the state changes or periodically (every second)
  if (fireDetected != lastFireDetected || millis() - lastSend > 1000) {
    // Format: FIRE:STATUS:true/false:maxTemp:avgTemp:normalPixels:extremePixels:hotX:hotY:top3
    Serial.print("FIRE:STATUS:");
    Serial.print(fireDetected ? "true" : "false");
    Serial.print(":");
    Serial.print(maxTemp, 1);  // Max temp with 1 decimal place
    Serial.print(":");
    Serial.print(avgTemp, 1);  // Avg temp with 1 decimal place
    Serial.print(":");
    Serial.print(pixelsAboveThreshold);
    Serial.print(":");
    Serial.print(pixelsAboveExtremeThreshold);
    Serial.print(":");
    Serial.print(hotPixelX);
    Serial.print(":");
    Serial.print(hotPixelY);
    Serial.print(":");
    Serial.print(top3Temps[0], 1);
    Serial.print(":");
    Serial.print(top3Temps[1], 1);
    Serial.print(":");
    Serial.println(top3Temps[2], 1);
    
    lastFireDetected = fireDetected;
    lastSend = millis();
  }
  
  // Small delay to maintain frame rate
  delay(62);  // ~16Hz refresh rate
}

bool isConnected() {
  Wire.beginTransmission(MLX90640_address);
  return (Wire.endTransmission() == 0);
}
