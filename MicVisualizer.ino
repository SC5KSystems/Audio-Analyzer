/*/
©2024 SC5K Systems 
Not for commercial use
/*/


#include <Adafruit_Protomatter.h>
#include "arduinoFFT.h"

// Matrix size definitions
#define WIDTH 64
#define HEIGHT 32

// RGB matrix pinout for MatrixPortal S3-ESP32
uint8_t rgbPins[]  = {42, 41, 40, 38, 39, 37};
uint8_t addrPins[] = {45, 36, 48, 35, 21};
uint8_t clockPin   = 2;
uint8_t latchPin   = 47;
uint8_t oePin      = 14;

// Initialize the Protomatter matrix
Adafruit_Protomatter matrix(
  WIDTH, 6, 1, rgbPins, 4, addrPins, clockPin, latchPin, oePin, true
);

// FFT configuration
#define MIC_PIN A1
#define GAIN_PIN A2
const uint16_t samples = 64;
const double samplingFrequency = 1000; 
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

int barHeights[WIDTH / 3]; 
bool redlined[WIDTH / 3]; 

const int smoothingFactor = 4;
double smoothedInput[samples];

void setup() {
  Serial.begin(115200); 
  delay(1000); 

  pinMode(GAIN_PIN, OUTPUT);
  digitalWrite(GAIN_PIN, LOW); 

  ProtomatterStatus status = matrix.begin();
  if (status != PROTOMATTER_OK) {
    Serial.print("Matrix initialization failed with status: ");
    Serial.println((int)status);
    for (;;);
  }

  for (int i = 0; i < WIDTH / 3; i++) {
    barHeights[i] = 1;
    redlined[i] = false;
  }

  Serial.println("MatrixPortal setup complete.");
}

void readFFT() {
  unsigned long microseconds = micros();
  
  for (int i = 0; i < samples; i++) {
    double rawInput = analogRead(MIC_PIN);
    smoothedInput[i] = (smoothedInput[i] * (smoothingFactor - 1) + rawInput) / smoothingFactor;
    vReal[i] = smoothedInput[i];
    vImag[i] = 0;
    
    while (micros() - microseconds < (1000000 / samplingFrequency)) {
      // Wait for next sample
    }
    microseconds += (1000000 / samplingFrequency);
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  for (int i = 2; i < (WIDTH / 3) + 2; i++) {  
    barHeights[i - 2] = map(vReal[i], 0, 1023, 1, HEIGHT);
    redlined[i - 2] = (barHeights[i - 2] > HEIGHT * 0.6);
  }
}

void drawBars() {
  int barWidth = 2;       // Use 2 pixels wide bars
  int barGap = 1;         // 1 pixel gap between bars
  int numBars = WIDTH / (barWidth + barGap); // Calculate number of bars based on matrix width

  // Calculate offset for centering the bars on the 64-pixel wide matrix
  int offsetX = (WIDTH - (numBars * (barWidth + barGap) - barGap)) / 2;

  for (int i = 0; i < numBars; i++) {
    int barX = offsetX + i * (barWidth + barGap);  // Start drawing from the offset position

    // Clear the entire area where the bar will be drawn first
    for (int y = 0; y < HEIGHT; y++) {
      for (int x = barX; x < barX + barWidth; x++) {
        matrix.drawPixel(x, HEIGHT - y - 1, 0);  // Clear the pixel (black)
      }
    }

    // Now draw the bar with the updated height and colors
    for (int y = 0; y < barHeights[i]; y++) {
      uint16_t color;
      if (redlined[i]) {
        color = matrix.color565(255, 0, 128); // Violent pink when redlined
      } else {
        color = getBarColor(y, barHeights[i]); // Dynamic gradient
      }

      // Draw the pixels for this bar
      for (int x = barX; x < barX + barWidth; x++) {
        matrix.drawPixel(x, HEIGHT - y - 1, color); // Draw from bottom to top
      }
    }
  }
}

uint16_t getBarColor(int y, int barHeight) {
  int gradientHeight = barHeight;

  if (y < gradientHeight / 3) {
    uint8_t red = map(y, 0, gradientHeight / 3, 128, 0);
    uint8_t blue = 255;
    return matrix.color565(red, 0, blue);
  } else if (y < (2 * gradientHeight / 3)) {
    uint8_t blue = map(y, gradientHeight / 3, (2 * gradientHeight / 3), 255, 0);
    uint8_t green = map(y, gradientHeight / 3, (2 * gradientHeight / 3), 0, 255);
    return matrix.color565(0, green, blue);
  } else {
    return matrix.color565(0, 255, 0);
  }
}

void loop() {
  readFFT();
  drawBars();
  matrix.show();
  delay(20); // Adjust for smoother animation
}
