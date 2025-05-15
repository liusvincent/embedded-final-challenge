#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include <arduinoFFT.h>
#include <math.h>

#define SAMPLE_RATE 100 // Hz
#define DURATION 3 // seconds
#define NUM_SAMPLES (SAMPLE_RATE * DURATION)


#define FFT_SAMPLES 128
#define SAMPLING_FREQUENCY 100
#define MA_WINDOW 3


//for moving average filter
float peakFreq[MA_WINDOW] = {0};
int index = 0;


float movingAverage(float newValue){
    peakFreq[index]=newValue;
    index = (index+1)% MA_WINDOW;
    float sum =0;
    for (int i = 0; i < MA_WINDOW; i++) {
        sum += peakFreq[i];
    }
    return sum / MA_WINDOW;
}


// Collect raw accelerometer data
void collectAccelerometerData(double *accelMag) {
  // collect raw data
  unsigned long interval = 1000 / SAMPLE_RATE;
  for (int i = 0; i < FFT_SAMPLES; i++) {
    double ax = CircuitPlayground.motionX();
    double ay = CircuitPlayground.motionY();
    double az = CircuitPlayground.motionZ();
    accelMag[i] = sqrt(ax * ax + ay * ay + az * az) - 9.8;
    delay (interval);
  }
  // subtract the average
  double avg = 0;
  for (int i = 0; i < FFT_SAMPLES; i++) {
    avg += accelMag[i];
  }
  avg /= FFT_SAMPLES;
  for (int i = 0; i < FFT_SAMPLES; i++) {
    accelMag[i] -= avg;
  }
}


float FFT(double vReal[FFT_SAMPLES], double vImag[FFT_SAMPLES]) {
  ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Apply windowing
  FFT.compute(FFTDirection::Forward); // Perform FFT
  FFT.complexToMagnitude(); // Compute magnitudes
  float peakFrequency = FFT.majorPeak(); // Get the peak frequency
 
  return peakFrequency; // Return the peak frequency
}
//returns the current peak frequency other option is returning FFT




// NeoPixel feedback
void set_neopixel(float freq) {
  float level = 0;


  // Tremors: red (3-5 Hz)
  if (freq >= 3.0  && freq < 5.0 ) {
    level = floor((freq - 3) / 0.4 + 0.0000001);
    level = min(level, 4.0f);
    CircuitPlayground.playTone(500, 100); 
    for (int pixel = 0; pixel < 10; pixel++) {
      CircuitPlayground.setPixelColor(pixel, 0, 0, 0);
    }
    for (int pixel = 0; pixel < level + 1; pixel++) {
      CircuitPlayground.setPixelColor(pixel, 0xE6, 0x00, 0x00);
    }
  }
  // Dyskinesia: green (5-7 Hz)
  else if (freq >= 5.0 && freq <= 7.0) {
    CircuitPlayground.playTone(1000, 100); 
    level = floor((freq - 5) / 0.4 + 0.0000001);
    level = min(level, 4.0f);
    level = 9 - level;
    for (int pixel = 0; pixel < 10; pixel++) {
      CircuitPlayground.setPixelColor(pixel, 0, 0, 0);
    }
    for (int pixel = 9; pixel > level - 1; pixel--) {
      CircuitPlayground.setPixelColor(pixel, 0x00, 0xFF, 0x00);
    }
  }
  // Clear otherwise
  else if (freq > 7.0) {
    for (int pixel = 0; pixel < 10; pixel++) {
      CircuitPlayground.setPixelColor(pixel, 0, 0, 0);
    }
  }
}


void setup() {
  Serial.begin(9600);
  CircuitPlayground.begin();
  delay(1000);
}


void loop() {
  double accelMag[FFT_SAMPLES];
  double Imag[FFT_SAMPLES] = {0};
  collectAccelerometerData(accelMag);
  float peakFrequency = FFT(accelMag, Imag);
   // Only trust FFT if it is > than threshold
   float magnitudeSum = 0;
   for (int i = 1; i < FFT_SAMPLES / 2; i++) {
     magnitudeSum += accelMag[i];  // only look at positive frequencies
   }
   float avgMagnitude = magnitudeSum / (FFT_SAMPLES / 2);
 
   // Only accept the peak frequency if signal is strong
   if (avgMagnitude < 1) {  // tweak this value as needed
     peakFrequency = 0.0;
   }

  float smoothedFreq = movingAverage(peakFrequency);
  Serial.print("Peak Frequency (Smoothed): ");
  Serial.println(smoothedFreq);
  set_neopixel (smoothedFreq);
  Serial.println("\n");
 
  delay(10);
}


