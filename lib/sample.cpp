
#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include <arduinoFFT.h>
#include <math.h>

// Indicators
#define RED 0xFF0000 // Dyskenesia
#define GREEN 0x00FF00 // Normal
#define YELLOW 0xFFFF00 // Parkinsons
#define ORANGE 0xFFA500 // Between Dyskenesia and Parkinsons
#define PARKINSON_LOW 3.0
#define PARKINSON_HIGH 5.0
#define DYSKINESIA_LOW 5.0
// #define DYSKINESIA_HIGH 7.0

// FFT
#define SAMPLES 128
#define SAMPLING_FREQUENCY 45 

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// State machine
enum State {
  RECORDING,
  ANALYZING,
  WAITING
};

// Variables
State currentState = RECORDING;
unsigned long lastSampleTime = 0;
int sampleIndex = 0;
unsigned long waitStartTime = 0;
const long waitDuration = 2000; // Wait 2 seconds between analyses

float peakFrequency = 0;
float peakMagnitude = 0;
int intensity = 0; // 0-5 intensity scale

void analyzeData();

void setup() {
  CircuitPlayground.begin(); // Initialize circuit playground

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}


void loop() {
  unsigned long currentMillis = millis();
  
  switch (currentState) {
    case RECORDING:
      // Time to collect the next sample?
      if (currentMillis - lastSampleTime >= (1000 / SAMPLING_FREQUENCY)) { // ensures 3 sec window for sampling 
        lastSampleTime = currentMillis;
        
        // Read accelerometer data
        float x = CircuitPlayground.motionX();
        float y = CircuitPlayground.motionY();
        float z = CircuitPlayground.motionZ();
        
        // Calculate the magnitude of the acceleration vector
        float magnitude = sqrt((x * x) + (y * y) + (z * z));
        magnitude -= 9.8 ; // Remove gravity component
        
        // Store the sample
        vReal[sampleIndex] = magnitude;
        vImag[sampleIndex] = 0.0; // Imaginary part must be zeroed
        
        // Debug output (optional)
        if (sampleIndex % 10 == 0) { // every 10 samples
          Serial.print("Sample ");
          Serial.print(sampleIndex);
          Serial.print(": ");
          Serial.println(magnitude);
        }
        
        sampleIndex++;
        
        // Samples are full, switch to analyzing
        if (sampleIndex >= SAMPLES) {
          Serial.println("Recording complete. Starting analysis...");
          currentState = ANALYZING;
        }
      }
      break;
      
    case ANALYZING:
      analyzeData();
      waitStartTime = currentMillis;
      currentState = WAITING;
      break;
      
    case WAITING:
      // Return to recording after wait duration
      if (currentMillis - waitStartTime >= waitDuration) {
        // Reset for next recording
        sampleIndex = 0;
        Serial.println("Starting new recording cycle...");
        currentState = RECORDING;
      }
      break;
  }
}

void analyzeData() {
  Serial.println("Performing FFT analysis...");
  
  // Prepare the FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.dcRemoval();
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  
  peakFrequency = 0;
  peakMagnitude = 0;

  float parkinsonPeakMag = 0;
  float parkinsonPeakFreq = 0;
  float dyskinesiaPeakMag = 0;
  float dyskinesiaPeakFreq = 0;
  
  // Print FFT results for debugging
  Serial.println("Frequency\tMagnitude");
  
  for (int i = 0; i < SAMPLES/2; i++) {
    float frequency = (i * SAMPLING_FREQUENCY) / SAMPLES;
    
    // removes the DC component and low frequencies
    if (frequency < 2) continue;

    // every 5th sample (for debugging)
    if (i % 5 ==0) { 
      Serial.print(frequency);
      Serial.print("Hz\t\t");
      Serial.println(vReal[i]);
    }
    
    if (vReal[i] > peakMagnitude) {
      peakMagnitude = vReal[i];
      peakFrequency = frequency;
    }
    
    // Track peaks in our specific ranges of interest
    if (frequency >= PARKINSON_LOW && frequency < PARKINSON_HIGH) {
      if (vReal[i] > parkinsonPeakMag) {
        parkinsonPeakMag = vReal[i];
        parkinsonPeakFreq = frequency;
      }
    } 
    else if (frequency >= DYSKINESIA_LOW) {  
      if (vReal[i] > dyskinesiaPeakMag) {
        dyskinesiaPeakMag = vReal[i];
        dyskinesiaPeakFreq = frequency;
      }
    }
  }

  float detectionThreshold = 1.5;
  float relevantFrequency = 0;
  
  // Determine condition based on peak frequencies and magnitudes
  if (parkinsonPeakMag > detectionThreshold && parkinsonPeakMag > dyskinesiaPeakMag) {
    Serial.println("*** PARKINSON DETECTED ***");
    Serial.print("Parkinson peak at: ");
    Serial.print(parkinsonPeakFreq);
    Serial.print(" Hz with magnitude: ");
    Serial.println(parkinsonPeakMag);

    relevantFrequency = parkinsonPeakFreq;
  } 
  else if (dyskinesiaPeakMag > detectionThreshold) {
    Serial.println("*** DYSKINESIA DETECTED ***");
    Serial.print("Dyskinesia peak at: ");
    Serial.print(dyskinesiaPeakFreq);
    Serial.print(" Hz with magnitude: ");
    Serial.println(dyskinesiaPeakMag);

    relevantFrequency = dyskinesiaPeakFreq;
  } 
  else {
    Serial.println("*** NO MOVEMENT DISORDER DETECTED ***");

    relevantFrequency = 0;
  }
  
  // Calculate intensity based on frequency (1-6 scale)
  if (relevantFrequency >= 3.0) {
    // Linear mapping: 3Hz->1, 4Hz->2, 5Hz->3, 6Hz->4, 7Hz->5, >7Hz->6
    intensity = min(6, 1 + (int)((relevantFrequency - 3.0) + 0.5)); // +0.5 for proper rounding
    Serial.print("Intensity level: ");
    Serial.print(intensity);
    Serial.print(" (based on ");
    Serial.print(relevantFrequency);
    Serial.println(" Hz)");
  } else {
    intensity = 0;
    Serial.println("Intensity level: 0 (no relevant movement)");
  }
  
  // Summary output
  Serial.print("Overall Peak Frequency: ");
  Serial.print(peakFrequency);
  Serial.print(" Hz, Magnitude: ");
  Serial.println(peakMagnitude);
  Serial.println("-----------------------------");
}
