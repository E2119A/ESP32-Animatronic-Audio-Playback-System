// Include libraries
#include "AudioTools.h"          // Audio processing (I2S)
#include "BluetoothA2DPSink.h"   // Bluetooth audio reception (A2DP)
#include "arduinoFFT.h"          // Fast Fourier Transform (FFT)

// ------------------------
// Project constants
// ------------------------

// Motor control pins
const int motorPin1 = 19;               // Head forward / Tail backward
const int motorPin2 = 21;
const int motorPinMouthOpen = 5;        // Motor opens mouth
const int motorPinMouthClose = 18;      // Motor closes mouth

// Movement time intervals (in milliseconds)
const int HEAD_FORWARD_TIME = 500;      // Head extension duration
const int HEAD_RETURN_TIME = 350;       // Head return duration
const int TAIL_FORWARD_TIME = 150;      // Tail hit duration
const int TAIL_RETURN_TIME = 150;       // Tail return duration
const int MOUTH_OPEN_TIME = 100;        // Mouth open duration
const int MOUTH_CLOSE_TIME = 100;       // Mouth closed duration

// Minimum time between two tail hits
const unsigned long MIN_TAIL_HIT_INTERVAL = 250;

// FFT settings
const uint16_t SAMPLES = 256;           // Sample size (must be a power of 2)
const double SAMPLING_FREQUENCY = 44100; // Audio sampling rate (Hz)

// ------------------------
// Global variables
// ------------------------

// Buffers to store audio data as complex numbers
double vReal[SAMPLES];                  // Real part of the signal
double vImag[SAMPLES];                  // Imaginary part of the signal

// FFT object
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// I2S stream for receiving audio
I2SStream i2s;

// Bluetooth receiver
BluetoothA2DPSink a2dp_sink(i2s);

// Current index in the audio sample
int sampleIndex = 0;

// Tail logic variables
unsigned long lastTailMoveTime = 0;     // Last tail trigger time
bool tailMoving = false;                // Tail movement flag

// Mouth logic variables
unsigned long lastMouthMoveTime = 0;    // Last mouth open time
bool mouthOpen = false;                 // Flag — is the mouth open

// Bass smoothing variables
double smoothedLogAvgBass = 0.0;
const double SMOOTHING_FACTOR = 0.2;     // Smoothing factor

// Bass sensitivity threshold (in logarithmic scale)
const double BASS_LOG_THRESHOLD = 4.5;   // Roughly corresponds to 110000 in linear scale

// ------------------------
// Initialization
// ------------------------

void setup() {
  Serial.begin(115200); // Start serial port for debugging

  // Set motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPinMouthOpen, OUTPUT);
  pinMode(motorPinMouthClose, OUTPUT);

  // Set all motors to initial off state
  digitalWrite(motorPinMouthOpen, LOW);
  digitalWrite(motorPinMouthClose, LOW);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  // Stop all motors
  stopTailMotor();
  stopMouthMotor();

  // I2S pin configuration
  auto cfg = i2s.defaultConfig();
  cfg.pin_bck = 26;   // Bit Clock (BCK)
  cfg.pin_ws = 25;    // Word Select (LRCK)
  cfg.pin_data = 22;  // Data

  i2s.begin(cfg);     // Initialize I2S with specified configuration

  // Bluetooth setup — set callback and start
  a2dp_sink.set_stream_reader(audioDataCallback); // Set audio stream handler
  a2dp_sink.start("Billy Bass");                 // Bluetooth device name
  Serial.println("Bluetooth ready!");
}

// ------------------------
// Main loop
// ------------------------

void loop() {
  unsigned long now = millis(); // Current time

  // Check if it's time to stop tail
  if (tailMoving && now - lastTailMoveTime >= TAIL_FORWARD_TIME) {
    stopTailMotor();           // Stop tail
    tailMoving = false;        // Reset flag
    lastTailMoveTime = now;    // Update time
    Serial.println("Tail stopped, waiting return...");
  }

  // Check if it's time to close mouth
  if (mouthOpen && now - lastMouthMoveTime >= MOUTH_OPEN_TIME) {
    closeMouth();              // Command to close mouth
    stopMouthMotor();          // Stop motor
    mouthOpen = false;         // Reset flag
    lastMouthMoveTime = now;   // Update time
    Serial.println("Mouth closing...");
  }
}

// ------------------------
// Tail control
// ------------------------

// Command to move tail (backward)
void moveTailForward() {
  if (tailMoving) {                         // Ignore if tail already moving
    Serial.println("Tail busy, ignoring hit");
    return;
  }
  digitalWrite(motorPin1, LOW);            // One pin LOW
  digitalWrite(motorPin2, HIGH);           // Other pin HIGH — causes rotation
  tailMoving = true;                       // Set flag
  lastTailMoveTime = millis();             // Record time
  Serial.println("Tail moving backward (hit)");
}

// Stop tail motor
void stopTailMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  Serial.println("Tail motor stopped");
}

// ------------------------
// Mouth control
// ------------------------

// Command to open mouth
void openMouth() {
  digitalWrite(motorPinMouthOpen, LOW);   // One pin LOW
  digitalWrite(motorPinMouthClose, HIGH); // Other HIGH — mouth opens
  mouthOpen = true;                       // Set flag
  lastMouthMoveTime = millis();           // Record time
  Serial.println("Mouth opening");
}

// Command to close mouth
void closeMouth() {
  digitalWrite(motorPinMouthOpen, HIGH);  // Reverse direction — mouth closes
  digitalWrite(motorPinMouthClose, LOW);
  Serial.println("Mouth closing");
}

// Stop mouth motor
void stopMouthMotor() {
  digitalWrite(motorPinMouthOpen, LOW);   // Fully stop motor
  digitalWrite(motorPinMouthClose, LOW);
  Serial.println("Mouth motor stopped");
}

// ------------------------
// Audio stream processing
// ------------------------

// Function called when new audio data arrives
void audioDataCallback(const uint8_t *data, uint32_t length) {
  const int16_t *samples = (const int16_t *)data; // Convert bytes to 16-bit samples
  int numSamples = length / 2;                    // Number of samples in buffer

  // Store audio samples into array for FFT
  for (int i = 0; i < numSamples && sampleIndex < SAMPLES; i++) {
    vReal[sampleIndex] = (double)samples[i]; // Store amplitude
    vImag[sampleIndex] = 0.0;                 // Imaginary part = 0
    sampleIndex++;
  }

  // If enough samples are collected — run FFT
  if (sampleIndex >= SAMPLES) {
    sampleIndex = 0;

    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Apply Hamming window
    FFT.compute(FFT_FORWARD);                        // Compute FFT
    FFT.complexToMagnitude();                        // Compute magnitudes

    // -------- Bass analysis --------

    // Indices for 0–150 Hz
    int bassIndexStart = 1;
    int bassIndexEnd = (int)ceil((150.0 * SAMPLES) / SAMPLING_FREQUENCY);
    if (bassIndexEnd < bassIndexStart) bassIndexEnd = bassIndexStart;

    double bassSum = 0;
    double bassMax = 0;

    // Calculate average and max values in bass range
    for (int i = bassIndexStart; i <= bassIndexEnd; i++) {
      bassSum += vReal[i];
      if (vReal[i] > bassMax) bassMax = vReal[i];
    }

    double avgBass = bassSum / (bassIndexEnd - bassIndexStart + 1);
    double logAvgBass = log10(avgBass + 1);     // Logarithmic scale
    double logBassMax = log10(bassMax + 1);

    // Smooth values
    smoothedLogAvgBass = (SMOOTHING_FACTOR * logAvgBass) + ((1 - SMOOTHING_FACTOR) * smoothedLogAvgBass);

    // Debug output to serial
    Serial.print("logAvgBass: "); Serial.print(logAvgBass, 2);
    Serial.print(" | smoothedLogAvgBass: "); Serial.print(smoothedLogAvgBass, 2);
    Serial.print(" | logBassMax: "); Serial.println(logBassMax, 2);

    // Condition for tail hit
    if ((smoothedLogAvgBass > BASS_LOG_THRESHOLD || logBassMax > BASS_LOG_THRESHOLD) &&
        (millis() - lastTailMoveTime > MIN_TAIL_HIT_INTERVAL)) {
      moveTailForward(); // Command tail hit
    }

    // -------- Vocal analysis --------

    double vocalStartFreq = 300;
    double vocalEndFreq = 1500;
    int vocalStartIndex = (int)ceil((vocalStartFreq * SAMPLES) / SAMPLING_FREQUENCY);
    int vocalEndIndex = (int)ceil((vocalEndFreq * SAMPLES) / SAMPLING_FREQUENCY);
    if (vocalEndIndex < vocalStartIndex) vocalEndIndex = vocalStartIndex;

    double vocalSum = 0;
    for (int i = vocalStartIndex; i <= vocalEndIndex; i++) {
      vocalSum += vReal[i];
    }

    double avgVocal = vocalSum / (vocalEndIndex - vocalStartIndex + 1);

    // Threshold to open mouth when vocal is present
    const double MOUTH_THRESHOLD = 15000;

    if (avgVocal > MOUTH_THRESHOLD && !mouthOpen) {
      openMouth(); // Command to open mouth
    }
  }
}
