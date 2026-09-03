/*
 ARDUINO 1: 3-Axis Synchronized Motion Controller
 ------------------------------------------------
 Serial commands supported:
   ON
   OFF
   stepsX stepsY stepsZ timeMs


 Improvements vs old version:
   1. Shared ease-in / ease-out profile across all 3 axes
   2. Minimum pulse spacing per axis for stability
   3. Slightly wider step pulse
   4. Same exact text protocol as your Python code


 Notes:
   - CNC Shield enable pin: LOW = enabled, HIGH = disabled
   - If the requested move is too aggressive, this code will prioritize
     motor stability over perfect timing and may finish slightly later.
*/


#include <Arduino.h>
#include <string.h>
#include <stdio.h>


// -----------------------------
// Pin mapping
// -----------------------------
const int EN_PIN  = 8;


const int X_STEP = 2;
const int Y_STEP = 3;
const int Z_STEP = 4;


const int X_DIR  = 5;
const int Y_DIR  = 6;
const int Z_DIR  = 7;


// -----------------------------
// Serial receive buffer
// -----------------------------
const byte NUM_CHARS = 64;
char receivedChars[NUM_CHARS];
bool newData = false;


// -----------------------------
// Motion tuning
// -----------------------------
const unsigned int PULSE_WIDTH_US       = 3;    // was 2, wider is usually safer
const unsigned long MIN_STEP_INTERVAL_US = 220; // raise to 220-300 if still rough
const unsigned int IDLE_LOOP_DELAY_US   = 8;    // small CPU relief when waiting


// Optional debug printing
const bool DEBUG_MOTION = false;


// -----------------------------
// Helpers
// -----------------------------
float smoothStep(float u) {
 // Cubic ease-in/ease-out: 3u^2 - 2u^3
 // u expected in [0,1]
 if (u <= 0.0f) return 0.0f;
 if (u >= 1.0f) return 1.0f;
 return (u * u * (3.0f - 2.0f * u));
}


void pulsePin(int stepPin) {
 digitalWrite(stepPin, HIGH);
 delayMicroseconds(PULSE_WIDTH_US);
 digitalWrite(stepPin, LOW);
}


// Safely reads a line ending in '\n'
void recvWithEndMarker() {
 static byte ndx = 0;
 const char endMarker = '\n';
 char rc;


 while (Serial.available() > 0 && newData == false) {
   rc = Serial.read();


   if (rc != endMarker && rc != '\r') {
     receivedChars[ndx] = rc;
     ndx++;
     if (ndx >= NUM_CHARS) {
       ndx = NUM_CHARS - 1; // prevent overflow
     }
   } else if (rc == endMarker) {
     receivedChars[ndx] = '\0';
     ndx = 0;
     newData = true;
   }
 }
}


// -----------------------------
// New synchronized motion engine
// -----------------------------
void executeSynchronizedMotion(long sX, long sY, long sZ, long tMs) {
 // Set directions exactly as in your old code
 digitalWrite(X_DIR, (sX >= 0) ? LOW  : HIGH);
 digitalWrite(Y_DIR, (sY >= 0) ? HIGH : LOW);
 digitalWrite(Z_DIR, (sZ >= 0) ? HIGH : LOW);


 const long absX = labs(sX);
 const long absY = labs(sY);
 const long absZ = labs(sZ);


 if (absX == 0 && absY == 0 && absZ == 0) {
   return;
 }


 if (tMs <= 0) {
   return;
 }


 const unsigned long totalUs = (unsigned long)tMs * 1000UL;
 const unsigned long startUs = micros();


 // Actual step counters
 long countX = 0;
 long countY = 0;
 long countZ = 0;


 // Last pulse times per axis (for stability clamp)
 unsigned long lastPulseX = 0;
 unsigned long lastPulseY = 0;
 unsigned long lastPulseZ = 0;


 if (DEBUG_MOTION) {
   Serial.println(F("--- MOTION DEBUG ---"));
   Serial.print(F("Steps X/Y/Z: "));
   Serial.print(absX); Serial.print(F(" / "));
   Serial.print(absY); Serial.print(F(" / "));
   Serial.println(absZ);


   Serial.print(F("Requested time (ms): "));
   Serial.println(tMs);


   Serial.print(F("Requested rates steps/s -> X: "));
   Serial.print((absX > 0) ? (absX * 1000.0f / tMs) : 0.0f);
   Serial.print(F(" Y: "));
   Serial.print((absY > 0) ? (absY * 1000.0f / tMs) : 0.0f);
   Serial.print(F(" Z: "));
   Serial.println((absZ > 0) ? (absZ * 1000.0f / tMs) : 0.0f);
 }


 while (countX < absX || countY < absY || countZ < absZ) {
   unsigned long nowUs = micros();
   unsigned long elapsedUs = nowUs - startUs;


   // Motion progress in [0,1]
   float u;
   if (elapsedUs >= totalUs) {
     u = 1.0f;
   } else {
     u = (float)elapsedUs / (float)totalUs;
   }


   // Shared eased progress
   float p = smoothStep(u);


   // Target cumulative step counts at this point in time
   long targetX = (elapsedUs >= totalUs) ? absX : (long)(p * absX);
   long targetY = (elapsedUs >= totalUs) ? absY : (long)(p * absY);
   long targetZ = (elapsedUs >= totalUs) ? absZ : (long)(p * absZ);


   if (targetX > absX) targetX = absX;
   if (targetY > absY) targetY = absY;
   if (targetZ > absZ) targetZ = absZ;


   bool steppedSomething = false;
   nowUs = micros();


   // X axis
   if (countX < targetX) {
     if ((unsigned long)(nowUs - lastPulseX) >= MIN_STEP_INTERVAL_US) {
       pulsePin(X_STEP);
       lastPulseX = micros();
       countX++;
       steppedSomething = true;
     }
   }


   // Y axis
   nowUs = micros();
   if (countY < targetY) {
     if ((unsigned long)(nowUs - lastPulseY) >= MIN_STEP_INTERVAL_US) {
       pulsePin(Y_STEP);
       lastPulseY = micros();
       countY++;
       steppedSomething = true;
     }
   }


   // Z axis
   nowUs = micros();
   if (countZ < targetZ) {
     if ((unsigned long)(nowUs - lastPulseZ) >= MIN_STEP_INTERVAL_US) {
       pulsePin(Z_STEP);
       lastPulseZ = micros();
       countZ++;
       steppedSomething = true;
     }
   }


   // If nothing was due yet, give the CPU a tiny breather
   if (!steppedSomething) {
     delayMicroseconds(IDLE_LOOP_DELAY_US);
   }
 }


 if (DEBUG_MOTION) {
   unsigned long actualUs = micros() - startUs;
   Serial.print(F("Actual elapsed (ms): "));
   Serial.println(actualUs / 1000.0f);
   Serial.println(F("--------------------"));
 }
}


// -----------------------------
// Command parser
// -----------------------------
void parseDataAndExecute() {
 // Handle ON/OFF first
 if (strcmp(receivedChars, "OFF") == 0) {
   digitalWrite(EN_PIN, HIGH);   // CNC shield: HIGH disables
   Serial.println(F("!!! Motors DISENGAGED !!!"));
   return;
 }


 if (strcmp(receivedChars, "ON") == 0) {
   digitalWrite(EN_PIN, LOW);    // CNC shield: LOW enables
   Serial.println(F("Motors ENGAGED"));
   return;
 }


 // Handle movement command: "stepsX stepsY stepsZ timeMs"
 long stepsX = 0;
 long stepsY = 0;
 long stepsZ = 0;
 long timeMs = 0;


 int parsed = sscanf(receivedChars, "%ld %ld %ld %ld", &stepsX, &stepsY, &stepsZ, &timeMs);


 if (parsed == 4 && timeMs > 0) {
   // Always enable motors before motion
   digitalWrite(EN_PIN, LOW);


   executeSynchronizedMotion(stepsX, stepsY, stepsZ, timeMs);


   Serial.println(F("DONE"));
 } else {
   // Uncomment if you want debug feedback to Python:
   // Serial.println(F("ERROR_BAD_FORMAT"));
 }
}


// -----------------------------
// Arduino setup / loop
// -----------------------------
void setup() {
 Serial.begin(115200);


 pinMode(EN_PIN, OUTPUT);


 pinMode(X_STEP, OUTPUT);
 pinMode(Y_STEP, OUTPUT);
 pinMode(Z_STEP, OUTPUT);


 pinMode(X_DIR, OUTPUT);
 pinMode(Y_DIR, OUTPUT);
 pinMode(Z_DIR, OUTPUT);


 // Default state
 digitalWrite(EN_PIN, LOW);   // enabled
 digitalWrite(X_STEP, LOW);
 digitalWrite(Y_STEP, LOW);
 digitalWrite(Z_STEP, LOW);


 Serial.println(F("ARDUINO_1_READY"));
}


void loop() {
 recvWithEndMarker();


 if (newData == true) {
   parseDataAndExecute();
   newData = false;
 }
}

