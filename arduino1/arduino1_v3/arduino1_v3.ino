/* CNC Shield Hardware Integration (arduino1.ino)
String Parsing: Upgraded the 3-axis code to intercept raw text commands (strcmp) alongside its standard numeric step/time commands.
Enable Pin Setup: Mapped the global CNC Shield enable pin to Pin 8.
Safety Logic: Configured Pin 8 to pull HIGH to disable the upper-arm motors (allowing free spin) and pull LOW to enable and lock them.
—---------------------------------------------- */


// ARDUINO 1: 3-Axis Time-Slice Executor
const int EN_PIN = 8;
const int X_STEP = 2; const int X_DIR = 5; // Motor 1
const int Y_STEP = 3; const int Y_DIR = 6; // Motor 2
const int Z_STEP = 4; const int Z_DIR = 7; // Motor 3

// --- Serial Buffer Variables ---
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT); 
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable motors (CNC Shield: LOW = Enabled)
  Serial.println("ARDUINO_1_READY");
}

void loop() {
  // 1. Constantly check for new data without blocking
  recvWithEndMarker();
  
  // 2. If a full line was received, parse and execute
  if (newData == true) {
    parseDataAndExecute();
    newData = false;
  }
}

// Safely reads characters until it hits a newline (\n)
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker && rc != '\r') {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1; // Prevent buffer overflow
      }
    } 
    else if (rc == endMarker) {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void parseDataAndExecute() {
  // --- NEW: Handle ON/OFF string commands ---
  if (strcmp(receivedChars, "OFF") == 0) {
    digitalWrite(EN_PIN, HIGH); // CNC Shield: HIGH disables motors
    Serial.println("!!! Motors DISENGAGED !!!");
    return;
  }
  if (strcmp(receivedChars, "ON") == 0) {
    digitalWrite(EN_PIN, LOW); // CNC Shield: LOW enables motors
    Serial.println("Motors ENGAGED");
    return;
  }

  // --- Handle movement commands ---
  long stepsX = 0, stepsY = 0, stepsZ = 0, timeMs = 0;
  
  // sscanf neatly extracts the 4 long integers from our string
  int parsed = sscanf(receivedChars, "%ld %ld %ld %ld", &stepsX, &stepsY, &stepsZ, &timeMs);

  // Only execute if it successfully found exactly 4 numbers AND time > 0
  if (parsed == 4 && timeMs > 0) {
    executeSynchronizedMotion(stepsX, stepsY, stepsZ, timeMs);
    Serial.println("DONE");
  } else {
    // Optional: Let the Pi know the command was corrupted
    // Serial.println("ERROR_BAD_FORMAT"); 
  }
}

void executeSynchronizedMotion(long sX, long sY, long sZ, long tMs) {
  // Set directions
  digitalWrite(X_DIR, (sX >= 0) ? LOW : HIGH);
  digitalWrite(Y_DIR, (sY >= 0) ? HIGH : LOW);
  digitalWrite(Z_DIR, (sZ >= 0) ? HIGH : LOW);

  long absX = abs(sX), absY = abs(sY), absZ = abs(sZ);
  
  // Calculate microsecond intervals
  unsigned long totalUs = tMs * 1000UL;
  unsigned long intX = (absX > 0) ? (totalUs / absX) : 0;
  unsigned long intY = (absY > 0) ? (totalUs / absY) : 0;
  unsigned long intZ = (absZ > 0) ? (totalUs / absZ) : 0;

  unsigned long startUs = micros();
  unsigned long lastX = startUs, lastY = startUs, lastZ = startUs;
  long countX = 0, countY = 0, countZ = 0;

  while (countX < absX || countY < absY || countZ < absZ) {
    unsigned long now = micros();
    
    if (absX > 0 && countX < absX && (now - lastX >= intX)) {
      digitalWrite(X_STEP, HIGH); delayMicroseconds(2); digitalWrite(X_STEP, LOW);
      lastX = now; countX++;
    }
    if (absY > 0 && countY < absY && (now - lastY >= intY)) {
      digitalWrite(Y_STEP, HIGH); delayMicroseconds(2); digitalWrite(Y_STEP, LOW);
      lastY = now; countY++;
    }
    if (absZ > 0 && countZ < absZ && (now - lastZ >= intZ)) {
      digitalWrite(Z_STEP, HIGH); delayMicroseconds(2); digitalWrite(Z_STEP, LOW);
      lastZ = now; countZ++;
    }
  }
}


