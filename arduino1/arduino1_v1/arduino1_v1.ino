// ARDUINO 1: 3-Axis Time-Slice Executor
const int EN_PIN = 8;
const int X_STEP = 2; const int X_DIR = 5; // Motor 1
const int Y_STEP = 3; const int Y_DIR = 6; // Motor 2
const int Z_STEP = 4; const int Z_DIR = 7; // Motor 3

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT); 
  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable motors
  Serial.println("ARDUINO_1_READY");
}

void loop() {
  if (Serial.available()) {
    // Read format: <StepsX StepsY StepsZ TimeMs>
    // Negative steps mean reverse direction.
    long stepsX = Serial.parseInt();
    long stepsY = Serial.parseInt();
    long stepsZ = Serial.parseInt();
    long timeMs = Serial.parseInt();
    
    // Clear buffer
    while (Serial.available()) Serial.read();

    if (timeMs > 0) {
      executeSynchronizedMotion(stepsX, stepsY, stepsZ, timeMs);
      Serial.println("DONE");
    }
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

