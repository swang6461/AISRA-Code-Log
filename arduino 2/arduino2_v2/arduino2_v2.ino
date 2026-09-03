Base Motor Alignment (arduino2.ino - Earlier Update)
Logic Correction: Aligned the ENA pin logic to match your driver's Common Anode setup, ensuring the NEMA 34 correctly locks on LOW and releases on HIGH.
Handshake Protocol: Ensured it reliably sends the "ARDUINO_2_READY" and "DONE" flags to keep the Pi's master sequencer from sending commands while the arm is moving.


/*
CHANGED: GR from 1.6 -> 1 + HIGH:LOW logic to reflect real conditions 
* STABILIZED NEMA 34 + HBS860H (ARDUINO 2 - BASE LINK)
* Logic: Common Anode
* ENA Logic: Pin 5 LOW = OFF, Pin 5 HIGH = ON
*/


#define PUL 3
#define DIR 4
#define ENA 5


const long motorStepsPerRevolution = 6400;
const float gearRatio = 1;


const unsigned int pulseWidthMicros = 20;
const unsigned int dirSetupMicros = 100;


struct Move {
 float angle;
 long speed;
 bool cw;
};


Move moveList[10];
int moveCount = 0;
bool looping = false;
Move lastMove = {0, 0, true};
String inputBuffer = "";


void setup() {
 // CRITICAL FIX 1: Match the Raspberry Pi's scanning speed
 Serial.begin(115200);
  pinMode(PUL, OUTPUT);
 pinMode(DIR, OUTPUT);
 pinMode(ENA, OUTPUT);


 // STARTUP CONFIGURATION:
 // In Common Anode, HIGH means 'No Current'
 // For ENA-, 'No Current' = Motor ENABLED
 digitalWrite(ENA, LOW);
 digitalWrite(PUL, HIGH);
 digitalWrite(DIR, HIGH);


 // CRITICAL FIX 2: Exact string the Python script is looking for
 Serial.println("ARDUINO_2_READY");
 Serial.println("Motor is currently: ON (Locked)");
 Serial.println("Type 'OFF' to release motor, 'ON' to lock.");
}


void loop() {
 if (Serial.available() > 0) {
   char c = Serial.read();
   if (c == '\n' || c == '\r') {
     if (inputBuffer.length() > 0) {
       processCommand(inputBuffer);
       inputBuffer = "";
     }
   } else {
     inputBuffer += c;
   }
 }


 if (looping && moveCount > 0) {
   for (int i = 0; i < moveCount; i++) {
     if (Serial.available() > 0) { looping = false; break; }
     moveWithAcceleration(moveList[i].angle, moveList[i].speed, moveList[i].cw);
     delay(800);
   }
 }
}


void processCommand(String cmd) {
 cmd.trim();
 cmd.toUpperCase();


 if (cmd == "OFF") {
   digitalWrite(ENA, HIGH); // Current flows -> Optocoupler triggers 'Disable'
   Serial.println("!!! Motor DISENGAGED (Free Spin) !!!");
   return;
 }


 if (cmd == "ON") {
   digitalWrite(ENA, LOW); // No current -> Driver defaults to 'Enable'
   Serial.println("Motor ENGAGED (Locked)");
   return;
 }


 if (cmd == "STOP") { looping = false; return; }
 if (cmd == "SET") {
   if (moveCount < 10) {
     moveList[moveCount] = lastMove;
     moveCount++;
     Serial.println("Step saved.");
   }
   return;
 }
 if (cmd == "LOOP") { looping = true; digitalWrite(ENA, LOW); return; }
 if (cmd == "CLEAR") { moveCount = 0; looping = false; Serial.println("Cleared."); return; }


 // Manual Movement Pipeline (Triggered by Python)
 cmd.replace(",", " ");
 int firstSpace = cmd.indexOf(' ');
 int secondSpace = cmd.indexOf(' ', firstSpace + 1);


 if (firstSpace != -1 && secondSpace != -1) {
   float angle = cmd.substring(0, firstSpace).toFloat();
   long speed = cmd.substring(firstSpace + 1, secondSpace).toInt();
   bool isCw = (cmd.substring(secondSpace + 1) == "CW");


   lastMove = {angle, speed, isCw};
 
   // Ensure motor is ON before attempting to move
   digitalWrite(ENA, LOW);
   moveWithAcceleration(angle, speed, isCw);
  
   // Tell the Pi we are done
   Serial.println("DONE");
 }
}


void moveWithAcceleration(float outputAngleDeg, long maxStepsPerSec, bool cw) {
 float motorRevs = (outputAngleDeg / 360.0) * gearRatio;
 long totalSteps = lround(motorRevs * motorStepsPerRevolution);
 long rampSteps = totalSteps / 4;
 if (rampSteps > 1500) rampSteps = 1500;
  digitalWrite(DIR, cw ? LOW : HIGH); // Adjusted for sinking logic
 delayMicroseconds(dirSetupMicros);


 for (long i = 0; i < totalSteps; i++) {
   long currentSpeed;


   if (i < rampSteps) {
     currentSpeed = map(i, 0, rampSteps, 100, maxStepsPerSec);
   } else if (i > (totalSteps - rampSteps)) {
     currentSpeed = map(i, totalSteps - rampSteps, totalSteps, maxStepsPerSec, 100);
   } else {
     currentSpeed = maxStepsPerSec;
   }


   if (currentSpeed < 50) currentSpeed = 50;


   unsigned long stepPeriod = 1000000UL / currentSpeed;


   // Pulse: Falling Edge Trigger (Matches Code 1)
   digitalWrite(PUL, HIGH);
   delayMicroseconds(pulseWidthMicros);
   digitalWrite(PUL, LOW);
 
   long waitTime = stepPeriod - pulseWidthMicros;
   if (waitTime < 25) waitTime = 25;
   delayMicroseconds(waitTime);
 }
}

