#include <string.h>

// ==========================================
// 1. PIN DEFINITIONS & CONSTANTS
// ==========================================
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8
#define dirA 12
#define dirB 13

#define SYNC_BYTE 0xAA   // The "Start" byte 
#define BAUD_RATE 115200 // Specified communication speed [cite: 5]

// ==========================================
// 2. GLOBAL STATE VARIABLES
// ==========================================
int current_delay = 5000;   // Motor speed in ms 
int isPulseTrain = 0;       // 0: Single pulse, 1: Pulse train [cite: 19]
int isForward = 1;          // 1: Forward, 0: Backward [cite: 25]

unsigned long lastStepTime = 0;

// ==========================================
// 3. SETUP
// ==========================================
void setup() {
  // Motor Shield Pin Modes
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);

  // Default Motor State: OFF
  digitalWrite(pwmA, LOW);
  digitalWrite(pwmB, LOW);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  // Start Serial Port
  Serial.begin(BAUD_RATE); // [cite: 5]
  
  // Visual guide for manual testing in Serial Monitor
  //Serial.println("--- SYSTEM READY ---");
  //erial.println("Commands: '1'=Start, '0'=Stop, 'F'=Forward, 'B'=Backward, 'S'=Status");
}


void dirAwayMotor() {
  digitalWrite(pwmA, HIGH); digitalWrite(pwmB, HIGH);
  digitalWrite(dirB, HIGH); digitalWrite(dirA, LOW);
  delayMicroseconds(5000);
  digitalWrite(dirB, LOW); digitalWrite(dirA, LOW);
  delayMicroseconds(5000);
  digitalWrite(dirB, LOW); digitalWrite(dirA, HIGH);
  delayMicroseconds(5000);
  digitalWrite(dirB, HIGH); digitalWrite(dirA, HIGH);
  delayMicroseconds(5000);
  digitalWrite(pwmA, LOW); digitalWrite(pwmB, LOW);
  //delay(current_delay);
}

void dirTowardMotor() {
  digitalWrite(pwmA, HIGH); digitalWrite(pwmB, HIGH);
  digitalWrite(dirB, LOW); digitalWrite(dirA, HIGH);
  delayMicroseconds(5000);
  digitalWrite(dirB, LOW); digitalWrite(dirA, LOW);
  delayMicroseconds(5000);
  digitalWrite(dirB, HIGH); digitalWrite(dirA, LOW);
  delayMicroseconds(5000);
  digitalWrite(dirB, HIGH); digitalWrite(dirA, HIGH);
  delayMicroseconds(5000);
  digitalWrite(pwmA, LOW); digitalWrite(pwmB, LOW);
  //delay(current_delay);
}

void executeMovement() {
  if (isForward == 1) {
    dirAwayMotor();
  } else {
    dirTowardMotor();
  }
}

void turnOffMotor() {
  digitalWrite(pwmA, LOW);
  digitalWrite(pwmB, LOW);
}
// ==========================================
// 4. MAIN EXECUTION LOOP
// ==========================================
void loop() {
  // Part A: Listen for Commands
  if (Serial.available() > 0) {
    if (Serial.peek() == SYNC_BYTE) {
      Serial.read(); // Remove Sync
      processIncomingMessage(); 
    } else {
      Serial.read(); // Clear junk
    }
  }

  // Part B: Act on State (Respects current_delay without blocking)
  if (isPulseTrain == 1) {
    if (millis() - lastStepTime >= current_delay) {
      executeMovement();
      lastStepTime = millis(); 
    }
  }
}

void processIncomingMessage() {
  while (Serial.available() < 2); 
  int header = Serial.read(); 
  int len = Serial.read();    

  // Handle the Message 4 discrepancy (Length 2 but 3 data bytes)
  int dataCount = (header == 0x04) ? 3 : len;
  int payload[dataCount];

  for (int i = 0; i < dataCount; i++) {
    while (Serial.available() == 0);
    payload[i] = Serial.read(); 
  }

  // Wait for the final trailing CRC byte
  while (Serial.available() == 0);
  int receivedCrc = Serial.read();

  // XOR Math: Sync ^ Header ^ Length ^ Data
  int calcXor = SYNC_BYTE ^ header ^ len;
  for (int i = 0; i < dataCount; i++) {
    calcXor ^= payload[i];
  }

  if (calcXor == receivedCrc) {
    handleCommand(header, payload);
  }
}

void handleCommand(int header, int data[]) {
  if (header == 0x01) {
    sendResponse(0xA1, 1); //
  }
  else if (header == 0x02) {
    isPulseTrain = (data[0] == 1); //
    sendResponse(0xA2, 1); 
  }
  else if (header == 0x03) {
    isForward = (data[0] == 0); //
    sendResponse(0xA3, 1); 
  }
  else if (header == 0x04) {
    current_delay = (data[1] << 8) | data[2]; //
    sendResponse(0xA4, 2); //
  }
}

void sendResponse(int header, int dataLen) {
  int packet[8]; 
  packet[0] = SYNC_BYTE;
  packet[1] = header;
  packet[2] = dataLen;
  
  int crc = SYNC_BYTE ^ header ^ dataLen;
  int payloadSize = 0;

  if (header == 0xA1) {
    int statusByte = 0;
    if (isPulseTrain == 1) statusByte |= 0x01;
    if (isForward == 0)    statusByte |= 0x02;
    packet[3] = statusByte;
    // Status response doesn't usually include speed bytes if length is 1
    payloadSize = 1;
  } 
  else if (header == 0xA2 || header == 0xA3) {
    packet[3] = (header == 0xA2) ? isPulseTrain : !isForward;
    payloadSize = 1;
  }
  else if (header == 0xA4) {
    packet[3] = 0xFF; 
    packet[4] = (current_delay >> 8); 
    packet[5] = (current_delay & 0xFF);
    payloadSize = 3;
  }

  // SEND FULL ENVELOPE: Sync, Header, Len, Data
  for (int i = 0; i < (3 + payloadSize); i++) {
    Serial.write(packet[i]);
    if (i > 2) crc ^= packet[i]; 
  }
  Serial.write(crc); // Trailing Checksum
}



