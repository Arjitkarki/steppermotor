/**
 * Linear Stage / Syringe Pump Controller - Stable Version
 * Resolution: 0.49609375 um per 1/8th step
 * Communication: 115200 Baud, XOR Checksum
 */

//DEFINE PINOUTS TO ARDUINO AND OVERALL CONSTS
#define ms1   5
#define ms2   6
#define step  7
#define dir   8
#define sleep 9
#define power 10
#define BAUDRATE 115200
#define SYNC_BYTE 0xAA
#define VOLT_PIN  A0


//GLOBALS  -- needed for the logic
int isPulseTrain = 0;       // 0: Stopped, 1: Running
int push = 1;               // 1: Infuse (Forward), 0: Refill (Reverse)
uint32_t delay_time = 3720; // Default: 4 mm/min in microseconds
uint16_t current_mm_min = 1;
const float LIMIT_PULL_VOLT = 2.73; // Threshold to switch to Push
const float LIMIT_PUSH_VOLT = 0.39; // Threshold to switch to Pull
uint16_t current_raw_speed = 1000; // Default 1.0 mm/min (1000)
int sleepState = 0; // 0 for Wake, 1 for Sleep
int powerState = 0; // 0 for Off, 1 for On


void setup() {
    Serial.begin(BAUDRATE);
    
    pinMode(ms1, OUTPUT);
    pinMode(ms2, OUTPUT);
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(sleep, OUTPUT);
    pinMode(power, OUTPUT);
    // Set 1/8th Microstepping
    digitalWrite(ms1, HIGH);
    digitalWrite(ms2, HIGH);

    digitalWrite(sleep, HIGH); // Wake the driver by default
    digitalWrite(power, HIGH); // Enable power to the motor
}

// SHOUDL SYNC TO POTENTIOMETER, HIGHER THE VOLTAGE MEANS CLOSER TO THE MOTOR
void checkVoltageLimits() {
    int raw = analogRead(VOLT_PIN);
    float currentVolt = (raw * 5.0) / 1023.0;

    // Logic Trigger 1: Reaches end of Pull -> Switch to Push
    if (push == 0 && currentVolt >= LIMIT_PULL_VOLT) {
        push = 1; 
    }
    // Logic Trigger 2: Reaches end of Push -> Switch to Pull
    else if (push == 1 && currentVolt <= LIMIT_PUSH_VOLT) {
        push = 0;
    }
}
void dirPull() {
    // Fast Retraction Logic
    digitalWrite(dir, HIGH);
    digitalWrite(step, HIGH);

     if (delay_time > 16000) {
        delay(delay_time / 1000); //REGULATOR FOR TOO SLOW/FAST TO AVOID BUZZING
        delayMicroseconds(delay_time % 1000);
    } else {
        delayMicroseconds(delay_time);
    }

    digitalWrite(step, LOW);

    if (delay_time > 16000) {
        delay(delay_time / 1000);
        delayMicroseconds(delay_time % 1000);
    } else {
        delayMicroseconds(delay_time);
    }
    //delayMicroseconds(200); 
    //digitalWrite(step, LOW);
    //delayMicroseconds(200);
}

void dirPush() {
    // Precise Infusion Logic
    digitalWrite(dir, LOW);
    digitalWrite(step, HIGH);
    
    // Hybrid Delay: Uses delay() for long periods (>16ms) to prevent timer overflow
    if (delay_time > 16000) {
        delay(delay_time / 1000);
        delayMicroseconds(delay_time % 1000);
    } else {
        delayMicroseconds(delay_time);
    }

    digitalWrite(step, LOW);

    if (delay_time > 16000) {
        delay(delay_time / 1000);
        delayMicroseconds(delay_time % 1000);
    } else {
        delayMicroseconds(delay_time);
    }
}

void executeMovement() {
  checkVoltageLimits(); //BIG CHECKER
    if (push == 1) {
        dirPush();
    } else {
        dirPull();
    }
}

// --- Communication Loop ---

void loop() {

    if (Serial.available() > 0) {
        if (Serial.peek() == SYNC_BYTE) {
            Serial.read(); // Clear Sync
            processIncomingMessage();
        } else {
            Serial.read(); // Clear Junk
        }
    }

    if (isPulseTrain == 1) { //will only run on pulse train command
        executeMovement();
    }
}

void processIncomingMessage() {
    while (Serial.available() < 2); // Wait for Header and Length
    int header = Serial.read();
    int len = Serial.read();

    // Guard against buffer overflow
    if (len > 8) {
        while (Serial.available() > 0) Serial.read();
        return;
    }

    int payload[10];
    payload[0] = SYNC_BYTE;
    payload[1] = header;
    payload[2] = len;

    for (int i = 3; i < (len + 3); i++) {
        while (Serial.available() == 0);
        payload[i] = Serial.read();
    }
    
    while (Serial.available() == 0);
    int receivedCrc = Serial.read();

    int calcXor = SYNC_BYTE ^ header ^ len;
    for (int i = 3; i < (len + 3); i++) {
        calcXor ^= payload[i];
    }

    if (calcXor == receivedCrc) {
        handleCommand(header, payload);
    }
}

// --- Command Handlers ---

void handleCommand(int header, int payload[]) {
    if (header == 0x01) { //STATUS

        sendResponse(0xA1, 5, 0);
    }
    else if (header == 0x02) { // 0x02 == SET DIRECTION
    // DOCUMENTATION MATCH: Bit 0 is Direction (0: Pull, 1: Push)
        if ((payload[3] & 0x01) == 1) {
            push = 1; // Push (Away from motor)
        }   
        else {
            push = 0; // Pull (Towards motor)
        }

    sendResponse(0xA2, 1, 0); 
    }
    else if (header == 0x03) {
        // Speed Control (mm/min)
        uint16_t raw_input = ((uint16_t)payload[3] << 8) | payload[4];
        current_raw_speed = raw_input;

        if (raw_input == 0) {
          isPulseTrain = 0;
        }

        else { 
            float speed_mm_min =  (float)raw_input / 1000;
            float microns_per_sec = (speed_mm_min / 60.0) * 1000.0;
            float steps_per_sec = microns_per_sec / 0.49609375;
            float total_period_us = 1000000.0 / steps_per_sec;
            delay_time = (uint32_t)(total_period_us / 2.0);
        }
        sendResponse(0xA3, 2, current_raw_speed);
    }
    else if (header == 0x04) { //start/stop control
        if ( (payload[3] & 0x01) == 1) {
            isPulseTrain = 1;
        }
        else {
            isPulseTrain = 0;
        }
        sendResponse(0xA4, 1, 0);
    }

    else if (header == 0x05) { //SLEEP/POWER CONTROL
        if (payload[3] & 0x01) {
            digitalWrite(sleep, HIGH); // Sleep
            sleepState = 1;
        } else {
            digitalWrite(sleep, LOW);  // Wake
            sleepState = 0;
        }
    // Check Bit 1 (Power): 0 = Off, 1 = On
        if (payload[3] & 0x02) {
            digitalWrite(power, HIGH); // On
            powerState = 1;
        } else {
            digitalWrite(power, LOW);  // Off
            powerState = 0;
         }
        sendResponse(0xA5, 1, 0);
    }
        
}

void sendResponse(int header, int dataLen, int speed) { //SEND RESPONSE BACK
    uint8_t packet[12]; 
    packet[0] = SYNC_BYTE;
    packet[1] = header;
    packet[2] = dataLen;
    int crc = SYNC_BYTE ^ header ^ dataLen;

    if (header == 0xA1) {
        uint8_t status = 0;
        if (push == 1) status |= (1 << 0); 
        
        // Bit 4.1: Started/Stopped (0: Stopped, 1: Started)
        if (isPulseTrain == 1) status |= (1 << 1); 
        
        // Bit 4.6: SparkFun Sleep (0: Awake, 1: Asleep)
        if (sleepState == 1) status |= (1 << 6); 
        
        // Bit 4.7: SparkFun Power (0: Off, 1: On)
        if (powerState == 1) status |= (1 << 7);

        packet[3] = status;
        packet[4] = (current_raw_speed >> 8) & 0xFF;
        packet[5] = current_raw_speed & 0xFF;
        
        int raw = analogRead(VOLT_PIN);
        float liveVolt = (raw * 5.0) / 1023.0;
        uint16_t voltToSend = (uint16_t)(liveVolt * 100.0); // e.g., 2.73V -> 273
        packet[6] = (voltToSend >> 8) & 0xFF;
        packet[7] = voltToSend & 0xFF;

        for (int i = 0; i < 8; i++) {
            Serial.write(packet[i]);
            if (i >= 3) crc ^= packet[i];
        }
    }
    else if (header == 0xA2) { //[PUSH/PULL]
        packet[3] = 0xFF; // Status OK
        packet[4] = (push == 1) ? 0x01 : 0x00;
        for (int i = 0; i < 5; i++) {
            Serial.write(packet[i]);
            if (i >= 3) crc ^= packet[i];
        }
    }
    else if (header == 0xA3) { //SPEED
        packet[3] = (speed >> 8) & 0xFF;
        packet[4] = speed & 0xFF;
        for (int i = 0; i < 5; i++) {
            Serial.write(packet[i]);
            if (i >= 3) crc ^= packet[i];
        }
    }
    else if (header == 0xA4) { //START/STOP
        if (isPulseTrain == 1) {
            packet[3] = 1;
        }
        else {
            packet[3] = 0;
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(packet[i]);
            if (i >= 3) crc ^= packet[i];
        }
    }
    else if (header == 0xA5) { //POWER/SLEEP
       uint8_t status = 0;
        if (sleepState == 1) status |= 0x01; // 4.1 Sleep bit
        if (powerState == 1) status |= 0x02; // 4.2 Power bit
        packet[3] = status;
        for (int i = 0; i < 4; i++) {
            Serial.write(packet[i]);
            if (i >= 3) crc ^= packet[i];
        }
    }

    Serial.write(crc);
}
