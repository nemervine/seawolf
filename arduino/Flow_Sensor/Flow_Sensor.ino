const int pumpPin = 5;        // PWM output pin that the pump is attached to
const int FlowSensorPin = 2;  // Interuptable pin used for flow sensor reading
const int LED = 13;           // Pin tied to LED
const boolean DEBUG = false;
int pumpValue = 0;            // value output to the PWM (analog out)
int pumpState = LOW;
byte incomingByte;
byte pulseOut[3];

int flowPulses = 0;           // count pulses from flow sensors
int oldPulses = 0;

long timer1 = 0;
long timer2 = 0;
long timer3 = 0;
long timer4 = 0;
long interval1 = 1000;         // interval to send serial
long interval2 = 7500;          // interval to toggle pump
long OnTime;
long OffTime;
int timeOff = 0;
int timeOn = 0;

SIGNAL(TIMER1_COMPA_vect)  {                 // Interrupt is called once a millisecond, looks for any pulses from the sensor!
  uint8_t x = digitalRead(FlowSensorPin);
  if (x == HIGH) {
    //low to high transition!
    flowPulses++;
  }
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer1 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK1 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK1 &= ~_BV(OCIE0A);
  }
}

int handshakeSerial() {
  //Show handshake LED
  digitalWrite(LED, HIGH);
  /* Wait for handshake signal */
  while (true) {
    Serial.println("Waiting for handshake");
    //blocking
    while (Serial.available() == 0) {}

    //Treat 0xFE as a signal for Identity
    incomingByte = Serial.read();
    if (incomingByte == 0xFE) {
      Serial.println("{ID|Flowsensor}");
      delay(250);
      break;
    }
  }

  //turn off handshake LED
  digitalWrite(LED, LOW);

  return 1;
}

void setup() {
  Serial.begin(19200);                             // initialize serial communications at 9600 bps
  pinMode(FlowSensorPin, INPUT);                  // set mode for the flow sensor pin
  pinMode(pumpPin, OUTPUT);
  digitalWrite(FlowSensorPin, LOW);
  handshakeSerial();
  Serial.read();  // Read extra newline at end of handshake
}

void loop() {
  unsigned long currentTime = millis();
  incomingByte = 0;

  while ((currentTime - timer3) < interval2) {

    if (Serial.available() > 0) {
      incomingByte = Serial.read();
      if ((incomingByte >= 0x00) && (incomingByte <= 0xFF)) {
        pumpValue = incomingByte;
      }
    }

    OnTime = map(pumpValue, 0, 255, 0, interval2);

    currentTime = millis();
    if ((currentTime - timer3) <= OnTime) {
      pumpState = HIGH;
      digitalWrite(pumpPin, pumpState);
    }
    else {
      pumpState = LOW;
      digitalWrite(pumpPin, pumpState);
    }

    if ((pumpState == LOW) && (currentTime - timer4 >= 1)) {
      timeOff++;
      timer4 = currentTime;
    }
    else if ((pumpState == HIGH) && (currentTime - timer4 >= 1)) {
      timeOn++;
      timer4 = currentTime;
    }
  }

  timer3 = currentTime;
  if ((currentTime - timer1) > interval1) {
    useInterrupt(false);
    timer1 = currentTime;
    // print the results to the serial monitor:
    flowPulses = flowPulses - oldPulses;
    float pps = (float)flowPulses / ((float)timeOff + (float)timeOn) * 1000.0;
    int ppsint = int(pps);
    if (DEBUG) {
      Serial.print("PWM Out = ");
      Serial.println(pumpValue);
      Serial.print("Flow Pulses = ");
      Serial.println(flowPulses);
      Serial.print("Pulses/Second = ");
      Serial.println(pps, 0);
      Serial.print("Pulses/Second = ");
      Serial.println(ppsint);
      Serial.print("Pulses/Second = ");
      Serial.println(ppsint, HEX);
      Serial.print("On Time = ");
      Serial.println(timeOn);
      Serial.print("Off Time = ");
      Serial.println(timeOff);
      Serial.println();
    }
    pulseOut[0] = 0x01;
    pulseOut[1] = ppsint / 256;
    pulseOut[2] = ppsint % 256;
    Serial.write(pulseOut, 3);
    oldPulses = flowPulses;
    timeOn = 0;
    timeOff = 0;
    useInterrupt(true);
  }
}
