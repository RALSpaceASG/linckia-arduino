//feedback
int mfeedon;

void AnalogPinSetup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  pinMode(2,OUTPUT);
  digitalWrite(2, HIGH);
}

void ReadAnalog(uint8_t targetPin) {
  int feedback = analogRead(targetPin);
  byte feedback100 = feedback/100;
  byte feedback10  = feedback - feedback100*100;
  if (feedback100 < 100) {
   if (feedback10 < 100) {
     ReturnPacket(1,targetPin,feedback100,feedback10);
   }
 }
}

void SensorInterval(byte command[6]) {
  //TODO Start stop readout
  if (command[1] == 1) { //command for changeg feedback interval
    mfeedon = command[2]; //turn it on if 1 turn it off if 0
    //command[3] not used
    ulSensorsInterval = (command[4]*100)+command[5];
    MSensors.interval(ulSensorsInterval);
    MSensors.reset();
  }
}

void CheckSensors() {
  if(mfeedon == 1) {
    static int j;
    ReadAnalog(j);
    j = j + 1;
    if(j == 5) { j = 0; }
  }
}
