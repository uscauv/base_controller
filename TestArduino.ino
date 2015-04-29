//Arduino Test Code
//Echos with "Hi: I Read "
//USC AUV
//Michael Kukar 2015
//mkukar@gmail.com

int recievedByte1 = 0;
int recievedByte2 = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Ready.");
  Serial.println("Please enter an integer to parse.");
}

void loop() {
  if (Serial.available() > 1) {
    recievedByte1 = Serial.read();
    recievedByte2 = Serial.read();
    Serial.print("I read: ");
    //Serial.print(recievedByte1);
    int first = recievedByte1 * 128 + recievedByte2;
    //int last = recievedByte1 * recievedByte2;
    Serial.print(" ");
    Serial.println(first);
    //Serial.println(recievedByte2);
  }
}
