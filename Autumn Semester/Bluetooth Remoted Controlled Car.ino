
// Left Motor  connections
int enA = 3;
int in1 = A1;
int in2 = A2;
// Right Motor connections
int enB = 11;
int in3 = A3;
int in4 = A4; 

int command; //Int to store app command state.

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);
  Serial.begin(9600);  //Set the baud rate to your Bluetooth module.
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();
    Stop(); //Initialize with motors stoped.
    switch (command) {
      case 'F':
        forward();
        break;
      case 'B':
        back();
        break;
        
      case 'L':
        left();
        break;
      case 'R':
        right();
        break;


    }
   
  }
}

void forward() {
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  analogWrite(enA, 110);
  analogWrite(enB, 100);
  Serial.println(1);
}

void back() {
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);

  analogWrite(enA, 110);
  analogWrite(enB, 100);
    Serial.println(1);
}

void right() {
  digitalWrite (in1, LOW);
   digitalWrite (in2, HIGH);
   digitalWrite (in3, HIGH);
   digitalWrite (in4, LOW);

    analogWrite (enA, 80);
    analogWrite (enB, 90);
  Serial.println(2);
}

void left() {
   digitalWrite (in1, HIGH);
   digitalWrite (in2, LOW);
   digitalWrite (in3, LOW);
   digitalWrite (in4, HIGH);

    analogWrite (enA, 80);
    analogWrite (enB, 90);
  Serial.println(3);
}


void Stop() {
  digitalWrite (in1, LOW);
   digitalWrite (in2, LOW);
   digitalWrite (in3, LOW);
   digitalWrite (in4, LOW);
     Serial.println(4);
 

}
