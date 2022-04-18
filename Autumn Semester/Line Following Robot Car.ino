 #include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Left Motor  connections
int enA = 3;
int in1 = A1;
int in2 = A2;
// Right Motor connections
int enB = 11;
int in3 = A3;
int in4 = A4; 

// IR connections
int IR1=12; //IR is left motor
int IR2=2; //IR is right motor

int value1, value2;
int vSpeed1=90;
int vSpeed2= 80;
int turn_speed = 75;
int turn_delay = 10; 
int turn_speed2=85;


void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(IR1,INPUT);
  pinMode(IR2,INPUT);
 
  
  lcd.print("Time taken:");
  

  

}

 
//when black, 1

void loop() {


  while(millis()/1000<35){
     
  int left_sensor_pin= digitalRead(IR1);
  int right_sensor_pin = digitalRead(IR2);
  lcd.print(millis()/1000);
  lcd.setCursor(1,1);

  if(right_sensor_pin ==  0 && left_sensor_pin == 0){
    Serial.println("go forward");
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);


    analogWrite (enA, 80);
    analogWrite (enB, 70); 
  }

  if(right_sensor_pin == 0 && left_sensor_pin == 1){
    Serial.println("turn right");

    digitalWrite (in1, HIGH);
    digitalWrite (in2, LOW);
    digitalWrite (in3, LOW);
    digitalWrite (in4, HIGH);

    analogWrite (enA, 75);
    analogWrite (enB, 85);

    
  }

if(right_sensor_pin == 1 && left_sensor_pin == 0){
  Serial.println("turn left");

  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  analogWrite (enA, 85);
  analogWrite(enB, 75);

  
} 

  if(right_sensor_pin == 1 && left_sensor_pin == 1){
    Serial.println("stop");
    
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);


   analogWrite (enA, 0);
    analogWrite (enB, 0);
  }


  } 
   while(millis()/1000>=35 && millis()/1000<=10000){
     
  int left_sensor_pin= digitalRead(IR1);
  int right_sensor_pin = digitalRead(IR2);
  lcd.print(millis()/1000);
  lcd.setCursor(1,1);
      if(right_sensor_pin ==  0 && left_sensor_pin == 0){
    Serial.println("go forward");
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);


    analogWrite (enA, 105);
    analogWrite (enB, 95); 
  }

  if(right_sensor_pin == 0 && left_sensor_pin == 1){
    Serial.println("turn right");

    digitalWrite (in1, HIGH);
    digitalWrite (in2, LOW);
    digitalWrite (in3, LOW);
    digitalWrite (in4, HIGH);

    analogWrite (enA, 75);
    analogWrite (enB, 85);

    
  }

  if(right_sensor_pin == 1 && left_sensor_pin == 0){
  Serial.println("turn left");

  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  analogWrite (enA, 85);
  analogWrite(enB, 75);

  
  } 

  if(right_sensor_pin == 1 && left_sensor_pin == 1){
    Serial.println("stop");

    analogWrite (enA, 0);
    analogWrite (enB, 0);
  }


  } 
  
  
 }
 
  
  
 


   
 

  

  

  
  
