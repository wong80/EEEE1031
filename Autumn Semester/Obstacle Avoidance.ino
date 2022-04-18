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

int trig = 12;
int echo = 2;
float duration, distance;

void setup()
{
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    // Set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    Serial.begin(9600);
}

void loop()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(10);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW); // transmit ultrasonicwave

    duration = pulseIn(echo, HIGH); // getting the time
    distance = (duration * 0.0343 / 2);
    lcd.home();
    lcd.print("Distance:");
    lcd.setCursor(10, 0);
    lcd.print(distance);
    Serial.println(distance);

    if (distance <= 15)
    {
        analogWrite(enA, 0);
        analogWrite(enB, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
    }
    else
    {
        analogWrite(enA, 200);
        analogWrite(enB, 200);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
}