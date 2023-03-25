#include "NewPing.h"
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400  // Maximum distance we want to ping for (in centimeters).
int MoveBot(int);

int s1 = A0, s2 = A3, s3 = A2, s4 = A1, s5 = A4;
int up= 5, down=6;
int Mb1 = 7, Mb2 = 8, Ma1 = 11, Ma2 = 12;
int Mbe = 9, Mae = 10, Mf1 = 13, Mf2 = A0;
int cm = 0;
int flag = 0;
int fork_state = 0;  // 0 = down 1 = up 

const int MAX_PWM = 255;
int SPEED_PERCENT = 40;

int pwm = (MAX_PWM * SPEED_PERCENT) / 100;
 // Maximum PWM value
//const int  // Overall speed as a percentage of the maximum speed

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);

  pinMode(Mae,OUTPUT);
  pinMode(Mbe,OUTPUT);
  pinMode(Ma1, OUTPUT);
  pinMode(Ma2, OUTPUT);
  pinMode(Mb1, OUTPUT);
  pinMode(Mb2, OUTPUT);

  pinMode(up, OUTPUT);
  pinMode(down, OUTPUT);

  while (!Serial){
    Serial.println("Yet to receive data");
  }
  Serial.println("Connected");
  Serial.print("Awaiting allocation");
}

void loop() { 
  //MoveBot();
  if (Serial.available())
  {
    String state = Serial.readString();
    Serial.println(state);

    flag = (int(state[5])-48)*100 + (int(state[6])-48)*10 + (int(state[7])-48);
    Serial.println(flag);

    if ( flag == 6)
    {
      //work();
      MoveBot();
      Serial.print("I have been allocated");
    }
    else if ( flag == 8)
    {
      Serial.print("Bot 2 is allocated");
    }
    
  }

}

void work() {
cm = sonar.ping_cm();
Serial.print("Distance = ");
Serial.print(sonar.ping_cm());
Serial.println(" cm");

if (cm < 1) {
digitalWrite(Ma1, LOW);
digitalWrite(Ma2, LOW);
digitalWrite(Mb1, LOW);
digitalWrite(Mb2, LOW);
digitalWrite(f_up, HIGH);
digitalWrite(f_down, LOW);
delay(5000); // 5 secs to pick
digitalWrite(f_up, LOW);
digitalWrite(f_down, LOW);
delay(2000);
}
else if( cm < 5){
MoveBot(18);
}
else if( cm < 10){
MoveBot(25);
}
else{
MoveBot(35);
}
}

int MoveBot( int SPEED_PERCENT) {
int sensor1 = digitalRead(s1);
int sensor2 = digitalRead(s2);
int sensor3 = digitalRead(s3);
int sensor4 = digitalRead(s4);
int sensor5 = digitalRead(s5);

int pwm = (MAX_PWM * SPEED_PERCENT) / 100;

Serial.print("Distance = ");
Serial.print(sonar.ping_cm());
Serial.println(" cm");
delay(500);

if (sensor3 == HIGH && sensor2 == LOW && sensor4 == LOW) { // Center sensor is on line
analogWrite(Mae, pwm);
analogWrite(Mbe, pwm);
digitalWrite(Ma1, HIGH);
digitalWrite(Ma2, LOW);
digitalWrite(Mb1, HIGH);
digitalWrite(Mb2, LOW);
} else if (sensor2 == HIGH && sensor1 == LOW && sensor3 == HIGH && sensor4 == LOW) { // Left sensor is on line
analogWrite(Mae, pwm / 2);
analogWrite(Mbe, pwm);
digitalWrite(Ma1, LOW);
digitalWrite(Ma2, HIGH);
digitalWrite(Mb1, HIGH);
digitalWrite(Mb2, LOW);}
else if (sensor2 == HIGH && sensor1 == LOW && sensor3 == LOW && sensor4 == LOW) { // Left sensor is on line
analogWrite(Mae, pwm / 2);
analogWrite(Mbe, pwm);
digitalWrite(Ma1, LOW);
digitalWrite(Ma2, HIGH);
digitalWrite(Mb1, HIGH);
digitalWrite(Mb2, LOW);
} else if (sensor4 == HIGH && sensor5 == LOW && sensor3 == HIGH && sensor2 == LOW) { // Right sensor is on line
analogWrite(Mae, pwm);
analogWrite(Mbe, pwm / 2);
digitalWrite(Ma1, HIGH);
digitalWrite(Ma2, LOW);
digitalWrite(Mb1, LOW);
digitalWrite(Mb2, HIGH);}
else if (sensor4 == HIGH && sensor5 == LOW && sensor3 == LOW && sensor2 == LOW) { // Right sensor is on line
  analogWrite(Mae, pwm);
  analogWrite(Mbe, pwm / 2);
  digitalWrite(Ma1, HIGH);
  digitalWrite(Ma2, LOW);
  digitalWrite(Mb1, LOW);
  digitalWrite(Mb2, HIGH);
} else { // All sensors are off line
  analogWrite(Mae, 0);
  analogWrite(Mbe, 0);
}
return 0;
}

void MoveBot() {
  while (1)
  {
    Serial.println("into movebot");
  int sensor1 = digitalRead(s1);
  int sensor2 = digitalRead(s2);
  int sensor3 = digitalRead(s3);
  int sensor4 = digitalRead(s4);
  int sensor5 = digitalRead(s5);  
  
   if (sensor4 == HIGH && sensor5 == HIGH && sensor3 == HIGH && sensor2 == HIGH && sensor1 == HIGH) 
   { Serial.println("node detected");
    analogWrite(Mae, 0);
    analogWrite(Mbe, 0);
    digitalWrite(Ma1, LOW);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, LOW);
    digitalWrite(Mb2, LOW);
    delay(2000);
    fork_state? lower_fork(5000): lift_fork(5000);
    
   }
 else if (sensor3 == HIGH && sensor2 == LOW && sensor4 == LOW) { // Center sensor is on line
    analogWrite(Mae, pwm);
    analogWrite(Mbe, pwm);
    digitalWrite(Ma1, HIGH);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, HIGH);
    digitalWrite(Mb2, LOW);
  } else if (sensor2 == HIGH && sensor1 == LOW && sensor3 == HIGH && sensor4 == LOW) { // Left sensor is on line
    analogWrite(Mae, pwm / 2);
    analogWrite(Mbe, pwm);
    digitalWrite(Ma1, LOW);
    digitalWrite(Ma2, HIGH);
    digitalWrite(Mb1, HIGH);
    digitalWrite(Mb2, LOW);}
    else if (sensor2 == HIGH && sensor1 == LOW && sensor3 == LOW && sensor4 == LOW) { // Left sensor is on line
    analogWrite(Mae, pwm / 2);
    analogWrite(Mbe, pwm);
    digitalWrite(Ma1, LOW);
    digitalWrite(Ma2, HIGH);
    digitalWrite(Mb1, HIGH);
    digitalWrite(Mb2, LOW);
  } else if (sensor4 == HIGH && sensor5 == LOW && sensor3 == HIGH && sensor2 == LOW) { // Right sensor is on line
    analogWrite(Mae, pwm);
    analogWrite(Mbe, pwm / 2);
    digitalWrite(Ma1, HIGH);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, LOW);
    digitalWrite(Mb2, HIGH);}
    else if (sensor4 == HIGH && sensor5 == LOW && sensor3 == LOW && sensor2 == LOW) { // Right sensor is on line
    analogWrite(Mae, pwm);
    analogWrite(Mbe, pwm / 2);
    digitalWrite(Ma1, HIGH);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, LOW);
    digitalWrite(Mb2, HIGH);
  }
  else
  { // All sensors are off line
    analogWrite(Mae, 0);
    analogWrite(Mbe, 0);
  }
  }
  
  }

void lift_fork(int dist)
{
    if ( fork_state == 0 )
    {
        Serial.println("lifting the fork up");
        digitalWrite(up, HIGH);
        digitalWrite(down, LOW);
        delay(dist);
        digitalWrite(up, LOW);
        digitalWrite(down, LOW);
        fork_state = 1; 
        analogWrite(Mae, pwm/1.5);
    analogWrite(Mbe, pwm/1.5);
    digitalWrite(Ma1, HIGH);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, HIGH);
    digitalWrite(Mb2, LOW);
    delay(1200);
    digitalWrite(Ma1, LOW);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, LOW);
    digitalWrite(Mb2, LOW);
    }
    else if (fork_state == 1 )
    {
      Serial.println("fork already up");
      }
}

void lower_fork(int dist)
{
  if ( fork_state == 1)
  {
     Serial.println("lowewring the fork");
    digitalWrite(up, LOW);
    digitalWrite(down, HIGH);
    delay(dist);
    digitalWrite(up, LOW);
    digitalWrite(down, LOW);
     fork_state = 0;
     delay(1000);
     analogWrite(Mae, pwm/1.5);
    analogWrite(Mbe, pwm/1.5);
    digitalWrite(Ma1, LOW);
    digitalWrite(Ma2, HIGH);
    digitalWrite(Mb1, LOW);
    digitalWrite(Mb2, HIGH);
    delay(1000);
     digitalWrite(Ma1, LOW);
    digitalWrite(Ma2, LOW);
    digitalWrite(Mb1, LOW);
    digitalWrite(Mb2, LOW);
  }
  
   else if ( fork_state == 0)
   {
     Serial.println("fork already down");
   }
}


  
