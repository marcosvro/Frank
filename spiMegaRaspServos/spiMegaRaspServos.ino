#include <SPI.h>
#include "SPI_anything.h"
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define PIN_SERVO_0 13//pe E
#define PIN_SERVO_1 18//calcanhar E
#define PIN_SERVO_2 17 //joelho E
#define PIN_SERVO_3 16//coxa E
#define PIN_SERVO_4 15//quadril E
#define PIN_SERVO_5 0//bacia E
#define PIN_SERVO_6 4//bacia D
#define PIN_SERVO_7 5//quadril D
#define PIN_SERVO_8 6//coxa D
#define PIN_SERVO_9 7//joelho D
#define PIN_SERVO_10 12//calcanhar D
#define PIN_SERVO_11 13// pe D 

//perna esquerda para direita 0-7
#define MIN_RANGE_SERVO_0 700 //pe E ( 38 )
#define MIN_RANGE_SERVO_1 600 //calcanhar E - (1F)
#define MIN_RANGE_SERVO_2 700 //joelho E - 36 
#define MIN_RANGE_SERVO_3 600//coxa E- 43
#define MIN_RANGE_SERVO_4 600//quadril E - 12N
#define MIN_RANGE_SERVO_5 700//bacia E
#define MIN_RANGE_SERVO_6 700//bacia D
#define MIN_RANGE_SERVO_7 700//quadril D - 10N
#define MIN_RANGE_SERVO_8 700//coxa D 2F
#define MIN_RANGE_SERVO_9 700//joelho D - 37
#define MIN_RANGE_SERVO_10 600 //calcanhar D - (11N)
#define MIN_RANGE_SERVO_11 700// pe D (39 )

#define MAX_RANGE_SERVO_0 2200
#define MAX_RANGE_SERVO_1 2300
#define MAX_RANGE_SERVO_2 2200
#define MAX_RANGE_SERVO_3 2700
#define MAX_RANGE_SERVO_4 2700
#define MAX_RANGE_SERVO_5 2300
#define MAX_RANGE_SERVO_6 2300
#define MAX_RANGE_SERVO_7 2300
#define MAX_RANGE_SERVO_8 2300
#define MAX_RANGE_SERVO_9 2500
#define MAX_RANGE_SERVO_10 2300
#define MAX_RANGE_SERVO_11 2400 

int motors[] = {90, 90, 90, 90, 90, 180, 173, 90, 90, 90, 130, 90}; //posição motores
int q[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     //posição inicial calibração copia

Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;
Servo servo_7;
Servo servo_8;
Servo servo_9;
Servo servo_10;
Servo servo_11;

int tempoDelayServo = 50;

typedef struct tanto
{
//  char cth= '#';
  byte pos[12];
}bloco;

char faz[8];
bool state = false;

void setup() {
  Serial.begin (9600);   // debugging

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPCR |= _BV(SPIE);

  initServos();
  
}

ISR (SPI_STC_vect){
  //delayMicroseconds(100);
  state = SPI_readAnything(faz, 8);
}// end of interrupt routine SPI_STC_vect

void loop() {
  if(state){
    //walkState(int(faz.pos[0]),int(faz.pos[1]),int(faz.pos[2]),int(faz.pos[3]),int(faz.pos[4]),int(faz.pos[5]),int(faz.pos[6]),int(faz.pos[7]),int(faz.pos[8]),int(faz.pos[9]),int(faz.pos[10]),int(faz.pos[11]));
    state = false;
    for(int i = 0; i < 8; i++){
      Serial.print(int(faz[i]));
      Serial.print(" ");
    }
    Serial.println();
    walkState();
  }
}

void initServos() {

  q[0] = motors[0];
  q[1] = motors[1];
  q[2] = motors[2];
  q[3] = motors[3];
  q[4] = motors[4];
  q[5] = motors[5];
  q[6] = motors[6];
  q[7] = motors[7];

  servo_0.attach(PIN_SERVO_0, MIN_RANGE_SERVO_0, MAX_RANGE_SERVO_0);
  writeServos(tempoDelayServo);
  servo_1.attach(PIN_SERVO_1, MIN_RANGE_SERVO_1, MAX_RANGE_SERVO_1);
  writeServos(tempoDelayServo);
  servo_2.attach(PIN_SERVO_2, MIN_RANGE_SERVO_2, MAX_RANGE_SERVO_2);
  writeServos(tempoDelayServo);
  servo_3.attach(PIN_SERVO_3, MIN_RANGE_SERVO_3, MAX_RANGE_SERVO_3);
  writeServos(tempoDelayServo);
  servo_4.attach(PIN_SERVO_4, MIN_RANGE_SERVO_4, MAX_RANGE_SERVO_4);
  writeServos(tempoDelayServo);
  servo_5.attach(PIN_SERVO_5, MIN_RANGE_SERVO_5, MAX_RANGE_SERVO_5);
  writeServos(tempoDelayServo);
  servo_6.attach(PIN_SERVO_6, MIN_RANGE_SERVO_6, MAX_RANGE_SERVO_6);
  writeServos(tempoDelayServo);
  servo_7.attach(PIN_SERVO_7, MIN_RANGE_SERVO_7, MAX_RANGE_SERVO_7);
  writeServos(tempoDelayServo);
}

void walkState() {
  
  for(int i =0; i<8; i++) {
     motors[i] = q[i] + faz[i];
  }
  
  writeServos(tempoDelayServo);
}

void writeServos(int espera) {
  servo_0.writeMicroseconds(map(motors[0], 0, 180, MIN_RANGE_SERVO_0, MAX_RANGE_SERVO_0));
  servo_1.writeMicroseconds(map(motors[1], 0, 180, MIN_RANGE_SERVO_1, MAX_RANGE_SERVO_1));
  servo_2.writeMicroseconds(map(motors[2], 0, 180, MIN_RANGE_SERVO_2, MAX_RANGE_SERVO_2));
  servo_3.writeMicroseconds(map(motors[3], 0, 180, MIN_RANGE_SERVO_3, MAX_RANGE_SERVO_3));
  servo_4.writeMicroseconds(map(motors[4], 0, 180, MIN_RANGE_SERVO_4, MAX_RANGE_SERVO_4));
  servo_5.writeMicroseconds(map(motors[5], 0, 180, MIN_RANGE_SERVO_5, MAX_RANGE_SERVO_5));
  servo_6.writeMicroseconds(map(motors[6], 0, 180, MIN_RANGE_SERVO_6, MAX_RANGE_SERVO_6));
  servo_7.writeMicroseconds(map(motors[7], 0, 180, MIN_RANGE_SERVO_7, MAX_RANGE_SERVO_7));

  delay(espera);
}
