#include <SPI.h>
#include "SPI_anything.h"
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


#define PIN_SERVO_0 13
#define PIN_SERVO_1 18
#define PIN_SERVO_2 17
#define PIN_SERVO_3 16
#define PIN_SERVO_4 15
#define PIN_SERVO_5 0
#define PIN_SERVO_6 4
#define PIN_SERVO_7 5

#define MIN_RANGE_SERVO_0 700 //calcanhar_eixo_X
#define MIN_RANGE_SERVO_1 700 //calcanhar_eixo_Y
#define MIN_RANGE_SERVO_2 700 //joelho
#define MIN_RANGE_SERVO_3 700 //coxa
#define MIN_RANGE_SERVO_4 700 //pelves
#define MIN_RANGE_SERVO_5 700 //bacia
#define MIN_RANGE_SERVO_6 700 //ombro
#define MIN_RANGE_SERVO_7 700 //cotovelo

#define MAX_RANGE_SERVO_0 2400
#define MAX_RANGE_SERVO_1 2400
#define MAX_RANGE_SERVO_2 2400
#define MAX_RANGE_SERVO_3 2400
#define MAX_RANGE_SERVO_4 2400
#define MAX_RANGE_SERVO_5 2400
#define MAX_RANGE_SERVO_6 2400
#define MAX_RANGE_SERVO_7 2400

int motors[] = {90, 90, 90, 90, 90, 90, 90, 90}; //posição motores
int qi[] = {0, 0, 0, 0, 0, 0, 0, 0};     //posição inicial calibração copia

Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;
Servo servo_7;

int tempoDelayServo = 0;

typedef struct tanto
{
//  char cth= '#';
  byte pos[12];
}bloco;

char faz[8];
bool state = false;

ISR (SPI_STC_vect){
  //delayMicroseconds(100);
  state = SPI_readAnything(faz, 8);
}// end of interrupt routine SPI_STC_vect

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(9600);

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPCR |= _BV(SPIE);

  initServos();
  
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  if(Serial.available() && Serial.read() == 127){
    for(int i = 0; i< 8; i++) {
      faz[i] = Serial.read();
    }
    ver = Serial.read();
    if (ver == -127) {
      state = true;
    }
  }
  
  if(state){
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

  qi[0] = motors[0];
  qi[1] = motors[1];
  qi[2] = motors[2];
  qi[3] = motors[3];
  qi[4] = motors[4];
  qi[5] = motors[5];
  qi[6] = motors[6];
  qi[7] = motors[7];

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
     motors[i] = qi[i] + faz[i];
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
