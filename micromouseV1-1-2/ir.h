#ifndef IR_H
#define IR_H

#include <Arduino.h>
extern int ir_tx1, ir_rx1; // right
extern int ir_tx2, ir_rx2 ;  // mid
extern int ir_tx3 , ir_rx3 ; // left

extern int tx[3];

extern int kc;
int getIR(int tx, int rx);

int get3IR();

int wallFront();
int wallLeft();
int wallRight();
#endif