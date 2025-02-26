#include "IR.h"

int ir_tx1 = 17, ir_rx1 = 34; // right
int ir_tx2 = 5, ir_rx2 = 39;  // mid
int ir_tx3 = 23, ir_rx3 = 36; // left

int tx[3] = {ir_tx1, ir_tx2, ir_tx3};
int kc = 200;
int getIR(int tx, int rx)
{
    digitalWrite(tx, HIGH);
    int rs = analogRead(rx);
    digitalWrite(tx, LOW);
    return rs;
}

int wallFront() {
    digitalWrite(ir_tx2, HIGH);
    int rs = analogRead(ir_rx2);
    digitalWrite(ir_tx2, LOW);
    return rs > kc ? 1 : 0;
}

int wallLeft() {
    digitalWrite(ir_tx3, HIGH);
    int rs = analogRead(ir_rx3);
    digitalWrite(ir_tx3, LOW);
    return rs > kc ? 1 : 0;
}

int wallRight() {
    digitalWrite(ir_tx1, HIGH);
    int rs = analogRead(ir_rx1);
    digitalWrite(ir_tx1, LOW);
    return rs > kc ? 1 : 0;
}
int get3IR()
{
    int left = getIR(ir_tx3, ir_rx3);
    int mid = getIR(ir_tx2, ir_rx2);
    int right = getIR(ir_tx1, ir_rx1);

    left = left > kc ? 1 : 0;
    mid = mid > kc ? 1 : 0;
    right = right > kc ? 1 : 0;

    return right + 2 * mid + 4 * left;
}