#ifndef IR_H
#define IR_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

extern int ir_tx1, ir_rx1; // right
extern int ir_tx2, ir_rx2 ;  // mid
extern int ir_tx3 , ir_rx3 ; // left
extern esp_now_peer_info_t peerInfo;
extern uint8_t broadcastAddress[6];


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void initWiFi();
void initESPNow();
void registerPeer();

extern int tx[3];

extern int kc;
int getIR(int tx, int rx);

void get3IR();

int wallFront();
int wallLeft();
int wallRight();
#endif