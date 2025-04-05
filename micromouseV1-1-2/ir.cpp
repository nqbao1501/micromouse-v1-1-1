#include "IR.h"

int ir_tx1 = 17, ir_rx1 = 34; // right
int ir_tx2 = 5, ir_rx2 = 39;  // mid
int ir_tx3 = 23, ir_rx3 = 36; // left

esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0xAB, 0x7A, 0x38};

int tx[3] = {ir_tx1, ir_tx2, ir_tx3};
int kc = 300;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
}

void initESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
}

void registerPeer() {
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}
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
void get3IR()
{
    int left = getIR(ir_tx3, ir_rx3);
    int mid = getIR(ir_tx2, ir_rx2);
    int right = getIR(ir_tx1, ir_rx1);

    tx[0] = left;
    tx[1] = mid;
    tx[2] = right;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&tx, sizeof(tx));
}