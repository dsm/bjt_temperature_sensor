/**/

static uint32_t timestamp = 0;
static uint32_t tx_ok = 1;
static uint8_t rx_buffer[2] = { 0 };
static uint16_t temp_raw = 0;
static float temp_f = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial1.setRX(D17);
  Serial1.setTX(D16);
  Serial1.setPollingMode(true);
  Serial1.begin(115200);
}

void loop() {
  if ((millis() - timestamp) > 100) {
    timestamp = millis();
    //if (tx_ok) {
      //tx_ok = 0;
      Serial1.print("RT\r\n");
    //}
    delayMicroseconds(1);
    if (Serial1.available()) {
      Serial1.readBytes(&rx_buffer[0], 2);
      temp_raw = (rx_buffer[1] << 8) | rx_buffer[0];
      if (temp_raw != 0xFFFF) {

        temp_f = temp_raw / 100.0;
        Serial.printf("/*TemperatureTest,%.2f*/\n\r", temp_f);

      } else {
        Serial.println("error!");
      }
      //tx_ok = 1;
    }
  }
}
