uint32_t nextAutoSend = 1800;
bool enableAutoSend = true;

uint32_t nextMeasurement = 1750;
bool enableMeasurement = true;

uint8_t coefficient = 100;
int16_t PM2_5 = 10;
int16_t PM10 = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(analogRead(0));
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, enableMeasurement);
  delay(1700);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (enableAutoSend && enableMeasurement && millis() > nextAutoSend) {
    uint8_t data[32];
    memset(data, 0, 32);
    data[0] = 0x42;
    data[1] = 0x4d;
    data[3] = 28;
    data[6] = (PM2_5 >> 8) & 255;
    data[7] = (PM2_5) & 255;
    data[8] = (PM10 >> 8) & 255;
    data[9] = (PM10) & 255;
    uint16_t checksum = calcChecksum(data, 30);
    data[30] = (checksum >> 8) & 255;
    data[31] = (checksum) & 255;
    Serial.write(data, 32);
    nextAutoSend += 1000;
  }
  if (enableMeasurement && millis() > nextMeasurement) {
    PM2_5 = random(max(0, PM2_5 - 1), min(PM2_5 + 2, 1000));
    PM10 = random(max(0, PM10 - 1), min(PM10 + 2, 1000));
    nextMeasurement += 750;
  }
  if (Serial.available()) {
    uint8_t buff[16];
    uint8_t dataLen = 0;
    dataLen += Serial.readBytes(buff, 2); // header and len
    dataLen += Serial.readBytes(&buff[dataLen], buff[1]); // cmd and data
    Serial.readBytes(&buff[dataLen], 1);
    uint16_t checksum = calcChecksum(buff, dataLen);
    checksum = (65536 - checksum) & 255;

    uint8_t isSuccess = false;
    uint8_t len = buff[1];
    if (buff[dataLen] == checksum && len > 0) {
      if (len == 1) {
        if (buff[2] == 0x01) {
          if (enableMeasurement == false && enableAutoSend) {
            nextAutoSend = millis() + 1000;
          }
          enableMeasurement = true;
          digitalWrite(LED_BUILTIN, enableMeasurement);
          isSuccess = true;
        } else if (buff[2] == 0x02) {
          enableMeasurement = false;
          digitalWrite(LED_BUILTIN, enableMeasurement);
          isSuccess = true;
        } else if (buff[2] == 0x04) {
          uint8_t data2send[8];
          data2send[0] = 0x40;
          data2send[1] = 0x05;
          data2send[2] = 0x04;
          data2send[3] = (PM2_5 >> 8) & 255;
          data2send[4] = (PM2_5) & 255;
          data2send[5] = (PM10 >> 8) & 255;
          data2send[6] = (PM10) & 255;
          uint16_t checksum = calcChecksum(data2send, 7);
          checksum = (65536 - checksum) & 255;
          data2send[7] = checksum;
          Serial.write(data2send, 8);

          isSuccess = 2;
        } else if (buff[2] == 0x10) {
          uint8_t data2send[5];
          data2send[0] = 0x40;
          data2send[1] = 0x02;
          data2send[2] = 0x10;
          data2send[3] = coefficient;
          uint16_t checksum = calcChecksum(data2send, 4);
          checksum = (65536 - checksum) & 255;
          data2send[4] = checksum;
          Serial.write(data2send, 5);

          isSuccess = 2;
        } else if (buff[2] == 0x20) {
          enableAutoSend = false;
          isSuccess = true;
        } else if (buff[2] == 0x40) {
          if (!enableAutoSend) {
            nextAutoSend = millis() + 1000;
          }
          enableAutoSend = true;
          isSuccess = true;
        }
      } else if (len == 2) {
        if (buff[2] == 0x08) {
          coefficient = buff[3];
          isSuccess = true;
        } 
      }
    }
    if (isSuccess == true) {
      sendACK();
    } else if (isSuccess == false) {
      sendNAK();
    }
  }
}

void sendACK() {
  Serial.write(0xA5);
  Serial.write(0xA5);
}

void sendNAK() {
  Serial.write(0x96);
  Serial.write(0x96);
}

uint16_t calcChecksum(uint8_t data[], uint8_t dataLen) {
  uint16_t checksum = 0;
  for (int i = 0; i < dataLen; i++) {
    checksum += data[i];
  }
  return checksum;
}
