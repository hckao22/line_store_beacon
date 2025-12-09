#include "esp_bt.h"
#include "esp_gap_ble_api.h"

// LINE SIMPLE BEACON 設定
static const uint8_t HWID[5] = {0x01, 0x7a, 0x97, 0x29, 0xf2}; // LINE 017a9729f2 = MOCHA
// static const uint8_t HWID[5] = {0x01, 0x8c, 0x2d, 0x31, 0x2a}; // LINE 018c2d312a = 桃園慈護宮 BEACON_參拜路線圖
// static const uint8_t HWID[5] = {0x01, 0x8c, 0x32, 0x3a, 0xe9}; // LINE 018c323ae9 = 桃園慈護宮 BEACON_天上聖母

// 電壓監測設定
static const uint8_t ADC_PIN = 35;
static const float R1 = 4000.0;
static const float R2 = 2000.0;

// LINE SIMPLE BEACON 常數
static const uint16_t UUID_FOR_LINECORP = 0xFE6F;
static const uint8_t MAX_SIMPLEBEACON_DEVICEMESSAGE_SIZE = 13;
static const uint8_t LINE_BEACON_FRAME_TYPE = 0x02; // Simple Beacon

// ========== 發送功率設定 (控制發送距離) ==========
// ESP32 BLE TX Power 等級對照表：
// ESP_PWR_LVL_N12 = -12dBm (最低功率，約 1-2 公尺)
// ESP_PWR_LVL_N9  = -9dBm  (約 2-3 公尺)
// ESP_PWR_LVL_N6  = -6dBm  (約 3-5 公尺)
// ESP_PWR_LVL_N3  = -3dBm  (約 5-8 公尺)
// ESP_PWR_LVL_N0  = 0dBm   (約 8-12 公尺) - 預設值
// ESP_PWR_LVL_P3  = +3dBm  (約 12-20 公尺)
// ESP_PWR_LVL_P6  = +6dBm  (約 20-30 公尺)
// ESP_PWR_LVL_P9  = +9dBm  (最高功率，約 30-50 公尺)

static const esp_power_level_t BLE_TX_POWER_LEVEL = ESP_PWR_LVL_N12; // 選擇功率等級
static const int8_t TX_POWER_DBM = -12; // 對應的 dBm 值 (用於廣播封包)

// Device Message 變數
static uint8_t deviceMessageSize = 1;
static uint8_t deviceMessage[MAX_SIMPLEBEACON_DEVICEMESSAGE_SIZE];

// BLE 廣播相關常數
static const uint8_t MAX_BLE_ADVERTISING_DATA_SIZE = 31;
static const uint16_t HCI_LE_Set_Advertising_Data = (0x08 << 10) | 0x0008;
static const uint16_t HCI_LE_Set_Advertising_Enable = (0x08 << 10) | 0x000A;
static const uint16_t HCI_LE_Set_Advertising_Parameters = (0x08 << 10) | 0x0006;

// LINE Beacon 建議間隔：152.5ms
static const uint16_t BEACON_INTERVAL_MS = 152; // 使用 152ms (最接近 152.5ms)
static const uint16_t ADVERTISING_INTERVAL = 244; // 152.5ms = 244 * 0.625ms

// 工具函數：顯示 hex dump
static void _dump(const char *title, uint8_t *data, size_t dataSize) {
  Serial.printf("%s [%d]:", title, dataSize);
  for (size_t i = 0; i < dataSize; i++) {
    Serial.printf(" %02x", data[i]);
  }
  Serial.println();
}

// 資料寫入工具函數
static void putUint8(uint8_t **bufferPtr, uint8_t data) {
  *(*bufferPtr)++ = data;
}

static void putUint16LE(uint8_t **bufferPtr, uint16_t data) {
  *(*bufferPtr)++ = lowByte(data);
  *(*bufferPtr)++ = highByte(data);
}

static void putArray(uint8_t **bufferPtr, const void *data, size_t dataSize) {
  memcpy(*bufferPtr, data, dataSize);
  (*bufferPtr) += dataSize;
}

// 執行 HCI 命令
static void executeBluetoothHCICommand(uint16_t opCode, const uint8_t *hciData, uint8_t hciDataSize) {
  uint8_t buf[5 + MAX_BLE_ADVERTISING_DATA_SIZE];
  uint8_t *bufPtr = buf;

  putUint8(&bufPtr, 1); // H4_TYPE_COMMAND
  putUint16LE(&bufPtr, opCode);
  putUint8(&bufPtr, hciDataSize);
  putArray(&bufPtr, hciData, hciDataSize);

  uint8_t bufSize = bufPtr - buf;

  while (!esp_vhci_host_check_send_available())
    ;
  esp_vhci_host_send_packet(buf, bufSize);
}

// 設定 BLE 發送功率 (控制距離)
static void setBluetoothTxPower() {
  // 設定 BLE 發送功率
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, BLE_TX_POWER_LEVEL);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BLE_TX_POWER_LEVEL);
  
  // 顯示當前功率設定
  const char* powerDesc;
  switch(BLE_TX_POWER_LEVEL) {
    case ESP_PWR_LVL_N12: powerDesc = "-12dBm (約 1-2m)"; break;
    case ESP_PWR_LVL_N9:  powerDesc = "-9dBm (約 2-3m)"; break;
    case ESP_PWR_LVL_N6:  powerDesc = "-6dBm (約 3-5m)"; break;
    case ESP_PWR_LVL_N3:  powerDesc = "-3dBm (約 5-8m)"; break;
    case ESP_PWR_LVL_N0:  powerDesc = "0dBm (約 8-12m)"; break;
    case ESP_PWR_LVL_P3:  powerDesc = "+3dBm (約 12-20m)"; break;
    case ESP_PWR_LVL_P6:  powerDesc = "+6dBm (約 20-30m)"; break;
    case ESP_PWR_LVL_P9:  powerDesc = "+9dBm (約 30-50m)"; break;
    default: powerDesc = "未知"; break;
  }
  
  Serial.printf("BLE 發送功率已設定為: %s\n", powerDesc);
}

// 設定廣播參數 (152.5ms 間隔)
static void setAdvertisingParameters() {
  uint8_t hciData[15];
  uint8_t *ptr = hciData;

  // Advertising_Interval_Min (244 * 0.625ms = 152.5ms)
  putUint16LE(&ptr, ADVERTISING_INTERVAL);
  // Advertising_Interval_Max (244 * 0.625ms = 152.5ms)
  putUint16LE(&ptr, ADVERTISING_INTERVAL);
  // Advertising_Type: ADV_NONCONN_IND (0x03)
  putUint8(&ptr, 0x03);
  // Own_Address_Type: Public (0x00)
  putUint8(&ptr, 0x00);
  // Peer_Address_Type: Public (0x00)
  putUint8(&ptr, 0x00);
  // Peer_Address: 00:00:00:00:00:00
  for (int i = 0; i < 6; i++) {
    putUint8(&ptr, 0x00);
  }
  // Advertising_Channel_Map: All channels (0x07)
  putUint8(&ptr, 0x07);
  // Advertising_Filter_Policy: Process all (0x00)
  putUint8(&ptr, 0x00);

  executeBluetoothHCICommand(HCI_LE_Set_Advertising_Parameters, hciData, 15);
  
  Serial.println("廣播間隔已設定為 152.5ms");
}

// 更新 Device Message
static void updateSimpleBeaconDeviceMessage(const char dm[], int dmsize) {
  memset(deviceMessage, 0x00, MAX_SIMPLEBEACON_DEVICEMESSAGE_SIZE);
  
  // 確保不超過最大長度 (13 bytes)
  int actualSize = (dmsize - 1) < MAX_SIMPLEBEACON_DEVICEMESSAGE_SIZE ? (dmsize - 1) : MAX_SIMPLEBEACON_DEVICEMESSAGE_SIZE;
  memcpy(deviceMessage, dm, actualSize);
  deviceMessageSize = actualSize;
  
  Serial.printf("Device Message: %s (%d bytes)\n", dm, deviceMessageSize);
}

// 更新廣播資料
static void updateAdvertisingData() {
  uint8_t data[MAX_BLE_ADVERTISING_DATA_SIZE];
  uint8_t *dataPtr = data;

  // Flags
  putUint8(&dataPtr, 2); // Length
  putUint8(&dataPtr, ESP_BLE_AD_TYPE_FLAG);
  putUint8(&dataPtr, ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

  // Complete List of 16-bit Service UUIDs
  putUint8(&dataPtr, 3); // Length
  putUint8(&dataPtr, ESP_BLE_AD_TYPE_16SRV_CMPL);
  putUint16LE(&dataPtr, UUID_FOR_LINECORP);

  // Service Data - LINE SIMPLE BEACON
  putUint8(&dataPtr, 1 + 9 + deviceMessageSize); // Length
  putUint8(&dataPtr, ESP_BLE_AD_TYPE_SERVICE_DATA);
  putUint16LE(&dataPtr, UUID_FOR_LINECORP);
  putUint8(&dataPtr, LINE_BEACON_FRAME_TYPE); // Frame Type: 0x02 (Simple Beacon)
  putArray(&dataPtr, HWID, 5);                // Hardware ID (5 bytes)
  putUint8(&dataPtr, TX_POWER_DBM);           // TX Power (dBm)
  putArray(&dataPtr, deviceMessage, deviceMessageSize); // Device Message

  uint8_t dataSize = dataPtr - data;
  _dump("LINE SIMPLE BEACON", data, dataSize);

  // 準備 HCI 資料
  uint8_t hciDataSize = 1 + MAX_BLE_ADVERTISING_DATA_SIZE;
  uint8_t hciData[hciDataSize];
  hciData[0] = dataSize;
  memcpy(hciData + 1, data, dataSize);

  executeBluetoothHCICommand(HCI_LE_Set_Advertising_Data, hciData, hciDataSize);
}

// 啟用廣播
static void enableBluetoothAdvertising() {
  uint8_t enable = 1;
  executeBluetoothHCICommand(HCI_LE_Set_Advertising_Enable, &enable, 1);
  Serial.println("LINE SIMPLE BEACON 廣播已啟用");
}

// VHCI 回呼函數
static void notifyHostSendAvailableHandler() {}

static int notifyHostRecvHandler(uint8_t *data, uint16_t len) {
  return 0;
}

static esp_vhci_host_callback_t vhciHostCallback = {
    notifyHostSendAvailableHandler,
    notifyHostRecvHandler
};

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("LINE SIMPLE BEACON 初始化");
  Serial.println("=================================");

  // ADC 設定
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  // Bluetooth 初始化
  btStart();
  esp_vhci_host_register_callback(&vhciHostCallback);
  
  // 設定 BLE 發送功率 (控制距離)
  setBluetoothTxPower();
  
  // 設定廣播參數 (152.5ms 間隔)
  setAdvertisingParameters();
  
  // 設定初始 Device Message
  char initialMessage[] = "actshop.io";
  updateSimpleBeaconDeviceMessage(initialMessage, sizeof(initialMessage));
  updateAdvertisingData();
  
  // 啟用廣播
  enableBluetoothAdvertising();
  
  Serial.println("=================================");
  Serial.println("LINE SIMPLE BEACON 就緒");
  Serial.println("發送間隔: 152ms (152.5ms)");
  Serial.println("=================================\n");
}

void loop() {
  // 讀取電壓
  uint16_t analog12bit = analogRead(ADC_PIN);
  float voltage = analog12bit / 4095.0 * 3.9 * (R1 + R2) / R2;
  
  Serial.printf("電壓: %.2fV | ADC: %d\n", voltage, analog12bit);

  // 更新 Device Message
  char dm[] = "actshop.io"; // 最多 13 bytes
  updateSimpleBeaconDeviceMessage(dm, sizeof(dm));
  updateAdvertisingData();

  // https://developers.line.biz/en/docs/messaging-api/beacon-device-spec/#requirements-for-devices-compliant-with-line-beacon-specs
  // We strongly recommend that you send LINE Beacon packets at 152.5ms intervals.
  // LINE Beacon 建議間隔：152.5ms
  // 實際使用 152ms (因為 delay 只能用整數)
  delay(BEACON_INTERVAL_MS);
}
