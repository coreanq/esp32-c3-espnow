#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// NUS (Nordic UART Service) UUIDs
#define NUS_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHARACTERISTIC "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHARACTERISTIC "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_DEVICE_NAME       "RS485-BLE"
#define BLE_MTU_SIZE          512

#define LED_PIN 8
#define DEBUG_MSG_BUFFER_SIZE 4096

#define BYPASS_SRC_PORT Serial0
#define DEBUG_PORT      Serial1

#define RS485_TX_ENABLE_PIN 5

// BLE 전역 변수
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t bleTxBuffer[BLE_MTU_SIZE];

// RS-485 bypass 데이터 버퍼 (bypssSerialTask 전용)
typedef struct struct_message {
    char message[DEBUG_MSG_BUFFER_SIZE];
} struct_message;

struct_message bypassSerialData;

// FreeRTOS 큐
QueueHandle_t msgRecv485Queue;
QueueHandle_t msgSend485Queue;

// BLE Server 콜백: 연결/해제 이벤트 처리
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
        deviceConnected = true;

        // 연결 파라미터 업데이트 요청 (min 7.5ms, max 15ms, latency 0, timeout 2s)
        pServer->updateConnParams(param->connect.remote_bda, 6, 12, 0, 200);

        DEBUG_PORT.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        DEBUG_PORT.println("BLE Client Disconnected");
    }
};

// BLE RX 콜백: 스마트폰에서 수신한 데이터를 RS-485 송신 큐에 넣음
class MyRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            DEBUG_PORT.print("recved-ble-data: ");

            for (int i = 0; i < rxValue.length(); i++) {
                uint8_t byte = rxValue[i];
                xQueueSend(msgSend485Queue, &byte, 0);
            }

            DEBUG_PORT.println("");
        }
    }
};

// BLE advertising 시작
void startAdvertising() {
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    // 연결 파라미터 힌트 (iOS 호환성)
    pAdvertising->setMinPreferred(0x06);  // 7.5ms
    pAdvertising->setMaxPreferred(0x0C);  // 15ms
    BLEDevice::startAdvertising();
    DEBUG_PORT.println("BLE Advertising started");
}

// BLE 초기화
void initBLE() {
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(BLE_MTU_SIZE);

    // Server 생성
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // NUS Service 생성
    BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

    // TX Characteristic (Notify) - 서버→클라이언트
    pTxCharacteristic = pService->createCharacteristic(
        NUS_TX_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Write) - 클라이언트→서버
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        NUS_RX_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new MyRxCallbacks());

    // Service 시작
    pService->start();

    // Advertising 시작
    startAdvertising();

    DEBUG_PORT.println("BLE NUS GATT Server initialized");
}

// RS-485에서 수신한 데이터를 BLE로 전송 (sendDataToWifi 대체)
void sendDataToBLE() {
    int itemCount = uxQueueMessagesWaiting(msgRecv485Queue);

    if (itemCount == 0) {
        return;
    }

    // MTU 기반 최대 패킷 크기 (ATT overhead 3바이트 제외)
    int maxPacketSize = BLE_MTU_SIZE - 3;
    if (maxPacketSize > (int)sizeof(bleTxBuffer)) {
        maxPacketSize = sizeof(bleTxBuffer);
    }

    while (itemCount > 0) {
        int sendSize = (itemCount > maxPacketSize) ? maxPacketSize : itemCount;

        for (int i = 0; i < sendSize; i++) {
            xQueueReceive(msgRecv485Queue, &bleTxBuffer[i], 0);
        }

        pTxCharacteristic->setValue(bleTxBuffer, sendSize);
        pTxCharacteristic->notify();

        DEBUG_PORT.print("send 485-data to ble: ");
        DEBUG_PORT.println(sendSize);

        // notify 간 짧은 딜레이 (BLE 스택 처리 여유)
        delay(2);

        itemCount = uxQueueMessagesWaiting(msgRecv485Queue);
    }
}

// RS-485 bypass 태스크 (변경 없음 - 큐 기반이라 무선 측과 독립적)
void bypssSerialTask(void* parameter)
{
    int sendLen = 0;
    int recvLen = 0;

    while( true )
    {
        recvLen = BYPASS_SRC_PORT.readBytes(bypassSerialData.message, DEBUG_MSG_BUFFER_SIZE);
        bypassSerialData.message[recvLen] = '\0';

        if( recvLen > 0 ) {
            for ( int i = 0; i < recvLen; i++ ) {
                xQueueSend( msgRecv485Queue, &bypassSerialData.message[i], 0 );
            }
            DEBUG_PORT.print("\nrecv data from 485: ");


            if( bypassSerialData.message[1] == 0x10 || bypassSerialData.message[1] == 0x06 ) {
                for(int i = 0; i < recvLen; i++) {
                    DEBUG_PORT.print(bypassSerialData.message[i], HEX);
                    DEBUG_PORT.print(" ");
                }
            }
        }

        else {
            // DEBUG_PORT.println("No data to send");
        }

        do{
            sendLen = uxQueueMessagesWaiting( msgSend485Queue );
            if( sendLen == 0 ) {
                vTaskDelay( portTICK_PERIOD_MS);
                break;
            }

            if( sendLen > 200 ) {
                sendLen = 200;
            }


            for( int i = 0; i < sendLen; i++ ) {
                xQueueReceive( msgSend485Queue, &bypassSerialData.message[i], 0 );
            }

            digitalWrite(RS485_TX_ENABLE_PIN, HIGH);
            vTaskDelay( portTICK_PERIOD_MS);

            BYPASS_SRC_PORT.write(bypassSerialData.message, sendLen);
            BYPASS_SRC_PORT.flush(true);

            digitalWrite(RS485_TX_ENABLE_PIN, LOW);
            vTaskDelay( portTICK_PERIOD_MS);

            DEBUG_PORT.print("send data to 485:  ");


            if( bypassSerialData.message[1] == 0x10 || bypassSerialData.message[1] == 0x06 ) {
                for( int i = 0; i < sendLen; i++ ) {
                    DEBUG_PORT.print(bypassSerialData.message[i], HEX);
                    DEBUG_PORT.print(" ");
                }
            }
            DEBUG_PORT.println("");


        }while(false);

    }


}

void setup() {
    // 시리얼 통신 초기화, before begin
    BYPASS_SRC_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    BYPASS_SRC_PORT.setTimeout(1);
    DEBUG_PORT.setRxBufferSize(DEBUG_MSG_BUFFER_SIZE);
    DEBUG_PORT.setTimeout(1);

    Serial.begin(3000000);                              // USB to serial
    BYPASS_SRC_PORT.begin(460800, SERIAL_8N1, 20, 21);  // RX:20, TX:21 핀 사용
    DEBUG_PORT.begin(3000000, SERIAL_8N1, 1, 0);  //  DEBUG RX:1, TX:0 핀 사용

    pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
    digitalWrite(RS485_TX_ENABLE_PIN, LOW); // DE  HIGH 송신 (TX) 활성화

    delay(1000);

    DEBUG_PORT.print("\n\n-------------------------------------------------------------------\n");
    DEBUG_PORT.print("Current Date: ");
    DEBUG_PORT.print(__DATE__);
    DEBUG_PORT.print(" ");
    DEBUG_PORT.println(__TIME__);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // BLE 초기화
    initBLE();

    // Queue 생성
    msgRecv485Queue = xQueueCreate( DEBUG_MSG_BUFFER_SIZE, sizeof(char) );
    msgSend485Queue = xQueueCreate( DEBUG_MSG_BUFFER_SIZE, sizeof(char) );

    // 태스크 스택 크기 2048로 증가 (BLE 스택 사용량 고려)
    xTaskCreate(bypssSerialTask, "bypssSerialTask", 2048, NULL, 1, NULL);

}

void led_on(int duration) {
    static unsigned long last_led_on_time = 0;
    static bool led_state = false;

    if( millis() - last_led_on_time > duration ) {
        digitalWrite(LED_PIN, led_state);
        led_state = !led_state;
        last_led_on_time = millis();
   }
    else {
        digitalWrite(LED_PIN, led_state);
    }
}


void loop() {
    // BLE 연결 해제 후 advertising 재시작
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);  // BLE 스택 정리 대기
        startAdvertising();
        oldDeviceConnected = false;
        DEBUG_PORT.println("Restarting advertising...");
    }

    // 새 연결 감지
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
    }

    // 연결 중: RS-485 수신 데이터를 BLE로 전송, LED 상시 ON
    if (deviceConnected) {
        sendDataToBLE();
        digitalWrite(LED_PIN, LOW);  // LED ON (active low)
    } else {
        // 미연결: LED 500ms 깜빡임 (advertising 중)
        led_on(500);
    }
}
