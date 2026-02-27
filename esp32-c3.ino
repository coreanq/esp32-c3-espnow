#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/stream_buffer.h>

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

// 디버그 출력 제어 (프로덕션에서는 주석 처리)
// #define DEBUG_VERBOSE

// BLE 전역 변수
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t bleTxBuffer[BLE_MTU_SIZE];

// BLE 수신 watchdog: 마지막 패킷 수신 시각 (0=비활성)
volatile unsigned long lastBleRxTime = 0;
#define BLE_RX_TIMEOUT_MS 5000  // 5초 무응답 시 버퍼 클리어

// 버퍼 리셋 플래그: BLE 콜백에서 직접 리셋 시 레이스 컨디션 방지
// bypssSerialTask 루프 상단에서 안전하게 처리
volatile bool pendingBufferReset = false;

// MTU 캐시: 연결당 한 번만 쿼리 (0=미협상)
uint16_t cachedMTU = 0;

// RS-485 bypass 데이터 버퍼 (bypssSerialTask 전용)
typedef struct struct_message {
    char message[DEBUG_MSG_BUFFER_SIZE];
} struct_message;

struct_message bypassSerialData;

// FreeRTOS 스트림 버퍼 (바이트 스트림 bulk I/O)
StreamBufferHandle_t msgRecv485Stream;
StreamBufferHandle_t msgSend485Stream;

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
        lastBleRxTime = 0;
        cachedMTU = 0;
        // bypssSerialTask 루프에서 안전하게 처리 (레이스 컨디션 방지)
        pendingBufferReset = true;
        DEBUG_PORT.println("BLE Client Disconnected");
    }
};

// BLE RX 콜백: 스마트폰에서 수신한 데이터를 RS-485 송신 스트림에 bulk 전송
class MyRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            lastBleRxTime = millis();  // watchdog 타이머 갱신
            xStreamBufferSend(msgSend485Stream, (const uint8_t*)rxValue.c_str(), rxValue.length(), 0);
#ifdef DEBUG_VERBOSE
            DEBUG_PORT.print("recved-ble-data: ");
            DEBUG_PORT.println(rxValue.length());
#endif
        }
    }
};

// BLE advertising 시작 (설정은 initBLE에서 1회만 - 재시작 시 UUID 중복 누적 방지)
void startAdvertising() {
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

    // Advertising 설정 (1회만 - startAdvertising 재호출 시 UUID 중복 방지)
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 7.5ms (iOS 호환성)
    pAdvertising->setMaxPreferred(0x0C);  // 15ms

    // Advertising 시작
    startAdvertising();

    DEBUG_PORT.println("BLE NUS GATT Server initialized");
}

// RS-485에서 수신한 데이터를 BLE로 전송 (sendDataToWifi 대체)
void sendDataToBLE() {
    size_t itemCount = xStreamBufferBytesAvailable(msgRecv485Stream);

    if (itemCount == 0) {
        return;
    }

    // MTU 캐시: 연결당 한 번만 쿼리 (매 호출마다 BLE 스택 쿼리 오버헤드 제거)
    if (cachedMTU == 0) {
        uint16_t negotiatedMTU = pServer->getPeerMTU(pServer->getConnId());
        if (negotiatedMTU <= 3) return;  // MTU 미협상, 다음 루프에서 재시도
        negotiatedMTU -= 3;  // ATT overhead 3바이트 제외
        cachedMTU = (negotiatedMTU < BLE_MTU_SIZE - 3) ? negotiatedMTU : (BLE_MTU_SIZE - 3);
    }
    int maxPacketSize = cachedMTU;

    while (itemCount > 0) {
        if (!deviceConnected) return;  // 연결 해제 시 즉시 중단 (크래시 방지)

        int sendSize = (itemCount > (size_t)maxPacketSize) ? maxPacketSize : itemCount;

        // bulk read: 한 번의 호출로 sendSize 바이트 수신
        sendSize = xStreamBufferReceive(msgRecv485Stream, bleTxBuffer, sendSize, 0);
        if (sendSize == 0) break;

        pTxCharacteristic->setValue(bleTxBuffer, sendSize);
        pTxCharacteristic->notify();

#ifdef DEBUG_VERBOSE
        DEBUG_PORT.print("send 485-data to ble: ");
        DEBUG_PORT.println(sendSize);
#endif

        // 단일코어: 1ms 고정 대기 대신 즉시 yield로 BLE 스택 태스크에 CPU 양보
        taskYIELD();

        itemCount = xStreamBufferBytesAvailable(msgRecv485Stream);
    }
}

// RS-485 bypass 태스크 (변경 없음 - 큐 기반이라 무선 측과 독립적)
void bypssSerialTask(void* parameter)
{
    int sendLen = 0;
    int recvLen = 0;

    while( true )
    {
        // BLE 연결 해제 시 버퍼 리셋 (onDisconnect 에서 직접 리셋 시 레이스 컨디션 방지)
        if (pendingBufferReset) {
            xStreamBufferReset(msgSend485Stream);
            xStreamBufferReset(msgRecv485Stream);
            pendingBufferReset = false;
            DEBUG_PORT.println("BLE disconnected - buffers cleared");
        }

        recvLen = BYPASS_SRC_PORT.readBytes(bypassSerialData.message, DEBUG_MSG_BUFFER_SIZE - 1);
        bypassSerialData.message[recvLen] = '\0';

        if( recvLen > 0 ) {
            // bulk write: 한 번의 호출로 recvLen 바이트 전송
            xStreamBufferSend( msgRecv485Stream, bypassSerialData.message, recvLen, 0 );
#ifdef DEBUG_VERBOSE
            DEBUG_PORT.print("\nrecv data from 485: ");

            if( bypassSerialData.message[1] == 0x10 || bypassSerialData.message[1] == 0x06 ) {
                for(int i = 0; i < recvLen; i++) {
                    DEBUG_PORT.print(bypassSerialData.message[i], HEX);
                    DEBUG_PORT.print(" ");
                }
            }
#endif
        }

        else {
            // DEBUG_PORT.println("No data to send");
        }

        do{
            sendLen = xStreamBufferBytesAvailable( msgSend485Stream );
            if( sendLen == 0 ) {
                vTaskDelay( pdMS_TO_TICKS(1) );
                break;
            }

            if( sendLen > 256 ) {
                sendLen = 256;
            }

            // bulk read: 한 번의 호출로 sendLen 바이트 수신
            sendLen = xStreamBufferReceive( msgSend485Stream, bypassSerialData.message, sendLen, 0 );

             digitalWrite(RS485_TX_ENABLE_PIN, HIGH);
             delayMicroseconds(10);  // RS-485 트랜시버 enable 시간 (~1-10μs)

             BYPASS_SRC_PORT.write(bypassSerialData.message, sendLen);
             BYPASS_SRC_PORT.flush(true);
             delayMicroseconds(25);  // 마지막 바이트 stop bit 전송 완료 대기 (460800baud 기준 ~22μs)

             digitalWrite(RS485_TX_ENABLE_PIN, LOW);

#ifdef DEBUG_VERBOSE
             DEBUG_PORT.print("send data to 485:  ");

             if( bypassSerialData.message[1] == 0x10 || bypassSerialData.message[1] == 0x06 ) {
                 for( int i = 0; i < sendLen; i++ ) {
                     DEBUG_PORT.print(bypassSerialData.message[i], HEX);
                     DEBUG_PORT.print(" ");
                 }
             }
             DEBUG_PORT.println("");
#endif


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

    // StreamBuffer 생성 (trigger level 1: 1바이트 수신 시 즉시 unblock)
    // BLE(~200KB/s) vs RS-485(~46KB/s) 속도 차이 고려하여 8192로 확대
    msgRecv485Stream = xStreamBufferCreate( DEBUG_MSG_BUFFER_SIZE * 2, 1 );
    msgSend485Stream = xStreamBufferCreate( DEBUG_MSG_BUFFER_SIZE * 2, 1 );

    // 태스크 스택 4096 (안전 마진 확보), 우선순위 2 (RS-485 타이밍 우선)
    xTaskCreate(bypssSerialTask, "bypssSerialTask", 4096, NULL, 2, NULL);

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
        vTaskDelay(pdMS_TO_TICKS(500));  // BLE 스택 정리 대기 (블로킹 delay → yield 방식으로 변경)
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
        // watchdog: 마지막 수신 후 5초 경과 시 stale 데이터 클리어
        if (lastBleRxTime > 0 && (millis() - lastBleRxTime > BLE_RX_TIMEOUT_MS)) {
            xStreamBufferReset(msgSend485Stream);
            lastBleRxTime = 0;
            DEBUG_PORT.println("BLE RX timeout - send buffer cleared");
        }
        sendDataToBLE();
        digitalWrite(LED_PIN, LOW);  // LED ON (active low)
    } else {
        // 미연결: LED 500ms 깜빡임 (advertising 중)
        led_on(500);
    }

    vTaskDelay(1);  // 스케줄러에 CPU 양보 (싱글코어 필수)
}
