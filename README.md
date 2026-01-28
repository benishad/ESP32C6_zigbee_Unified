# AMKIT_zigbee_통합_펌웨어

이 프로젝트는 AMKIT 매쉬망 구성을 위한 지그비 펌웨어를 Build & Flash 가이드를 포함함
- ~~리눅스 Build & Flash 버전~~
- 이전 리눅스 기반 Build & Flash 가이드를 윈도우 버전으로 업데이트
- **단일 펌웨어**로 Coordinator / End Device 동작
- **ROLE 변경은 NVS 저장 + 재부팅 기반**
- 명령어는 **ASCII 문자열 + CRLF(`\r\n`)** 기준

---

# 구성
- **코디네이터, 라우터,엔드디바이스 통합ver**

ESP-IDF v5.3.2 + esp-zigbee-sdk(참고) 기반이며, ESP32-C6 보드에서 테스트 완료

---

## 사전 준비 - 1

- 윈도우 Visual Studio Code 설치
- Extensions에서 ESP-IDF 설치 (해당 확장이 가상환경 빌드 플래시 등 필요한거 지원함)
- IDF 버전을 5.3.2로 설정 (esp32c6개발을 위한 버전)

---

## 사전 준비 - 2

- [여기서 espressif 지그비 sdk 다운로드](https://github.com/espressif/esp-zigbee-sdk)
- 깃을 내려받거나 zip를 받기
- esp-zigbee-sdk-main 폴더가 생기는데 components 폴더 하나 만들어서 아래에 위치

esp-idf가 components 폴더를 인식하여 내부 include를 사용함

  ```bash
  ─ components
    └─esp-zigbee-sdk-main
      ├─아래
      ├─많은
      ├─내부
      └─파일들
  ```

이제 후에 이 components를 통쨰로 옮겨서 지그비 라이브러리를 사용함 ha, zcl, nwk등..

---

## 어디에 폴더를 위치 시키는가

#### 기본 디렉토리 구조 (프로젝트 만들면 자동으로 생성)
 ```bash
  ─ Project
    ├─.devcontainer
    ├─.vscode
    ├─build
    ├─main
    ├─managed_components
    ├─.clangd
    ├─CMakeLists.txt
    ├─dependencies.lock
    ├─partitions.csv
    ├─README.md
    ├─sdkconfig
    └─sdkconfig.defaults
```

#### 여기에
 ```bash
  ─ Project
    ├─.devcontainer
    ├─.vscode
    ├─build
    ├─components
      └─esp-zigbee-sdk-main
    ├─main
    ├─managed_components
    ├─.clangd
    ├─CMakeLists.txt
    ├─dependencies.lock
    ├─partitions.csv
    ├─README.md
    ├─sdkconfig
    └─sdkconfig.defaults
```

## 프로젝트 설정

수정 위치
 ```bash
  ─ Project
      ├─components
      ├─main
        └─CMakeLists.txt
 ```
components 폴더를 위치 시키고 지그비 sdk 사용을 위해 CMakeLists.txt 수정

```
 idf_component_register(
    SRCS "프로젝트.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp-zigbee-lib
        nvs_flash
        esp_timer
    )
```

---

## 기본 통신 정보

| 항목       | 내용            |
| -------- | ------------- |
| 인터페이스    | UART          |
| Baudrate | 115200        |
| 데이터      | 8-N-1         |
| 방향       | STM32 → ESP32 |
| 응답       | ESP32 → STM32 |

---

## 명령어 목록 요약

```text
ZB+OK
ZB+PING,START
ZB+RESET
ZB+FACTORY_RESET
ZB+PERMIT,OPEN
ZB+PERMIT,CLOSE
ZB+ROLE,COORD
ZB+ROLE,ROUTER
ZB+ROLE,ED
ZB+JOIN
ZB+TX
ZB+MESSAGE
```

---

## `ZB+OK` — Zigbee 모듈 준비 상태 확인

### 📌 설명

ESP32 Zigbee 펌웨어가 **정상 동작 중인지 확인**하기 위한 기본 명령어입니다.

### ▶ 입력

```text
ZB+OK
```

### ◀ 응답

```text
ZB
ZB+OK
```

### 비고

* Zigbee 스택 상태와 무관
* UART 통신 확인용 핑 명령

---

## `ZB+PING,START` — PING-PONG 헬스 체크 시작

### 설명

Coordinator가 **네트워크에 참여한 End Device들에게 주기적으로 PING 전송**을 시작합니다.

* APS Layer 기반
* ED 응답(PONG)을 기반으로 생존 상태 관리

### ▶ 입력

```text
ZB+PING,START
```

### ◀ 응답

```text
ZB
ZB+PING_OK
```

### 조건

* **Coordinator 전용**
* 이미 실행 중이면 중복 실행되지 않음

### 내부 동작

* 10초마다 `ZB+PING` 전송
* 30초 이상 PONG 미수신 시 MISS
* 3회 MISS → ED DEAD 판정

---

## `ZB+RESET` — ESP32 소프트 리셋

### 설명

ESP32 Zigbee 모듈을 **즉시 재부팅**합니다.

### ▶ 입력

```text
ZB+RESET
```

### ◀ 응답

```text
ZB+RESETTING,OK
```

### 비고

* NVS 데이터 유지
* ROLE 변경과는 무관

---

## `ZB+FACTOPRY_RESET` — ESP32 팩토리 리셋

### 설명

ESP32 Zigbee 모듈을 **NVS에 포함된 데이터를 지우며, 즉시 재부팅**합니다.

### ▶ 입력

```text
ZB+FACTORY_RESET
```

### ◀ 응답

```text
ZB+FACTOPRY,OK
```

### 비고

* NVS 데이터 삭제
* ROLE 변경과는 무관

---

## `ZB+PERMIT,OPEN` — 네트워크 조인 허용 (Coordinator 전용)

### 설명

Zigbee 네트워크에 **새 End Device 조인을 허용**합니다.

### ▶ 입력

```text
ZB+PERMIT,OPEN
```

### ◀ 응답

```text
ZB
ZB+OPEN_OK
```

### 조건

* **Coordinator 전용**
* 기본 허용 시간: **60초**

---

## `ZB+PERMIT,CLOSE` — 네트워크 조인 차단 (Coordinator 전용)

### 설명

Zigbee 네트워크의 **조인 허용을 즉시 종료**합니다.

### ▶ 입력

```text
ZB+PERMIT,CLOSE
```

### ◀ 응답

*(별도 OK 메시지 없음 — 로그만 출력)*

---

## `ZB+ROLE,COORD` — 역할 설정: Coordinator

### 설명

ESP32 Zigbee 역할을 **Coordinator**로 설정합니다.

* NVS 저장
* **즉시 재부팅**

### ▶ 입력

```text
ZB+ROLE,COORD
```

### ◀ 응답

```text
ZB
ZB+ROLE,COORD,OK
```

### 동작 흐름

1. ROLE → NVS 저장
2. 재부팅
3. Network Formation 자동 시작

---

## `ZB+ROLE,ROUTER` — 역할 설정: Router

### 설명

ESP32 Zigbee 역할을 **Router**로 설정합니다.

* NVS 저장
* **즉시 재부팅**

### ▶ 입력

```text
ZB+ROLE,ROUTER
```

### ◀ 응답

```text
ZB
ZB+ROLE,ROUTER,OK
```

### 동작 흐름

1. ROLE → NVS 저장
2. 재부팅
3. Network Formation 자동 시작

---

## `ZB+ROLE,ED` — 역할 설정: End Device

### 설명

ESP32 Zigbee 역할을 **End Device**로 설정합니다.

* NVS 저장
* **즉시 재부팅**

### ▶ 입력

```text
ZB+ROLE,ED
```

### ◀ 응답

```text
ZB
ZB+ROLE,ED,OK
```

### 비고

* 네트워크에는 자동으로 조인하지 않음
* `ZB+JOIN` 필요

---

## `ZB+JOIN` — 네트워크 조인 요청 (End Device 전용)

### 설명

End Device가 **Coordinator 네트워크에 조인(Network Steering)** 을 시도합니다.

### ▶ 입력

```text
ZB+JOIN
```

### ◀ 응답

```text
ZB
ZB+JOIN,OK
```

### 조건

* **End Device 전용**
* Zigbee Stack이 시작된 이후만 가능
* Coordinator에서 `PERMIT,OPEN` 상태여야 성공 가능

---

# 오류 응답 목록

| 응답                            | 의미                                 |
| ----------------------------- | ---------------------------------- |
| `ZB+ERR,UNKNOWN`              | 알 수 없는 명령                          |
| `ZB+ERR,ROLE,NOT_COORDINATOR` | Coordinator 전용 명령을 ED에서 실행         |
| `ZB+ERR,ROLE,NOT_ED`          | End Device 전용 명령을 Coordinator에서 실행 |
| `ZB+ERR,ZB_NOT_READY`         | Zigbee 스택 초기화 전                    |


---

# 참고 사항

* ROLE 변경은 **항상 재부팅 기반**
* End Device 생존 관리:

  * `PING → PONG`
  * 3회 미응답 → DEAD
* DIP Switch **5초 롱프레스**:

  * Zigbee Factory Reset
  * NVRAM 삭제 후 재부팅

---

# Troubleshooting

> ~~신규 보드에서 펌웨어가 동작안함~~

> ~~엔드디바이스 1.14 딥스위치 동작시 강제 초기화~~

> ~~엔드디바이스 바인딩 테이블 오류로 코디네이터로 프레임 전송 불가~~