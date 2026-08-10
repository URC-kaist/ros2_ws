# Rocket M2 RTP 링크 지연 측정 구현 계획

이 문서는 영상에 새 타임스탬프를 삽입하거나 영상 속 움직임을 검출하지
않고, 로버 송신측과 베이스 수신측에서 동일한 RTP 패킷을 캡처해 Rocket M2
구간의 지연을 측정하기 위한 구현 계획이다. 컨텍스트가 초기화되더라도 패킷
식별자, 시계 보정 부호, 결과 스키마와 검증 범위가 바뀌지 않도록 세부 계약을
함께 기록한다.

## 1. 목표와 측정 경계

측정 대상은 다음 경로다.

```text
로버 컴퓨터의 Rocket 방향 네트워크 인터페이스
  -> 로버측 OS 송신 큐/NIC
  -> Rocket M2 무선 링크
  -> 베이스측 NIC/수신 큐
  -> 베이스 컴퓨터의 Rocket 방향 네트워크 인터페이스
```

엄밀히 말해 결과에는 양쪽 호스트의 NIC와 커널 네트워크 처리 시간이 조금
포함된다. 캡처 인터페이스를 Rocket M2에 직접 연결된 인터페이스로 선택하면
Rocket 구간에 가장 가까운 값을 얻는다.

이번 구현은 다음 값을 제공해야 한다.

- 같은 RTP 패킷의 로버 캡처 시각과 베이스 캡처 시각 차이;
- 스트림별 지연 `min/mean/p50/p95/p99/max`;
- 패킷 손실, 베이스에서만 보인 패킷, 중복 및 음수 지연 패킷 수;
- 로버 제공 bitrate와 베이스 전달 bitrate;
- 도착 간격 변동으로 계산한 packet-delay variation 통계;
- 개별 매칭 패킷 CSV와 기계 판독 가능한 JSON 보고서;
- JSON 보고서를 대시보드에서 불러와 Rocket M2 구간 통계 표시.

## 2. 측정 원리와 불변 계약

### 2.1 영상 프로토콜 무변경

RTP/H.264 payload, RTP header, UDP port, XBEE 프로토콜 및 브라우저 영상
프로토콜은 변경하지 않는다. 패킷 내부에 호스트 시각을 추가하지 않는다.

동일 패킷은 기존 RTP 필드로 식별한다.

```text
(udp_destination_port, ssrc, sequence_number, rtp_timestamp)
```

- `udp_destination_port`는 중앙 `video_streams.json`의 스트림 매핑이다.
- `ssrc`는 동시 송신기와 파이프라인 재시작을 구분한다.
- 16비트 sequence wrap은 32비트 RTP timestamp가 함께 있으므로 구분된다.
- 동일 키가 실제로 중복 캡처되면 각 키의 시각 목록을 오름차순으로 짝짓는다.
- RTP payload type 기본값은 현재 파이프라인과 같은 96이다.

### 2.2 캡처 시각과 시계 보정

`tcpdump`가 기록한 pcap packet timestamp를 사용한다. 가능하면 nanosecond
precision을 요청하되, 분석기는 microsecond/nanosecond classic pcap을 모두
읽는다.

두 호스트 시계는 chrony/PTP 등으로 동기화해야 한다. 분석기의 보정 인자는
다음 부호를 고정한다.

```text
clock_offset_us = base_clock - rover_clock
link_latency_us = base_capture_us - rover_capture_us - clock_offset_us
```

예를 들어 베이스 시계가 로버보다 250 us 빠르면 `--clock-offset-us 250`을
전달한다. 값을 생략하면 0으로 계산하되 보고서에 `clock_offset_assumed=true`
경고를 남긴다. 음수 지연은 버리지 않고 통계와 경고에 포함해 잘못된 시계
보정을 드러낸다.

### 2.3 캡처 포맷과 데이터 최소화

- concrete interface에서 classic pcap으로 캡처한다. `any` 사용은 피한다.
- BPF는 중앙 설정의 영상 UDP destination port만 허용한다.
- snap length는 Ethernet/IP/UDP/RTP header와 작은 payload prefix만 남기는
  192 bytes로 제한한다.
- 전체 H.264 영상 payload를 별도 JSON/CSV에 복사하지 않는다.
- pcap과 보고서는 기본적으로 기존 파일을 덮어쓰지 않는다.
- 캡처 스크립트는 root 권한을 자체 획득하지 않으며, 필요한 경우 운영자가
  명시적으로 `sudo` 또는 `CAP_NET_RAW/CAP_NET_ADMIN`을 제공한다.

## 3. 큰 계획

### 단계 A: 재현 가능한 양단 캡처 도구

중앙 영상 설정에서 스트림/UDP 포트를 읽고 안전한 `tcpdump` 명령을 생성하는
Python CLI를 추가한다. 로버와 베이스에서 동일 CLI를 사용하되 role과 interface,
output만 다르게 지정한다. 종료 후 role, host, interface, 선택 스트림, 시작/종료
시각 및 tcpdump 결과를 sidecar JSON에 기록한다.

완료 조건: 실제 네트워크나 root 없이 명령 생성과 설정 검증을 단위 테스트할
수 있고, 운영 명령이 영상 포트 이외의 트래픽을 캡처하지 않는다.

### 단계 B: 무의존 pcap/RTP 분석기

외부 Python 패키지 없이 classic pcap, Ethernet/VLAN/Linux cooked header,
IPv4/UDP/RTP를 읽는 분석기를 추가한다. 양쪽 패킷을 기존 RTP 키로 매칭하고
스트림별 JSON 보고서 및 선택적 packet sample CSV를 생성한다.

완료 조건: 합성 pcap으로 정상 매칭, 고정 지연, 시계 offset, 손실, 중복,
sequence wrap, 다른 UDP port 무시 및 malformed packet 처리를 검증한다.

### 단계 C: 기존 T6 움직임 측정 제거 및 브라우저 통계 재구성

로버 XBEE 브리지의 motion-onset helper, odometry 진단 subscription, T6 trace와
관련 파라미터를 제거한다. 명령 경로 T0/T1/T3/T4는 유지한다.

대시보드는 T6 및 `T7c-T6` 응답 프레임 추정을 제거한다. 대신 선택한 영상
스트림에서 이미 존재하는 T7a/T7b/T7c를 매 프레임 매칭해 다음 rolling 통계를
계산한다.

```text
base_to_browser = corrected(T7b) - T7a
decode_render    = T7c - T7b
```

이 통계는 Rocket M2 구간이 아니라 베이스 이후 병목을 배제하기 위한 보조
지표다. 최대 600개 표본만 유지하고 UI 갱신은 매 프레임이 아니라 묶어서 한다.

완료 조건: UI와 JSONL 어디에도 T6나 움직임 기반 video-back 값이 남지 않고,
진단 비활성 상태의 rover 제어/deadman 동작은 그대로다.

### 단계 D: Rocket 보고서 대시보드 표시

분석기 JSON schema의 strict TypeScript parser를 추가한다. 사용자가 로컬 JSON
보고서를 선택하면 현재 영상 stream 또는 보고서의 첫 stream에 대해 다음을
표시한다.

- matched/sent/received packet 수;
- loss percentage;
- link latency p50/p95/p99/max;
- rover offered/base delivered bitrate;
- clock offset 및 보고서 warning.

보고서는 브라우저 메모리에서만 읽고 서버에 업로드하지 않는다. 기존 Arm은
명령 전달 T0/T3 측정용임을 UI에 명확히 표시하고 Rocket 측정은 pcap import로
분리한다.

완료 조건: 유효 보고서 import, 잘못된 schema 오류, stream 선택 연동을 순수
TypeScript 테스트와 dashboard check로 검증한다.

### 단계 E: 통합 문서와 전체 회귀 검증

운영자가 다음 순서를 그대로 실행할 수 있도록 `scripts/latency/README.md`와
영상/dashboard 문서를 갱신한다.

1. 로버/베이스 시계 동기화 확인;
2. 양쪽 인터페이스와 같은 stream 선택;
3. 베이스/로버에서 캡처 시작;
4. 일정 시간 영상 부하 발생;
5. pcap을 한 호스트로 모아 분석;
6. JSON을 dashboard에 import;
7. 필요하면 bitrate sweep과 Rocket 우회 결과 비교.

하드웨어 캡처는 자동 검증에서 수행하지 않는다. 소프트웨어 테스트는 합성
pcap, gateway tests, dashboard tests/check, ROS Humble 선택 빌드 및 diff 검사를
포함한다.

## 4. 작은 계획

### A1. 디렉터리와 공통 설정 로더

추가 파일:

- `scripts/latency/rtp_capture.py`
- `scripts/latency/test_rtp_latency.py`
- `scripts/latency/README.md`

`rtp_capture.py`는 repository root를 파일 위치로부터 계산하고 기본 설정
`rover/ros2_ws/src/mr2_launch/config/video_streams.json`을 읽는다. CLI 인자:

```text
--role rover|base                 필수
--interface <name>                필수
--output <capture.pcap>           필수
--stream-id <id>                  반복 가능, 생략 시 전체
--duration-s <seconds>            선택
--config <video_streams.json>     선택
--tcpdump <binary>                기본 tcpdump
--overwrite                       명시적 덮어쓰기
```

stream ID, UDP port, output parent, 기존 파일, duration, interface 문자열을
검증한다. shell을 사용하지 않고 argument array로 `tcpdump`를 실행한다.
SIGINT/종료 시 pcap을 flush하도록 tcpdump에 SIGINT를 전달한다.

예상 명령 형태:

```text
tcpdump -i <iface> -n -U -s 192 -B 4096
  --time-stamp-precision=nano -w <output>
  udp and (dst port 5000 or dst port 5002)
```

### A2. 캡처 sidecar

`<output>.metadata.json`에 schema version, role, hostname, interface, config path,
stream/port 목록, requested duration, started/finished epoch, tcpdump exit code를
기록한다. pcap과 sidecar 모두 자격 증명이나 H.264 payload를 JSON으로 복제하지
않는다.

### B1. pcap decoder

추가 파일:

- `scripts/latency/analyze_rtp_latency.py`

지원 범위:

- classic pcap little/big endian;
- microsecond/nanosecond timestamp magic;
- link type Ethernet(1), Linux SLL(113), Linux SLL2(276);
- 802.1Q/802.1ad VLAN;
- IPv4 UDP, non-fragmented RTP v2;
- RTP CSRC/extension/padding이 있어도 식별 header 파싱;
- payload type filter 기본 96.

pcapng는 명확한 오류로 거부하고 `tcpdump` wrapper가 만드는 classic pcap을
사용하도록 안내한다.

### B2. 매칭과 통계

CLI 인자:

```text
--rover <rover.pcap>              필수
--base <base.pcap>                필수
--output <report.json>            필수
--samples-csv <samples.csv>       선택
--stream-id <id>                  반복 가능
--config <video_streams.json>     선택
--payload-type 96                 선택
--clock-offset-us <float>         선택, base-rover
--overwrite                       명시적 덮어쓰기
```

패킷별 출력 열은 stream, port, SSRC, sequence, RTP timestamp, rover/base capture
epoch, corrected link latency, UDP payload bytes다. 지연 percentile은 nearest-rank
또는 선형 보간 중 하나를 구현 코드와 테스트에서 고정한다. 보고서에는 계산
방법을 명시한다.

stream별 bitrate duration은 해당 캡처에서 첫/마지막 선택 패킷 사이로 계산한다.
표본이 2개 미만이면 bitrate와 percentile 중 계산 불가능한 값은 null로 둔다.

### B3. 합성 pcap 테스트

표준 `unittest`만 사용해 테스트 내부에서 최소 Ethernet/IPv4/UDP/RTP 패킷과
classic pcap bytes를 만든다. 저장 위치는 임시 디렉터리다. 다음을 assertion한다.

- 5 ms 고정 지연의 p50/p95/max;
- `base-rover` offset 적용 부호;
- rover packet 누락에 따른 loss;
- duplicate/unmatched base count;
- 두 stream port 분리;
- sequence 65535 -> 0 wrap 매칭;
- VLAN 및 잘린 packet 무시;
- microsecond/nanosecond pcap 처리;
- 기존 output 덮어쓰기 거부;
- 캡처 BPF/argument 생성.

### C1. rover motion 진단 제거

수정/삭제:

- `mr2_xbee_bridge/src/xbee_bridge_node.cpp`
- `mr2_xbee_bridge/CMakeLists.txt`
- `mr2_xbee_bridge/README.md`
- `motion_onset_detector.hpp` 삭제
- `motion_onset_detector_test.cpp` 삭제

제거 대상은 `motion_odom_topic`, linear/angular threshold, consecutive samples,
motion mutex/state/subscription, `motion_onset` JSON뿐이다. T3/T4 command trace와
`enable_latency_diagnostics`는 유지한다. deadman zero command는 계속 latency
trace를 만들지 않는다.

### C2. dashboard 진단 store 변경

수정:

- `dashboard/src/lib/latencyDiagnostics.ts`
- `dashboard/test/latencyDiagnostics.test.ts`
- `dashboard/src/components/VideoStreamCard.tsx`
- `dashboard/src/lib/videoGateway.ts`

`physical_motion_detected_t6`, response-frame selection, rover `motion_onset`,
`roverExecutionMs`, `coarseVideoBackMs`를 제거한다. T7 metadata 매칭은
`streamId:T7a` 키를 계속 사용하되 렌더 완료 때 rolling 표본을 추가한다.
표본 snapshot은 count와 p50/p95를 제공한다. stream 변경/reset 때 초기화한다.

### D1. 보고서 parser와 테스트

추가/수정:

- `dashboard/src/lib/rtpLatencyReport.ts`
- `dashboard/test/rtpLatencyReport.test.ts`
- `dashboard/package.json`

parser는 schema/kind/stream array/숫자 범위를 검증하고 UI가 사용하는 최소
형태로 normalize한다. NaN, 음수 packet count, 없는 latency stats를 거부하거나
null로 보존하는 규칙을 테스트한다. dashboard test script가 두 테스트를 모두
실행하도록 test entrypoint를 정리한다.

### D2. UI 변경

수정:

- `dashboard/src/components/LatencyDiagnosticsPanel.tsx`
- `dashboard/src/components/ControlPanel/LatencyDiagnosticsPanel.css`

화면을 세 구역으로 표시한다.

```text
Command path: T0/T1/T3/T4와 computer->rover, rover publish
Rocket M2 RTP report: import, link p50/p95/p99/max, loss, bitrate
Browser live: base->browser p50/p95, decode/render p50/p95
```

T6와 `Video back (coarse)` 문구는 완전히 제거한다. report import 오류는 제어와
영상을 중단시키지 않고 패널 안에 표시한다.

### E1. 문서와 실행 예시

다음 문서를 갱신한다.

- `scripts/latency/README.md`
- `docs/video-pipeline.md`
- `dashboard/README.md`
- `rover/ros2_ws/src/mr2_xbee_bridge/README.md`
- 기존 `docs/latency-mvp-implementation-plan.md`에는 T6 방식이 폐기되었고 새
  문서로 대체되었다는 표시를 추가한다.

### E2. 검증 순서

각 단계 완료 후 해당 테스트를 수행하고 실패를 수정한 뒤 다음으로 넘어간다.

```bash
python3 -m unittest scripts.latency.test_rtp_latency

cd base/gateway
npm test

cd ../../dashboard
npm run test:latency
npm run check

cd rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to mr2_xbee_bridge
colcon test --packages-select mr2_xbee_bridge
colcon test-result --verbose

cd ../../..
python3 -m py_compile scripts/latency/*.py
git diff --check
```

호스트에 ROS가 없으면 기존 Humble Docker 이미지에서 선택 빌드한다. 실제
Rocket M2, NIC, root tcpdump와 카메라 캡처는 하드웨어 검증으로 명확히 남긴다.

## 5. 완료 기준

- 영상/RTP/UDP wire format이 바뀌지 않는다.
- 양단 pcap에서 기존 RTP 식별자로 같은 패킷이 매칭된다.
- base-rover clock offset 부호가 문서/코드/테스트에서 일치한다.
- 고정 backlog도 packet transit latency p50/p95로 드러난다.
- loss, bitrate와 개별 표본을 함께 보존해 대역폭 병목을 해석할 수 있다.
- T6 움직임 검출 코드와 UI가 제거된다.
- 대시보드가 분석 JSON과 브라우저 이후 통계를 구분해 표시한다.
- 합성 pcap, gateway, dashboard, 가능한 ROS 검증이 통과한다.
- 실제 RF 결과를 소프트웨어 테스트만으로 검증했다고 주장하지 않는다.

## 6. 구현 결과 (2026-08-10)

단계 A부터 E까지 구현을 완료했다.

- 캡처 CLI는 중앙 영상 설정으로 BPF를 만들고 192-byte snap length의 classic
  pcap 및 sidecar metadata를 생성한다. 기존 파일은 기본적으로 보호한다.
- 분석기는 외부 Python 패키지 없이 Ethernet/VLAN/SLL/SLL2, IPv4/UDP/RTP와
  microsecond/nanosecond pcap을 읽는다.
- RTP key별 중복 목록을 시간순으로 매칭하고 clock offset을 적용해 packet
  sample CSV와 stream별 JSON 통계를 만든다.
- sidecar의 tcpdump kernel-drop 수를 JSON에 포함하고, 0보다 크면 loss 결과가
  신뢰 불가능하다는 warning을 생성한다.
- rover motion-onset/T6 코드와 dashboard의 움직임 기반 video-back 추정을
  제거했다. T0/T1/T3/T4 명령 경로는 유지했다.
- dashboard는 RTP JSON을 로컬 import해 Rocket 지연/loss/bitrate를 표시하고,
  T7a/T7b/T7c는 별도의 rolling base-to-browser/decode-render 통계로 표시한다.

완료된 검증:

```text
Python synthetic pcap/CLI unittest         8 passed
Python py_compile                          passed
Dashboard latency/report tests             passed
Dashboard typecheck, ESLint, Vite build    passed
Gateway Node tests                         55 passed
mr2_xbee_bridge ROS 2 Humble build         passed
mr2_xbee_bridge colcon test                0 failures (no package tests remain)
Launch-file Python compilation             passed
git diff --check                           passed
```

Dashboard build는 호스트 Node 18.19.1에서도 성공했지만 Vite가 요구하는
20.19+/22.12+ 버전 경고를 출력했다. ROS 검증은 disposable Humble Docker
container에서 수행했고 이미지에 없던 `ros-humble-control-msgs`를 container
내부에만 설치했다. 실제 Rocket M2 인터페이스, RF 링크, root tcpdump 및 카메라
부하는 이 환경에서 검증하지 않았다.
