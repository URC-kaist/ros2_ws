# 방향별 자동 latency diagnostics 상세 구현 계획

상태: 1~7단계 구현 완료, 실제 2-host 현장 검증 대기

이 문서는 `scripts/latency/notes/target-dashboard-workflow.md`에 기록한 목표를
현재 mono-repo에 구현하기 위한 상세 계획이다. 기존 수동 RTP 캡처 도구와
T0/T1/T3/T4 command trace를 폐기하지 않고 자동 측정의 하위 구성 요소로
재사용한다.

구현은 아래 7단계로 진행한다.

1. 공통 측정 모델과 상태 머신
2. Chrony 및 브라우저-베이스 clock readiness
3. Downlink 자동 측정
4. 로버 Uplink 측정 에이전트와 영상 stream 제어
5. 베이스 Uplink coordinator와 캡처/분석 작업
6. RTP 패킷과 브라우저 렌더 프레임의 상관관계
7. 세 버튼 UI, 결과 표시, 통합 검증과 배포

각 단계는 그 단계의 단위 테스트와 완료 조건을 통과한 뒤 다음 단계로
진행한다. 실제 Rocket M2와 카메라를 요구하는 검증만 마지막 현장 검증으로
남긴다.

## 0. 고정할 측정 계약

### 0.1 Downlink 경계

```text
D0 browser controller input
 -> D1 base gateway receives command
 -> D2 rover XBEE bridge receives command
 -> D3 rover publishes actuator command (/base/cmd_vel)
```

표시할 값:

- `browser_to_gateway = D1 - corrected(D0)`
- `gateway_to_rover = D2 - D1`
- `rover_receive_to_publish = D3 - D2`
- `downlink_total = D3 - corrected(D0)`

`corrected(D0)`는 브라우저 시각에 `base_clock - browser_clock` offset을 더한
값이다. 베이스와 로버는 chrony로 동기화되어 있다고 검증한 뒤 같은 epoch
domain으로 취급한다.

종료점은 로버 software의 `/base/cmd_vel` publish다. 실제 wheel 또는 actuator가
움직이기 시작한 시점은 이번 측정에 포함하지 않는다.

### 0.2 Uplink 경계

한 영상 frame은 여러 RTP 패킷으로 나뉜다. 브라우저에 그려진 frame과 하나의
RTP 패킷을 모호하지 않게 연결하기 위해 RTP marker bit가 설정된 frame의 마지막
패킷을 기준 패킷으로 사용한다.

```text
U0 rover captures the frame's RTP marker packet on the Rocket-facing interface
 -> U1 base captures the same RTP marker packet
 -> U2 browser receives the correlated encoded frame over WebSocket
 -> U3 browser completes canvas draw for the decoded frame
```

표시할 값:

- `rocket_m2 = U1 - U0`
- `base_to_browser = corrected(U2) - U1`
- `decode_render = U3 - U2`
- `uplink_total = corrected(U3) - U0`

각 값은 동일한 frame 표본에서 먼저 계산하고 그 표본 배열로
`min/mean/p50/p95/p99/max`를 계산한다. 서로 다른 분포의 percentile을 더해
전체 latency를 만들지 않는다.

`base_to_browser`에는 base NIC 수신 이후 jitterbuffer, depay, H.264 parse,
gateway WebSocket 전송과 브라우저 callback까지가 포함된다. 카메라 exposure,
capture, encoding과 marker 패킷 이전의 frame packetization 시간은 포함하지
않는다.

### 0.3 RTP/frame 식별자

기존 packet match key를 그대로 유지한다.

```text
(udp_destination_port, ssrc, sequence_number, rtp_timestamp)
```

브라우저 frame correlation에는 marker 패킷의 다음 값을 전달한다.

```text
(stream_id, ssrc, marker_sequence_number, rtp_timestamp)
```

SSRC나 sequence가 wrap되더라도 capture window, RTP timestamp와 함께 사용해
충돌을 방지한다. 중복 key가 있으면 capture 시각 순으로 짝짓고 warning을
남긴다.

### 0.4 Video feed count

사용자가 선택한 feed count `N`은 브라우저에 보이는 feed 수만 뜻하지 않는다.
측정 중 로버에서 실제로 송신되는 영상 stream도 N개로 제한한다.

초기 구현의 stream 선택 규칙은 다음과 같다.

1. `video_streams.json`의 `display.order` 오름차순으로 정렬한다.
2. 동률 또는 order가 없으면 설정 파일 순서를 사용한다.
3. 앞의 N개를 선택하고 실제 stream ID를 준비 화면과 결과에 기록한다.
4. 선택된 stream 하나라도 warm-up 후 available 상태가 되지 않으면 측정을
   실패시킨다. 조용히 더 적은 feed로 측정하지 않는다.

측정 종료, 실패, 취소 또는 timeout 시 측정 전 stream enable 상태를 복원한다.

### 0.5 공통 안전 규칙

- Downlink와 Uplink는 독립적인 trial과 결과를 가진다.
- Uplink trial은 동시에 하나만 실행한다.
- 측정값 일부가 빠지면 전체값을 추정하지 않고 해당 trial을 invalid로 표시한다.
- shell command string을 만들지 않고 executable과 argument array로 process를
  실행한다.
- interface, stream ID, duration, path, body size와 trial ID를 모두 검증한다.
- capture process와 stream lease는 성공/실패/취소 모든 경로에서 정리한다.
- normal command/video 경로는 diagnostics가 꺼져 있을 때 기존과 같아야 한다.

## 1. 공통 측정 모델과 상태 머신

### 1.1 목적

현재 `dashboard/src/lib/latencyDiagnostics.ts` 하나에 command arm, clock sample,
영상 rolling timing과 export가 함께 들어 있다. 이를 방향별 trial 상태와 공통
결과 schema로 분리해 UI가 비동기 작업 순서를 직접 추측하지 않게 한다.

### 1.2 추가/수정 파일

```text
dashboard/src/lib/latency/types.ts                         추가
dashboard/src/lib/latency/statistics.ts                    추가
dashboard/src/lib/latency/downlinkTrial.ts                 추가
dashboard/src/lib/latency/uplinkTrial.ts                   추가
dashboard/src/lib/latency/latencyApi.ts                    추가
dashboard/src/lib/latencyDiagnostics.ts                    축소/호환 facade
dashboard/src/hooks/useLatencyDiagnostics.ts               수정
dashboard/test/latencyTrialState.test.ts                   추가
dashboard/test/run.ts                                      수정
```

기존 import를 한 번에 모두 깨지 않도록 `latencyDiagnostics.ts`는 새 store를
감싸는 facade로 남기고 UI 전환이 끝난 뒤 불필요한 legacy API를 제거한다.

### 1.3 공통 타입

```ts
type TrialDirection = 'downlink' | 'uplink'

type TrialPhase =
  | 'idle'
  | 'checking_clocks'
  | 'preparing'
  | 'waiting_for_neutral'
  | 'waiting_for_input'
  | 'waiting_for_trace'
  | 'warming_streams'
  | 'capturing'
  | 'uploading'
  | 'analyzing'
  | 'restoring_streams'
  | 'completed'
  | 'failed'
  | 'cancelled'

type Distribution = {
  count: number
  minMs: number | null
  meanMs: number | null
  p50Ms: number | null
  p95Ms: number | null
  p99Ms: number | null
  maxMs: number | null
}

type SegmentResult = {
  id: string
  label: string
  boundaryStart: string
  boundaryEnd: string
  distribution: Distribution
}
```

Downlink는 한 번의 controller input이므로 각 segment의 `count=1` distribution을
만들거나 별도 scalar를 중복 저장하지 않는다. UI에서는 단일 표본일 때 하나의
ms 값으로 표시하고, 향후 반복 trial 집계 시 같은 schema를 사용할 수 있게 한다.

### 1.4 Store 불변 조건

- `activeDownlinkTrial`과 `activeUplinkTrial`을 별도로 가진다.
- 방향별로 마지막 성공 결과와 마지막 실패를 보존한다.
- terminal phase에서 늦게 도착한 event는 결과를 변경하지 않는다.
- trial ID가 다른 gateway/rover/browser event는 현재 trial에 섞지 않는다.
- phase 전이는 명시적인 reducer/event로만 수행한다.
- `failed`에는 machine-readable `errorCode`와 사용자용 `message`를 함께 둔다.
- 결과 export schema는 `schema_version: 2`로 올리고 측정 경계를 문자열로
  포함한다.

### 1.5 구현 순서

1. 순수 percentile 함수와 빈 배열/null 규칙을 옮긴다.
2. 공통 trial/result/error 타입을 만든다.
3. Downlink reducer와 Uplink reducer를 별도 작성한다.
4. 기존 singleton의 subscribe/getSnapshot 계약을 유지한 facade를 만든다.
5. legacy command/video test를 새 event API에 맞게 옮긴다.
6. invalid phase transition, duplicate event, stale trial ID와 timeout을 테스트한다.

### 1.6 완료 조건

- 브라우저나 ROS 없이 reducer 단위 테스트가 실행된다.
- Downlink 실패가 Uplink 결과를 지우지 않고 반대 방향도 동일하다.
- 각 결과에 segment breakdown과 total segment가 모두 존재한다.
- total-only 결과 object를 생성할 수 없도록 parser/validator가 거부한다.

## 2. Chrony 및 브라우저-베이스 clock readiness

### 2.1 목적

현재 base chrony 상태는 버튼을 누를 때 한 번만 가져오기 때문에 5초 freshness
조건이 지나면 UI가 다시 blocked가 된다. rover 상태는 2초마다 발행되므로 base도
같은 방식으로 갱신하고, trial 시작과 종료에서 clock 품질을 다시 검증한다.

### 2.2 수정 파일

```text
dashboard/src/lib/chronyStatus.ts
dashboard/src/hooks/useLatencyClockReadiness.ts              추가
dashboard/src/components/LatencyDiagnosticsPanel.tsx
dashboard/test/chronyStatus.test.ts
base/gateway/src/runtime/chrony_status.js
base/gateway/test/chrony_status.test.js
rover/ros2_ws/src/mr2_system_status/mr2_system_status/system_status_node.py
rover/ros2_ws/src/mr2_system_status/test/test_chrony_status.py
```

### 2.3 Background readiness 동작

- latency panel mount 시 rover `/system_status/clock` subscription을 시작한다.
- base `/latency/clock-status`는 2초마다 polling한다.
- `Check chrony sync`는 다음 scheduled poll을 기다리지 않고 즉시 base fetch를
  수행하고 가장 최근 rover sample로 평가한다.
- panel unmount 또는 page hidden 상태에서는 불필요한 polling을 중지하거나
  간격을 늘린다. 측정 중에는 page visibility와 무관하게 2초 간격을 유지한다.
- HTTP 오류는 마지막 값을 ready로 유지하지 않고 stale/error 상태로 전환한다.

### 2.4 Freshness 판정

다음 두 age를 구분한다.

```text
sample_age  = browser_now - sampled_at_epoch_ms
receive_age = browser_now - received_at_epoch_ms
```

base/rover 시계가 동기화된 뒤에는 sample timestamp도 비교할 수 있다. 두 값 중
하나라도 허용 범위를 넘으면 stale로 처리한다. 미래 시각이 큰 폭으로 관측되어도
clock/config 오류로 차단한다.

기존 기준을 유지한다.

- status age 최대 5초
- rover absolute system offset 최대 2 ms
- rover `root_dispersion + abs(root_delay)/2` 최대 2 ms
- base와 rover 모두 synchronized

### 2.5 Browser-base offset

Downlink D0와 Uplink U2/U3를 base epoch으로 변환하기 위해 기존
`/latency/time` NTP-style sampling을 trial 직전에 자동 실행한다.

1. 기본 8개 sample을 얻는다.
2. server processing time을 제외한 RTT를 계산한다.
3. RTT가 낮은 절반을 선택한다.
4. offset과 RTT median을 사용한다.
5. `sampledAt`, sample count와 RTT 분포를 trial clock snapshot에 저장한다.

측정 종료 후 짧은 second sample을 얻어 offset drift를 확인한다. 시작/종료
offset 차이가 설정된 허용치보다 크면 전체값을 invalid로 표시한다. 허용치는
초기 기본값 2 ms로 두고 상수와 테스트에서 한 곳에서 관리한다.

### 2.6 Trial clock snapshot

각 trial은 다음을 고정 저장한다.

```json
{
  "base": { "sampled_at_epoch_ms": 0, "root_error_bound_ms": 0 },
  "rover": { "sampled_at_epoch_ms": 0, "root_error_bound_ms": 0 },
  "browser_to_base": {
    "offset_us": 0,
    "rtt_us": 0,
    "sample_count": 8,
    "sampled_at_epoch_us": 0
  }
}
```

UI가 나중에 갱신되어도 이미 끝난 결과의 clock snapshot은 바뀌지 않는다.

### 2.7 테스트와 완료 조건

- fake timer로 base polling이 readiness를 5초 이상 유지하는지 검증한다.
- base/rover missing, stale, unsynchronized, offset 초과와 error-bound 초과를
  각각 검증한다.
- browser offset의 부호를 synthetic timestamp로 고정한다.
- trial 도중 clock이 stale 또는 unsynchronized가 되면 partial 결과를 성공으로
  표시하지 않는다.
- 기존 chrony runtime 변경 파일에 있는 사용자 작업을 보존하며 통합한다.

## 3. Downlink 자동 측정

### 3.1 재사용할 현재 경로

- Dashboard D0: controller vector가 threshold를 처음 넘는 시각
- Gateway D1: `encodeDashboardDriveMessage()` 호출 직전 수신 시각
- Rover D2: XBEE frame decode 직후 `rover_rx_epoch_us`
- Rover D3: `/base/cmd_vel` publish 직후 `cmd_publish_epoch_us`
- Correlation: `(xbee_seq, wire_timestamp_ms)`와 `trial_id`

wire payload는 변경하지 않는다. trial metadata는 현재처럼 dashboard-gateway
WebSocket message와 gateway/rover diagnostic trace에서만 전달한다.

### 3.2 수정 파일

```text
dashboard/src/lib/latency/downlinkTrial.ts
dashboard/src/lib/latencyDiagnostics.ts
dashboard/src/hooks/useLatencyDiagnostics.ts
dashboard/src/components/LatencyDiagnosticsPanel.tsx
dashboard/test/downlinkTrial.test.ts                         추가
base/gateway/src/app/create_gateway_app.js
base/gateway/test/gateway_latency_trace.test.js
rover/ros2_ws/src/mr2_xbee_bridge/src/xbee_bridge_node.cpp
rover/ros2_ws/src/mr2_xbee_bridge/README.md
```

### 3.3 Button 실행 순서

```text
press Measure downlink
 -> reject if another downlink trial is active
 -> refresh chrony readiness
 -> measure browser/base offset
 -> enter waiting_for_neutral
 -> observe neutral controller sample
 -> enter waiting_for_input
 -> observe first deliberate non-neutral input (D0)
 -> attach trial_id/client_tx_epoch_us to next cmd_drive
 -> ingest gateway D1 trace
 -> ingest rover D2/D3 trace
 -> calculate four segment results
 -> verify end clock snapshot
 -> completed or failed
```

### 3.4 Input과 timeout 규칙

- neutral threshold는 기존 `DRIVE_TRIGGER_THRESHOLD=0.02`를 유지한다.
- neutral을 먼저 보지 않은 non-neutral input은 trial을 시작하지 않는다.
- D0를 만든 뒤 첫 tagged `cmd_drive` 하나만 측정한다.
- deadman zero publish와 heartbeat는 trial event로 취급하지 않는다.
- neutral 대기, user input 대기와 rover trace 대기에 각각 timeout을 둔다.
- 기본값은 상수로 두고 UI 문구가 아닌 설정/코드에서 변경 가능하게 한다.
- timeout 후 늦게 온 trace는 같은 trial ID여도 결과에 반영하지 않는다.

### 3.5 Segment 계산 검증

다음 불변 조건을 확인한다.

```text
D1 >= corrected(D0)
D2 >= D1
D3 >= D2
downlink_total ~= sum(three component segments)
```

음수 segment나 합계 오차가 clock uncertainty보다 크면 sample을 숨기지 않고
invalid와 raw timestamp를 결과에 기록한다. 사용자가 정상값으로 오해할 수 있는
성공 색상은 사용하지 않는다.

### 3.6 결과 표시 계약

Downlink 완료 시 네 row를 항상 함께 제공한다.

1. Browser input -> base gateway
2. Base gateway -> rover receive
3. Rover receive -> command publish
4. Total browser input -> command publish

trace가 하나라도 없으면 total만 계산하거나 추정하지 않는다.

### 3.7 테스트와 완료 조건

- rover trace가 gateway trace보다 먼저 도착해도 correlation된다.
- 8-bit XBEE sequence wrap과 서로 다른 wire timestamp가 구분된다.
- duplicate trace는 첫 유효 stage만 기록한다.
- neutral/input/trace timeout과 cancel이 terminal phase로 이동한다.
- diagnostics disabled 상태의 command wire bytes와 publish 동작이 기존과 같다.
- 실제 joystick으로 한 번 조작했을 때 UI 네 row가 모두 표시된다.

## 4. 로버 Uplink 측정 에이전트와 영상 stream 제어

### 4.1 새 ROS package

다음 package를 추가한다.

```text
rover/ros2_ws/src/mr2_latency_msgs/                       custom msg/srv
rover/ros2_ws/src/mr2_latency_diagnostics/                Python agent
```

예상 package 내부 파일:

```text
mr2_latency_msgs/CMakeLists.txt
mr2_latency_msgs/package.xml
mr2_latency_msgs/msg/UplinkTrialStatus.msg
mr2_latency_msgs/srv/PrepareUplinkTrial.srv
mr2_latency_msgs/srv/StartUplinkTrial.srv
mr2_latency_msgs/srv/CancelUplinkTrial.srv
mr2_latency_msgs/srv/AcquireVideoStreamLease.srv
mr2_latency_msgs/srv/ReleaseVideoStreamLease.srv

mr2_latency_diagnostics/package.xml
mr2_latency_diagnostics/setup.py
mr2_latency_diagnostics/mr2_latency_diagnostics/agent_node.py
mr2_latency_diagnostics/mr2_latency_diagnostics/capture.py
mr2_latency_diagnostics/mr2_latency_diagnostics/uploader.py
mr2_latency_diagnostics/mr2_latency_diagnostics/trial_state.py
mr2_latency_diagnostics/test/test_capture.py
mr2_latency_diagnostics/test/test_trial_state.py
```

`mr2_latency_msgs`에는 최소한 다음 interface를 둔다.

```text
srv/PrepareUplinkTrial.srv
srv/StartUplinkTrial.srv
srv/CancelUplinkTrial.srv
srv/AcquireVideoStreamLease.srv
srv/ReleaseVideoStreamLease.srv
msg/UplinkTrialStatus.msg
```

### 4.2 ROS service 계약

`PrepareUplinkTrial` request:

```text
string trial_id
string[] stream_ids
```

response:

```text
bool accepted
string message
```

`StartUplinkTrial` request:

```text
string trial_id
float64 duration_s
string upload_base_url
string upload_token
```

response는 process 완료를 기다리지 않고 validation과 spawn 성공 여부만
반환한다. 진행/완료는 `UplinkTrialStatus`로 발행한다.

`CancelUplinkTrial`은 trial ID가 일치할 때만 capture 중지와 stream lease 해제를
수행한다.

### 4.3 Agent 상태 머신

```text
idle
 -> prepared
 -> capturing
 -> uploading_metadata
 -> uploading_pcap
 -> restoring_streams
 -> completed

any active phase
 -> restoring_streams
 -> failed | cancelled
```

한 번에 하나의 prepared/active trial만 허용한다. status message에는 token이나
credential을 절대 포함하지 않는다.

### 4.4 Stream lease

현재 영상 node는 stream별 `SetBool`만 제공해 측정 전 상태를 안전하게 복원할
수 없다. `mr2_video_streaming`에 batch lease를 구현한다.

`AcquireVideoStreamLease`는 다음을 원자적으로 수행한다.

1. owner trial ID와 requested stream ID를 검증한다.
2. 현재 enabled stream ID snapshot을 저장한다.
3. requested stream만 enable하고 나머지를 disable한다.
4. lease owner와 expiry를 기록한다.
5. effective/previous stream 목록을 response로 반환한다.

`ReleaseVideoStreamLease`는 owner가 일치할 때 snapshot을 복원한다. agent crash에
대비해 lease watchdog timeout에서도 자동 복원한다. lease가 활성화된 동안
기존 개별 `set_enabled` service는 명시적 busy 오류를 반환한다.

수정 파일:

```text
rover/ros2_ws/src/mr2_video_streaming/src/video_streaming_node.cpp
rover/ros2_ws/src/mr2_video_streaming/CMakeLists.txt
rover/ros2_ws/src/mr2_video_streaming/package.xml
rover/ros2_ws/src/mr2_launch/launch/video_streaming.launch.py
rover/ros2_ws/src/mr2_launch/launch/rover.launch.py
rover/ros2_ws/src/mr2_launch/launch/rover_real.launch.py
```

### 4.5 Rover capture

Agent는 validated argument array로 `tcpdump`를 실행한다.

```text
tcpdump -i <rover_rocket_interface> -n -U -s 192 -B 4096
  --time-stamp-precision=nano -w <trial>.pcap
  udp and (dst port <selected-port> ...)
```

- interface는 launch argument 또는 `MR2_ROVER_ROCKET_INTERFACE`에서 받는다.
- config의 stream ID -> UDP port만 BPF에 사용한다.
- duration은 허용 범위 안으로 제한한다.
- output은 agent가 만든 전용 artifact directory 아래에만 쓴다.
- 기존 파일을 덮어쓰지 않고 trial ID를 path component로 직접 사용하지 않는다.
- SIGINT로 tcpdump를 종료해 pcap을 flush한다.
- exit code와 stderr의 captured/received/kernel dropped packet 수를 metadata로
  남긴다.

### 4.6 Artifact upload

측정 window가 끝난 뒤에만 베이스로 전송한다. 따라서 pcap upload 트래픽은
측정 window에 포함되지 않는다.

```text
PUT <base>/latency/uplink/trials/<id>/rover-metadata   application/json
PUT <base>/latency/uplink/trials/<id>/rover-capture  application/vnd.tcpdump.pcap
Authorization: Bearer <trial-scoped upload token>
```

pcap은 메모리에 전부 읽지 않고 file stream으로 전송한다. 일시 오류는 제한된
횟수와 backoff로 재시도한다. 최종 upload 실패 시 local artifact를 즉시 지우지
않고 status에 경로가 아닌 opaque artifact ID만 남긴다.

upload token은 짧은 expiry를 가지며 metadata/capture endpoint에만 사용할 수 있다.
각 endpoint는 한 번의 성공 upload만 허용하고 두 artifact가 모두 도착하거나
trial이 terminal 상태가 되면 token을 폐기한다.

### 4.7 Launch와 권한

- `enable_latency_diagnostics:=true`일 때만 agent를 launch한다.
- tcpdump 실행 권한은 agent 전체를 root로 실행하지 않고 one-time install
  script에서 필요한 capture capability를 설정한다.
- install script는 실제 tcpdump path를 resolve하고 현재 capability를 출력한 뒤
  변경한다.
- diagnostics가 false면 service, capture process와 stream lease가 존재하지 않는다.

추가 파일과 launch parameter:

```text
scripts/latency/install_capture_permissions.bash

capture_interface
video_config_path
artifact_directory
max_capture_duration_s
stream_lease_timeout_s
```

installer는 `--check` dry-run을 지원해 실제 변경 전에 binary와 capability를
검증할 수 있게 한다.

### 4.8 테스트와 완료 조건

- ROS와 tcpdump를 fake로 둔 순수 Python state-machine test를 작성한다.
- invalid trial ID, stream, interface, duration, duplicate start를 거부한다.
- prepare/start/cancel/timeout/upload 실패마다 stream snapshot이 복원된다.
- BPF에는 선택한 N개 port만 들어간다.
- pcap upload는 capture process 종료 뒤 시작한다.
- ROS package build/test와 launch-file compile이 통과한다.

## 5. 베이스 Uplink coordinator와 캡처/분석 작업

### 5.1 추가/수정 파일

```text
base/gateway/src/latency/uplink_trial_manager.js           추가
base/gateway/src/latency/capture_process.js                추가
base/gateway/src/latency/artifact_store.js                 추가
base/gateway/src/runtime/http_handlers.js                  수정
base/gateway/src/app/create_gateway_app.js                 수정
base/gateway/src/config.js                                 수정
base/gateway/test/uplink_trial_manager.test.js             추가
base/gateway/test/http_handlers.test.js                    수정
scripts/latency/analyze_rtp_latency.py                     확장
scripts/latency/test_rtp_latency.py                        확장
```

### 5.2 Gateway 설정

```text
MR2_LATENCY_DIAGNOSTICS_ENABLE
MR2_BASE_ROCKET_INTERFACE
MR2_LATENCY_ARTIFACT_DIR
MR2_LATENCY_PUBLIC_BASE_URL
MR2_LATENCY_CAPTURE_DURATION_S
MR2_LATENCY_CAPTURE_EDGE_MARGIN_S
MR2_LATENCY_MAX_UPLOAD_BYTES
MR2_LATENCY_PYTHON
```

필수 설정이 없으면 gateway normal 기능은 시작하되 Uplink endpoint preflight가
`not_configured`를 반환한다. 잘못된 interface로 임의 fallback하지 않는다.

### 5.3 REST API

#### Trial 생성

```text
POST /latency/uplink/trials
```

request:

```json
{
  "feed_count": 2,
  "stream_ids": ["rgbd_camera", "top_cam"],
  "duration_s": 15
}
```

이 endpoint는 trial과 artifact directory만 만들고 아직 capture를 시작하지 않는다.
server가 생성한 opaque trial ID를 반환한다.

#### Capture 시작

```text
POST /latency/uplink/trials/<id>/start
```

request:

```json
{
  "browser_clock": {
    "offset_us": 120.0,
    "rtt_us": 800.0,
    "sampled_at_epoch_us": 0
  },
  "chrony_snapshot": {}
}
```

start endpoint는 clock snapshot freshness를 검증하고 base tcpdump를 먼저 spawn한
뒤 rover upload URL과 trial-scoped token을 반환한다. 브라우저는 이 값을
`StartUplinkTrial` ROS service로 전달한다.

#### Rover artifact

```text
PUT /latency/uplink/trials/<id>/rover-metadata
PUT /latency/uplink/trials/<id>/rover-capture
```

token, content type, content length와 maximum size를 검증한다. body는 `.part`에
streaming하고 완료 후 atomic rename한다. upload가 끊기면 partial file을
제거한다.

#### Browser samples

```text
POST /latency/uplink/trials/<id>/browser-samples
```

sample은 다음 필드를 가진다.

```json
{
  "stream_id": "top_cam",
  "ssrc": 1,
  "rtp_timestamp": 90000,
  "marker_sequence": 101,
  "browser_receive_epoch_us": 0,
  "browser_render_epoch_us": 0
}
```

허용 stream, capture 시간 범위, 숫자 범위, sample count와 body size를 검증한다.

#### 상태와 취소

```text
GET    /latency/uplink/trials/<id>
DELETE /latency/uplink/trials/<id>
```

GET은 phase, progress, error와 완료된 report를 반환한다. DELETE는 local capture와
분석 process를 중지하고 terminal `cancelled`로 전환한다.

### 5.4 Coordinator 상태 머신

```text
created
 -> base_capturing
 -> waiting_for_rover_artifacts_and_browser_samples
 -> stopping_base_capture
 -> analyzing
 -> completed

any active phase -> cleaning_up -> failed | cancelled
```

base capture는 rover보다 먼저 시작하고 rover artifact 완료 뒤 정지한다. 별도
edge timeout으로 rover 시작 실패 시 base capture가 무기한 실행되지 않게 한다.

### 5.5 Artifact 안전성

- trial directory는 `mkdtemp` 또는 random server ID로 생성한다.
- HTTP path의 ID를 filesystem path에 직접 이어 붙이지 않는다.
- raw video payload를 JSON에 복사하지 않는다.
- 성공 결과는 report와 metadata를 보존하고 pcap 보존 기간은 configurable하게
  둔다.
- startup 시 오래된 `.part`와 expired trial을 정리한다.
- 한 trial 실패가 gateway process를 종료하지 않는다.

### 5.6 Analyzer 확장

기존 analyzer에 browser sample input을 추가한다.

```text
--browser-samples <samples.json>
--browser-clock-offset-us <base-browser>
```

분석기는 다음 순서로 처리한다.

1. rover/base pcap의 packet match를 기존 방식으로 수행한다.
2. matched marker packet만 frame candidate로 만든다.
3. browser sample을 marker key로 연결한다.
4. 각 동일 frame에서 네 Uplink segment를 계산한다.
5. stream별 distribution과 모든 stream을 합친 aggregate distribution을 만든다.
6. packet loss와 frame match/drop 통계를 별도로 기록한다.

aggregate percentile은 모든 유효 frame sample을 합친 배열에서 계산한다. stream별
percentile 평균은 사용하지 않는다.

### 5.7 Report schema

```json
{
  "schema_version": 2,
  "kind": "mr2_automated_uplink_latency_report",
  "trial_id": "...",
  "feed_count": 2,
  "stream_ids": ["..."],
  "clock": {},
  "aggregate": {
    "rocket_m2": {},
    "base_to_browser": {},
    "decode_render": {},
    "total": {}
  },
  "streams": [
    {
      "stream_id": "...",
      "segments": {},
      "matched_frames": 0,
      "packet_loss_percent": 0,
      "warnings": []
    }
  ],
  "warnings": []
}
```

네 segment 중 하나라도 report에 없으면 dashboard parser가 report를 거부한다.

### 5.8 테스트와 완료 조건

- fake child process로 base-start-before-rover 순서를 검증한다.
- 한 active trial 제한, timeout, cancel과 process cleanup을 검증한다.
- upload token mismatch, oversized/chunked body와 interrupted upload를 검증한다.
- synthetic pcap/browser sample로 각 segment와 total의 정확한 p50/p95/p99를
  검증한다.
- total percentile이 component percentile 합이 아님을 별도 test로 고정한다.
- Node gateway test와 Python analyzer test가 통과한다.

## 6. RTP 패킷과 브라우저 렌더 프레임의 상관관계

### 6.1 현재 gap

현재 base receiver는 RTP jitter/depay/H.264 parse 뒤 Annex-B access unit만 Node에
전달한다. 그 시점에 SSRC, sequence와 RTP timestamp가 사라지고 gateway epoch만
WebSocket chunk에 들어간다. 따라서 현재 정보만으로는 rover pcap의 frame과
브라우저 canvas frame을 정확하게 매칭할 수 없다.

### 6.2 수정 파일

```text
base/gateway/src/video/receiver.js
base/gateway/src/video/rtp_stream_parser.js                 추가
base/gateway/src/video/service.js
base/gateway/src/video/protocol.js
base/gateway/test/video_receiver.test.js
base/gateway/test/video_protocol.test.js
dashboard/src/lib/videoProtocol.ts
dashboard/src/lib/videoGateway.ts
dashboard/src/components/VideoStreamCard.tsx
dashboard/test/videoProtocol.test.ts                       추가
```

### 6.3 GStreamer metadata branch

`rtpjitterbuffer` 뒤에서 tee하여 depay branch와 RTP metadata branch가 동일하게
정렬되고 동일하게 drop된 패킷을 보게 한다.

```text
udpsrc
 ! rtpjitterbuffer
 ! tee name=rtp

rtp. ! queue ! rtpstreampay ! fdsink fd=3
rtp. ! queue ! rtph264depay ! h264parse ! fdsink fd=1
```

child process는 stdout(fd 1)의 Annex-B access unit과 fd 3의 length-prefixed RTP를
동시에 읽는다. `rtp_stream_parser.js`는 RTP v2 header, CSRC와 extension을
검증하고 SSRC/sequence/timestamp/marker만 보존한다.

Jetson의 설치된 GStreamer에서 `rtpstreampay`가 없거나 fd framing이 예상과
다르면 이 단계를 진행하지 않고 먼저 dependency를 설치한다. 임의 stdout log
파싱으로 대체하지 않는다.

### 6.4 Access unit pairing

- metadata branch에서 marker packet을 stream별 queue에 넣는다.
- Annex-B parser가 access unit을 완성하면 가장 오래된 marker와 짝짓는다.
- 두 fd의 read callback 순서는 보장되지 않으므로 양쪽 queue가 준비된 뒤 emit한다.
- queue 크기와 pairing timeout을 제한한다.
- marker 없이 access unit만 남거나 반대 상황이면 correlation warning과 counter를
  증가시키고 그 frame은 end-to-end sample에서 제외한다.
- receiver restart 때 두 queue를 모두 비운다.

### 6.5 WebSocket protocol v2 chunk

기존 config message와 legacy chunk parser는 유지하고 correlation metadata가 있는
새 message type을 추가한다.

```text
u8   message_type = TIMED_CHUNK
u16  stream_id_length
u8   flags
u64  decode_timestamp_us
u32  ssrc
u32  rtp_timestamp
u16  marker_sequence
u64  base_access_unit_epoch_us
u32  payload_length
...  stream_id
...  H.264 access unit
```

`decode_timestamp_us`는 stream별로 단조 증가하고 중복되지 않게 만든다.
`base_access_unit_epoch_us`는 기존 live 보조 진단과 디버깅을 위해 유지하지만
Uplink `base_to_browser`의 시작점은 pcap U1이다.

### 6.6 Browser render correlation

`videoGateway`는 WebSocket message callback entry에서 U2를 기록한다.
`VideoStreamCard`는 decoder에 chunk를 넣기 전에
`decode_timestamp_us -> RTP frame key + U2` map을 만든다.

WebCodecs output frame의 timestamp로 map을 찾고 `drawImage()` 직후 U3를 기록한다.
다음 경우 sample을 제외하고 counter를 올린다.

- decoder queue 정책으로 chunk가 drop됨
- output timestamp mapping 없음
- canvas/context 없음
- frame이 trial capture window 밖임
- stream 또는 trial ID 불일치

sample map은 stream별 상한을 두고 decoder reset, unsubscribe와 trial 종료 때
정리한다.

### 6.7 호환성과 정상 영상 검증

- 새 dashboard는 legacy chunk와 timed chunk를 모두 decode한다.
- 새 gateway는 correlation 실패 frame을 영상에서 버리지 않는다. 영상은 legacy
  metadata로 계속 전달하되 latency sample에서만 제외한다.
- protocol 변경으로 normal live view의 keyframe bootstrap, reconnect와 buffered
  amount drop 정책이 바뀌지 않아야 한다.

### 6.8 테스트와 완료 조건

- synthetic length-prefixed RTP에서 marker/SSRC/timestamp를 파싱한다.
- fd 1과 fd 3 event 순서를 뒤집어도 같은 access unit과 marker가 매칭된다.
- missing marker, duplicate marker, receiver restart와 queue overflow를 검증한다.
- protocol binary layout을 byte offset 단위로 Node/TypeScript 양쪽에서 검증한다.
- WebCodecs fake로 receive -> decode -> canvas draw sample이 동일 frame key를
  보존하는지 검증한다.
- 정상 영상 화면과 diagnostics sample 생성이 서로 독립적으로 동작한다.

## 7. 세 버튼 UI, 결과 표시, 통합 검증과 배포

### 7.1 UI 구성

`LatencyDiagnosticsPanel`의 primary action은 다음 세 개만 둔다.

1. `Measure downlink latency`
2. `Measure uplink latency`
3. `Check chrony sync`

제거/이동 대상:

- `Arm command`: Downlink button 내부 단계로 흡수
- `Measure browser/base`: 각 trial preflight로 자동 실행
- `Import RTP report`: Uplink coordinator 자동 분석으로 제거
- `Export`, `Reset`: 결과 영역 secondary icon/menu로 이동

수정 파일:

```text
dashboard/src/components/LatencyDiagnosticsPanel.tsx
dashboard/src/components/ControlPanel/LatencyDiagnosticsPanel.css
dashboard/src/components/LatencyResultTable.tsx              추가
dashboard/src/components/UplinkMeasurementFeeds.tsx          추가
dashboard/src/hooks/useLatencyDiagnostics.ts
dashboard/src/lib/latency/latencyApi.ts
dashboard/src/lib/rtpLatencyReport.ts
dashboard/test/latencyDiagnostics.test.ts
dashboard/test/uplinkWorkflow.test.ts                        추가
dashboard/README.md
scripts/latency/README.md
```

### 7.2 Feed count control

- configured stream 수를 기준으로 `1..N`을 선택할 수 있는 numeric/select control을
  제공한다.
- idle 또는 terminal phase에서 자유롭게 변경할 수 있다.
- preparing부터 terminal phase까지 값을 잠근다.
- count에서 선택된 실제 stream 이름을 측정 시작 전에 표시한다.
- 선택된 feed 각각에 measurement decoder/canvas를 mount해 실제 브라우저 decode와
  draw 부하를 발생시킨다.
- 측정용 canvas는 사용자가 실제 영상을 확인할 수 있어야 하며 display:none이나
  decode 없는 hidden subscriber로 대체하지 않는다.

### 7.3 Uplink browser orchestration

```text
press Measure uplink
 -> determine N stream IDs
 -> create base trial
 -> call rover PrepareUplinkTrial
 -> mount N video feeds
 -> wait until every feed renders a fresh frame
 -> refresh chrony and browser/base offset
 -> POST base trial start (base capture starts)
 -> call rover StartUplinkTrial with one-time upload token
 -> collect correlated U2/U3 samples for the capture window
 -> observe rover status and poll base trial status
 -> POST browser samples
 -> wait for analyzer report
 -> rover restores stream lease
 -> display per-stream and aggregate segment tables
```

어느 단계에서 실패해도 가능한 경우 rover cancel service와 base DELETE를 모두
호출한다. browser unload는 `sendBeacon`에 의존하지 않고 양쪽 timeout cleanup을
최종 안전장치로 둔다.

### 7.4 진행 상태

Downlink 상태:

```text
Checking clocks -> Return controller to neutral -> Send one command
-> Waiting for rover trace -> Complete
```

Uplink 상태:

```text
Checking clocks -> Configuring N feeds -> Waiting for video
-> Capturing -> Uploading -> Analyzing -> Complete
```

현재 phase, elapsed time, selected feed count와 실패 원인을 표시한다. 진행 중
primary measurement button은 중복 실행되지 않게 하고 `Cancel`은 secondary
command로 제공한다.

### 7.5 결과 표시

Downlink table은 다음 네 항목을 항상 표시한다.

```text
Browser -> Gateway
Gateway -> Rover
Rover receive -> Command publish
Total
```

Uplink aggregate와 각 stream table은 다음 네 항목을 항상 표시한다.

```text
Rover RTP -> Base RTP (Rocket M2)
Base RTP -> Browser receive
Browser receive -> Decode/render
Total rover RTP -> Browser render
```

Uplink 각 row에는 count, p50, p95, p99와 max를 표시한다. packet loss, matched
packet/frame 수, kernel drop, clock uncertainty와 warning은 같은 결과에 함께
표시한다. total만 보여주거나 segment를 펼침 메뉴 뒤에 기본적으로 숨기지 않는다.

### 7.6 오류 문구와 invalid 결과

최소 error code:

```text
clock_not_ready
browser_clock_unreliable
rover_agent_unavailable
stream_configuration_failed
stream_warmup_timeout
capture_permission_denied
capture_process_failed
rover_upload_failed
browser_samples_missing
frame_correlation_insufficient
analysis_failed
trial_cancelled
```

사용자 메시지는 어느 host와 어느 단계가 실패했는지 포함한다. raw process stderr나
credential은 브라우저에 그대로 노출하지 않는다.

### 7.7 자동 검증 순서

```bash
python3 -m unittest scripts.latency.test_rtp_latency

cd base/gateway
npm test

cd ../../dashboard
npm run test:latency
npm run check

cd ../rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to mr2_latency_diagnostics mr2_video_streaming mr2_xbee_bridge
colcon test --packages-select \
  mr2_latency_msgs mr2_latency_diagnostics mr2_video_streaming mr2_xbee_bridge
colcon test-result --verbose

cd ../../..
python3 -m py_compile scripts/latency/*.py
git diff --check
```

### 7.8 실제 베이스/로버 검증

1. 두 host에서 tcpdump capability와 지정 interface를 확인한다.
2. chrony ready가 30초 이상 유지되는지 확인한다.
3. Downlink를 실행해 네 segment와 total이 모두 나오는지 확인한다.
4. Uplink 1 feed를 실행해 양쪽 pcap, browser frame과 report가 매칭되는지 확인한다.
5. 2, 3, 최대 허용 feed로 반복한다.
6. 각 run에서 selected stream ID, bitrate, packet loss, kernel drop과 latency
   distribution을 비교한다.
7. 측정 성공, 실패, browser refresh와 cancel 뒤 stream 상태가 복원되는지 확인한다.
8. diagnostics를 끄고 normal control/video 회귀를 확인한다.

### 7.9 배포 순서

mono-repo에서 모든 코드를 먼저 구현하고 같은 commit을 베이스와 로버가 pull한다.

1. 베이스/로버 모두 branch pull
2. 로버 ROS workspace build
3. 양쪽 one-time capture permission installer 실행
4. base gateway env에 base Rocket interface/public URL 설정
5. rover launch env/argument에 rover Rocket interface 설정
6. rover를 `enable_latency_diagnostics:=true`로 launch
7. base gateway와 dashboard restart
8. UI preflight 및 단계별 smoke test

gateway/dashboard만 먼저 배포해 새 UI가 구형 rover agent를 호출하는 상태를 만들지
않는다. API/schema version mismatch는 명시적인 incompatible 오류로 처리한다.

### 7.10 최종 완료 조건

- 사용자가 terminal command나 RTP report import 없이 두 방향 측정을 시작한다.
- Downlink 결과에 세 구간과 total이 모두 표시된다.
- Uplink 결과에 세 구간과 total이 aggregate 및 stream별로 모두 표시된다.
- feed count 변경이 실제 rover RTP 송신 stream 수와 browser render 수를 함께
  변경한다.
- Uplink total은 동일 marker frame의 U0와 U3에서 직접 계산된다.
- chrony 상태가 정상인 동안 5초 후 readiness가 임의로 사라지지 않는다.
- 실패/취소/timeout 뒤 capture process와 stream lease가 남지 않는다.
- diagnostics disabled 상태의 rover control과 video 동작이 회귀하지 않는다.
- 자동 테스트와 실제 2-host field checklist가 모두 통과한다.

## 단계 의존성과 권장 commit 경계

```text
1 common model
  -> 2 clock readiness
  -> 3 downlink workflow

1 common model
  -> 4 rover agent/stream lease
  -> 5 base coordinator/analyzer
  -> 6 RTP-frame correlation
  -> 7 UI/integration/deployment
```

권장 commit은 단계별 하나 이상으로 나눈다. 특히 6단계 protocol 변경은 gateway와
dashboard 변경을 같은 commit에 넣고, 4단계 ROS interface package는 구현 package보다
먼저 build 가능한 commit으로 둔다. 각 commit은 해당 단계 테스트가 통과하는
상태여야 하며 사용자가 이미 수정한 unrelated working-tree 파일을 포함하지 않는다.
