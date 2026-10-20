# BAG / MCAP 파일 구조 및 파싱 정리

Elios 드론 데이터에서 보이는 `.bag`, `.mcap` 파일을 분석하기 위한 별도 정리 문서.
포맷 기본 설명과 실제 sepoong 프로젝트 파싱 결과, ROSM 도구 구현 내용 정리.

- 갱신 기준: 2026-09-17
- 실제 분석 대상: `D:\Conda_envs\ROS\ROSM\sepoong`의 MCAP 4개
- 분석 산출물: `D:\Conda_envs\ROS\ROSM\outputs`
- 구현 코드: `D:\Conda_envs\ROS\ROSM\rosm_tools`

---

## 1. 요약

| 파일 | 한 줄 설명 |
|---|---|
| `.bag` | 엘리오스 데이터에서는 ROS1 bag v2 기반 센서·상태 메시지 기록 파일 |
| `.mcap` | ROS1, ROS2, Protobuf, JSON 등 여러 형식의 메시지를 담는 데이터 컨테이너 |

둘 다 일반 영상 재생용 파일이 아니라, 시간 정보가 붙은 메시지 기록 파일. 원시 센서값뿐 아니라 이미 계산된 위치·자세와 이벤트도 저장 가능(LiDAR, IMU, pose, TF 등 원시 센서 기록).

---

## 2. Elios 데이터의 bag / mcap 파일

Flyability 공식 문서 기준, Elios 3 2025.01 업데이트 이후 `.bag` 파일을 `.mcap`으로 대체. 확장자만으로 기체 모델 차이나 Inspector import 전후를 판정할 수 없음. [FARO Connect 안내](https://knowledge.flyability.com/third-party-software/faro-connect)

| 데이터 | 확인 내용 |
|---|---|
| `106-inspection-Undefined-01` | 2023년 데이터, Inspector4 계열 `.efly`, ROS1 `.bag` 포함 |
| `230-inspection-GC-3-HandyHandle` | 2023년 데이터, Inspector4 계열 `.efly`, ROS1 `.bag` 포함 |
| `Ouster REV7(128채널 LiDAR)_하수관로` | 기존 폴더 조사에서 Inspector5 프로젝트 구조와 `.mcap` 확인. 정확한 생성·import·export 이력은 폴더명만으로 확정 불가 |
| `sepoong` | 이번 도구 개발의 실제 분석 대상. MCAP 4개, ROS2 profile, robot metadata version `2026.02.0` |

---

## 3. `.bag` 파일

`.bag` 파일은 ROS bag 파일.
ROS topic message를 시간순으로 저장하고, 나중에 다시 읽거나 재생하기 위한 파일.

### 주요 특징

| 항목 | 내용 |
|---|---|
| 주 사용 환경 | ROS1 |
| Elios 데이터에서의 위치 | 구형 Elios 3 / Inspector4 계열 원시 기록 |
| 로컬 파일 예시 | `E300..._obcbag-001.bag`, `E300..._obcbag-002.bag` |
| 로컬 확인 헤더 | `#ROSBAG V2.0` |
| 주요 도구 | `rosbag info`, `rosbag play`, Python `rosbag` |

### 들어갈 수 있는 데이터

| 데이터 | 의미 |
|---|---|
| LiDAR 원시 데이터 | Ouster LiDAR packet 또는 point cloud 생성 전 데이터 |
| IMU | 가속도, 자이로 등 자세 추정 센서값 |
| Pose / Odometry | 드론 위치와 자세 추정값 |
| TF | body, imu, camera, lidar 등 좌표계 변환 |
| Camera 관련 topic | 이미지, 카메라 상태, 동기화 정보 가능 |
| System 상태 | 노드 상태, 장비 상태, 시간 동기화 정보 가능 |

### 내부 구조

ROS bag v2 계열은 여러 record로 구성.

| 구성 요소 | 의미 |
|---|---|
| File Header | bag 파일 전체 정보와 index 위치 |
| Connection | topic 이름, message type, md5, message definition |
| Message Data | 실제 시간별 ROS message |
| Chunk | 여러 message를 묶은 저장 블록 |
| Index Data | topic/time 기준 빠른 검색용 index |
| Chunk Info | chunk 시작/끝 시간, message 수 등 요약 |

ROS `rosbag_storage` 문서 기준 주요 opcode.

| opcode | 의미 |
|---|---|
| `OP_MSG_DATA = 0x02` | message data |
| `OP_FILE_HEADER = 0x03` | file header |
| `OP_INDEX_DATA = 0x04` | index data |
| `OP_CHUNK = 0x05` | chunk |
| `OP_CHUNK_INFO = 0x06` | chunk info |
| `OP_CONNECTION = 0x07` | topic connection |

### 분석 방법

| 목적 | 방법 |
|---|---|
| 파일 정보 확인 | `rosbag info <file.bag>` |
| 시간순 재생 | `rosbag play <file.bag>` |
| 특정 topic 추출 | Python `rosbag.Bag(...).read_messages(topics=[...])` |
| 포맷 확인 | 파일 첫 줄 `#ROSBAG V2.0` 확인 |

---

## 4. `.mcap` 파일

`.mcap` 파일은 timestamped pub/sub message 저장용 컨테이너.
ROS1, ROS2, Protobuf, JSON Schema 등 여러 serialization format 지원.

중요한 점.

```text
MCAP = 파일 컨테이너
MCAP = 무조건 ROS2 아님
```

따라서 내부 `profile`, `schema`, `channel`, `message_encoding` 확인 필요.

### 주요 특징

| 항목 | 내용 |
|---|---|
| 주 사용 환경 | ROS1/ROS2, Foxglove, 최신 로봇 데이터 파이프라인 |
| Elios 데이터에서의 위치 | Elios 3 2025.01 이후 `.bag` 대체 원시 기록 |
| 로컬 파일 예시 | `E3R..._obcbag.mcap` |
| 로컬 확인 magic | `89 4D 43 41 50 30 0D 0A` |
| 주요 도구 | `mcap info`, `mcap list`, `mcap cat`, Python `mcap` |

---

## 5. MCAP 기본 파일 구조

MCAP 공식 spec 기준 구조.

```text
<Magic>
<Header>
<Data section>
[Summary section]
[Summary Offset section]
<Footer>
<Magic>
```

| 구성 요소 | 의미 |
|---|---|
| Magic | MCAP 파일 식별용 고정 바이트, 파일 시작과 끝에 위치 |
| Header | profile, writer library 등 기본 정보 |
| Data section | schema, channel, message, chunk 등 실제 데이터 영역 |
| Summary section | 빠른 조회용 요약/index 영역, 선택 사항 |
| Summary Offset section | summary record 위치 검색용 영역, 선택 사항 |
| Footer | summary 위치, CRC 등 파일 마지막 정보 |

MCAP record 공통 구조.

```text
<record type><record content length><record content>
```

| 항목 | 의미 |
|---|---|
| record type | 1 byte opcode |
| record content length | uint64 길이 |
| record content | record별 실제 내용 |

---

## 6. MCAP 주요 record

| record | 의미 |
|---|---|
| Header | profile, writer library |
| Schema | message type / schema definition |
| Channel | topic 이름, schema id, message encoding |
| Message | 실제 시간별 message data |
| Chunk | 여러 record/message를 묶은 블록, 압축 가능 |
| Message Index | chunk 내부 message 위치 index |
| Chunk Index | chunk 위치, 시간 범위, 압축 정보 |
| Attachment | 별도 첨부 데이터 |
| Metadata | 파일/기록 관련 metadata |
| Statistics | message 수, channel 수, 시작/종료 시간 |
| Data End | Data section 종료 표시 |
| Footer | Summary 위치, CRC 등 EOF 정보 |

---

## 7. MCAP Message 구조

MCAP의 실제 센서 message는 `Message` record에 저장.

| 필드 | 의미 |
|---|---|
| `channel_id` | 어떤 channel/topic의 message인지 식별 |
| `sequence` | uint32 필드는 항상 존재. 순번을 사용하지 않는 경우 0 가능 |
| `log_time` | recorder가 기록한 시간, nanosecond timestamp |
| `publish_time` | publisher가 발행한 시간, nanosecond timestamp |
| `data` | 실제 message bytes, schema와 encoding 기준 decode 필요 |

---

## 8. Schema / Channel / Message 관계

MCAP 파싱에서 가장 먼저 이해할 관계.

```text
Schema
  -> message type 정의

Channel
  -> topic 이름
  -> schema_id
  -> message_encoding

Message
  -> channel_id
  -> data bytes
```

decode 순서.

```text
message.channel_id 확인
-> channel 조회
-> channel.schema_id 확인
-> schema 확인
-> message_encoding 확인
-> data bytes decode
```

---

## 9. Chunk / Index 구조

MCAP은 대용량 센서 로그를 빠르게 읽기 위해 chunk와 index 활용 가능.

| 항목 | 의미 |
|---|---|
| Chunk | 여러 message를 묶은 블록 |
| Compression | `zstd`, `lz4`, none 등 가능 |
| Message Index | chunk 안에서 특정 channel/time message 위치 검색 |
| Chunk Index | 파일 안에서 chunk 위치와 시간 범위 검색 |
| Summary | schema, channel, statistics, index 요약 |

읽기 방식.

| 방식 | 특징 |
|---|---|
| linear scan | 파일 처음부터 끝까지 순차 읽기 |
| indexed read | summary/index 활용, topic/time 범위 조회에 유리 |

---

## 10. `.bag`와 `.mcap` 차이

| 구분 | `.bag` | `.mcap` |
|---|---|---|
| 성격 | ROS bag 파일 | 최신 로봇 데이터 컨테이너 |
| 중심 환경 | ROS1 | ROS1, ROS2, Protobuf, JSON 등 |
| 파일 시작 | `#ROSBAG V2.0` | `89 4D 43 41 50 30 0D 0A` magic |
| topic 정보 | Connection record | Channel record |
| message type 정보 | type, md5, message definition | Schema record |
| 실제 message | Message Data record | Message record |
| 묶음 저장 | Chunk | Chunk |
| 빠른 조회 | Index Data, Chunk Info | Summary, Message Index, Chunk Index |
| 압축 | 지원 | `zstd`, `lz4`, none 등 |
| 변환 | MCAP으로 변환 가능 | `.bag`, `.db3`에서 변환 가능 |

핵심 차이.

```text
.bag
-> ROS1 중심 원시 센서 기록 포맷

.mcap
-> 여러 메시지 형식을 담는 최신 원시 센서 기록 컨테이너
```

---

## 11. 실제 분석 결과 요약

### 11.1 확인 범위

이하 수치는 `outputs`에 저장된 분석 결과와 원본 MCAP schema 재확인 기준.
목록·통계는 4개 파일 전체, 메시지 상세 구조는 관심 topic 샘플과 전량 추출 항목을 구분.

| 항목 | 확인 결과 |
|---|---|
| MCAP 파일 수 | 4개 |
| 전체 크기 | 3,973,931,898 bytes, 약 3.97 GB |
| 전체 메시지 수 | 2,714,164개 |
| Header profile | 4개 모두 `ros2` |
| 기록 라이브러리 | `libmcap 1.3.0` |
| Channel message encoding | `cdr` |
| Schema encoding | `ros2msg` |
| 파일별 등록 Channel / Schema | 각각 97개 / 54개 |
| 파일별 실제 메시지가 있는 Channel | 63개 또는 64개 |
| Chunk | 총 4,616개, MCAP Chunk 압축 없음 |
| Attachment | 4개 파일 모두 0개 |
| Metadata | 파일마다 `robot` 1개, `version=2026.02.0` |
| 스냅샷 이벤트 | 총 7건 |
| 기본 궤적 추출 | `/kalman_scan2map_node/odometry` 7,821건 |
| LiDAR 바이너리 색인 | `PacketBytes` 8,045건, `Mask2d` 8,045건 |

- 388개 Channel 행과 216개 Schema 행은 파일 4개의 합계. 서로 다른 topic 388개라는 의미는 아님
- Schema 또는 Channel의 등록과 실제 메시지 존재는 별개
- `robot.version`은 기록된 버전 문자열. 촬영 날짜나 Inspector 버전으로 대체 해석하지 않음
- 파일 수정 날짜도 실제 촬영 날짜로 단정하지 않음
- MCAP Chunk가 무압축이어도 내부 이미지나 LiDAR payload는 압축·별도 인코딩 가능

### 11.2 비행별 기록

식별자는 공통 접두사 `E3RDAA23400087_`과 아래 문자열의 결합.

| 비행 식별자 뒷부분 | 메시지 수 | 기록 시간 범위 길이 | 궤적 행 수 | 스냅샷 수 |
|---|---:|---:|---:|---:|
| `02008_338_1flight` | 120,438 | 약 35.701초 | 356 | 0 |
| `02010_339_1flight` | 388,820 | 약 114.704초 | 1,148 | 0 |
| `02011_340_1flight` | 860,194 | 약 254.612초 | 2,546 | 2 |
| `02012_342_1flight` | 1,344,712 | 약 399.526초 | 3,771 | 5 |

기록 시간 범위는 MCAP 첫·마지막 `log_time`의 차이. 실제 이륙~착륙 시간과 동일하다는 보장은 없음.

근거: `outputs/mcap/mcap_files_detail.csv`, `mcap_inventory_summary.json`,
`outputs/lidar/lidar_trajectory_summary.json`, `outputs/snapshots/snapshot_pose_summary.json`.

---

## 12. MCAP 안에 실제로 들어 있는 데이터

### 12.1 주요 topic과 메시지 수

아래 수치는 4개 MCAP 합계. `CameraState`, `Odometry`, `Imu` 등은 이름이 비슷해도 ROS 표준 메시지와 구분 필요.

| 데이터 | 실제 topic | Schema 또는 의미 | 메시지 수 |
|---|---|---|---:|
| 스냅샷 이벤트 | `/point_of_interest/snapshot` | `elios_ros_msgs/msg/Snapshot` | 7 |
| 기본 비행 위치·자세 | `/kalman_scan2map_node/odometry` | `elios_ros_msgs/msg/Odometry` | 7,821 |
| 고주기 위치·자세 | `/kalman_scan2map_node/fast_odometry` | 같은 Odometry 형식 | 160,903 |
| VIO 위치·자세 | `/vio/odometry` | 같은 Odometry 형식 | 24,137 |
| VIO 고주기 위치·자세 | `/vio/odometry_fast` | 같은 Odometry 형식 | 160,828 |
| LiDAR 기록 바이너리 | `/os_node/lidar_scan/compressed` | `elios_ros_msgs/msg/PacketBytes` | 8,045 |
| LiDAR 2D 마스크 | `/os_node/lidar_scan_mask2d` | `elios_ros_msgs/msg/Mask2d` | 8,045 |
| camera_0 이미지 | `/camera_0/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 24,137 |
| camera_1 이미지 | `/camera_1/image_raw/compressed` | 같은 CompressedImage 형식 | 24,135 |
| camera_2 이미지 | `/camera_2/image_raw/compressed` | 같은 CompressedImage 형식 | 24,136 |
| 카메라 상태 | `/camera_0/camera_state` ~ `/camera_2/camera_state` | 노출·gain·취득 시간 등 | 72,408 |
| 자세 포함 IMU | `/sensors/imu` | `elios_ros_msgs/msg/ImuWithOrientation` | 160,901 |
| LiDAR IMU | `/sensors/lidar_imu` | `elios_ros_msgs/msg/Imu` | 100,570 |
| VIO IMU | `/vio_module/imu` | 같은 Imu 형식 | 804,575 |
| 필터링 IMU | `/vio_module/imu_filtered` | 같은 Imu 형식 | 160,916 |
| 동적 좌표계 변환 | `/tf` | `tf2_msgs/msg/TFMessage` | 31,958 |
| 정적 좌표계 변환 | `/tf_static` | Channel만 등록, 실제 메시지 없음 | 0 |
| 거리 센서 | `/sensors/tof` | `elios_ros_msgs/msg/TimeOfFlight` | 20,571 |
| 서보 각도 | `/sensors/servo_angle` | `std_msgs/msg/Float32` | 87,319 |
| GNSS 관련 기록 | `/sensors/gnss` | `elios_ros_msgs/msg/GNSS` | 8,045 |
| 자기장 | `/sensors/mag` | `sensor_msgs/msg/MagneticField` | 80,452 |
| 비행 상태 | `/flight/flight_state` | `elios_ros_msgs/msg/FlightState` | 1,610 |

추가 등록·기록 항목: IMU bias, 조종 명령, 모터 속도, 통신 상태, 경고, VIO 건강 상태, 귀환 경로, 처리 시간 통계 등.

GNSS topic에 메시지가 있다는 사실만으로 유효한 GPS 위치 존재를 의미하지 않음.
거리 센서 topic과 snapshot의 `wall_distance`가 어떤 계산·동기화 관계인지도 별도 검증 대상.

전체 목록과 파일별 수치: `outputs/mcap/mcap_channels.csv`.

### 12.2 완성된 점군의 존재 여부

다음 `sensor_msgs/msg/PointCloud2` Channel은 등록됐지만 메시지 수 0.

- `/kalman_scan2map_node/initial_map`
- `/reloc/reference_chunk`
- `/reloc/source_chunk`

현재 확인한 MCAP에서는 이 topic들을 읽어 XYZ 점군을 얻을 수 없음.
현재 뷰어에 보이는 3D 점군은 프로젝트의 LAS에서 로드한 결과.

### 12.3 MCAP에 있다고 확인하지 못한 항목

| 항목 | 이번 데이터의 확인 결과 |
|---|---|
| 4000 x 3000 Xacti 스냅샷 JPG | 프로젝트의 별도 이미지 파일 사용. snapshot 메시지에 이미지 bytes 없음 |
| 4K 촬영 동영상 원본 | 프로젝트의 별도 미디어 파일. 확인한 MCAP 이미지 topic은 VIO 카메라 |
| 열화상 원본 | 확인한 MCAP topic 목록에서 열화상 이미지 스트림 미발견 |
| Xacti 4K `K/D/P` | 검토한 schema·snapshot·metadata·attachment에서 읽을 수 있는 보정값 미발견 |
| 프로젝트의 완성된 LAS 파일 | MCAP Attachment가 아닌 별도 LAS 파일 |

MCAP 포맷 자체가 사진·영상·보정값을 담을 수 없다는 의미는 아님.
미해석 LiDAR 바이너리 내부까지 모든 의미를 해독했다는 의미도 아님.

---

## 13. 스냅샷 위치·자세와 시간

### 13.1 실제 Snapshot 구조

원본 MCAP에 포함된 `elios_ros_msgs/msg/Snapshot` schema의 주요 필드.

```text
Snapshot
  stamp         : builtin_interfaces/Time
  id            : string
  poi_id        : string
  flight_id     : string
  criticality   : uint8
  camera_pose   : geometry_msgs/Pose
  camera_pitch  : float32
  drone_pose    : geometry_msgs/Pose
  wall_distance : float32

Pose
  position      : x, y, z
  orientation   : x, y, z, w
```

| 필드 | 활용 |
|---|---|
| `id` | 프로젝트 snapshot ID와 연결 |
| `poi_id`, `flight_id` | POI·비행 소속 연결 |
| `camera_pose` | 스냅샷 카메라 위치·자세, 현재 POI와 프러스텀의 기준 |
| `drone_pose` | 같은 이벤트의 드론 위치·자세 |
| `camera_pitch` | 카메라 pitch 별도 값. 현재 프러스텀에 중복 회전 적용하지 않음 |
| `wall_distance` | 거리 값. 양수일 때 프러스텀 길이 후보로 사용 |
| `criticality` | 중요도 코드, schema에 0~5 상수 정의 |
| `stamp` | 이벤트 내부 시간. MCAP의 `log_time`과 구분 |

스냅샷 이벤트는 촬영 정보 기록이며 JPG 이미지 자체는 아님.
`camera_pose`에는 이미 카메라 자세가 있으므로, 드론-카메라 외부 보정이나 pitch를 다시 적용하면 이중 적용 가능.

### 13.2 프로젝트 JSON 비교 결과

| 비교 항목 | 결과 |
|---|---|
| 프로젝트 snapshot JSON | 7개 |
| MCAP snapshot 이벤트 | 7개 |
| snapshot ID 매칭 | 7/7 |
| 최대 카메라 위치 차이 | 0 m |
| 최대 카메라 자세 차이 | 0 deg |
| snapshot 이벤트 stamp와 JSON 시간의 최대 절댓값 차이 | 40,882,352 ns, 약 40.88 ms |

이번 7건에서는 MCAP의 `camera_pose` 직접 사용 가능.
시간이 완전히 같아서가 아니라, snapshot ID로 대응시킨 카메라 pose 값이 일치한 결과.

현재 뷰어의 데이터 흐름.

```text
MCAP Snapshot.camera_pose / wall_distance
  -> mcap_snapshot_events.csv
  -> POI 위치, 카메라 축, 프러스텀 배치

프로젝트 snapshot JSON / POI JSON
  -> snapshot_json_poses.csv
  -> 같은 snapshot ID의 JPG 경로, 이미지 이름, POI 이름 연결
```

MCAP pose가 없는 경우 프로젝트 JSON pose로 자동 대체하는 동작은 현재 뷰어에 없음.
다른 프로젝트·펌웨어 또는 재정렬한 지도까지 같은 pose 일치를 보장하지 않음.

근거: `snapshot_mcap_matches.csv`, `snapshot_pose_summary.json`,
`rosm/snapshot_pose_index.py`, `rosm/lidar_viewer.py`.

### 13.3 비행 전체의 위치·자세

`elios_ros_msgs/msg/Odometry`는 ROS 표준 `nav_msgs/msg/Odometry`와 다른 사용자 정의 메시지.

```text
header
child_frame_id
pose          : geometry_msgs/Pose
twist         : geometry_msgs/Twist
confidence    : bool
```

- 기본 추출 topic: `/kalman_scan2map_node/odometry`
- 추출 항목: 시간, frame ID, 위치, quaternion, 속도·각속도, confidence 등
- CSV의 Euler 각도는 quaternion에서 계산한 파생값
- 스냅샷 최근접 odometry 비교에서 최대 시간 차이 약 37.06 ms, drone 위치 차이 약 0.017 m
- 최근접 odometry의 drone pose와 snapshot의 camera pose는 서로 다른 기준. 같은 값으로 취급하지 않음

### 13.4 시간 해석 주의

| 시간 | 의미 |
|---|---|
| `message.log_time` | MCAP 기록 시간 |
| `message.publish_time` | MCAP 발행 시간 필드 |
| Snapshot `stamp` | 스냅샷 이벤트 내부 시간 |
| Odometry `header.stamp` | 자세 메시지 내부 시간 |
| 프로젝트 JSON 시간 | Inspector가 관리하는 별도 시간 필드 |

로컬 MCAP에는 67.989초, 474.123초처럼 작은 초 단위 값도 존재.
UTC 촬영 날짜로 즉시 변환하지 말고 시간 기준부터 확인.
시간 계산은 정수 nanosecond 기준, 초 단위 실수는 표시용으로 제한.

---

## 14. LiDAR 바이너리와 jfly 분석 범위

### 14.1 PacketBytes

`/os_node/lidar_scan/compressed`의 실제 ROS2 schema.

```text
uint8[] buf
```

해독 단계는 세 층으로 구분.

```text
MCAP record 읽기                         -> 완료
  -> ROS2 CDR decode, PacketBytes.buf 추출 -> 완료
  -> buf 내부를 XYZ·intensity로 복원       -> 미완료
```

- 총 8,045개 메시지의 `buf` 앞부분에서 `jfly` 식별 바이트 확인
- 첫 4 bytes: `6a 66 6c 79`
- `jfly`는 관찰된 바이너리 식별 문자열. 내부 필드 배치와 디코딩 규칙은 미확정
- 일반 Ouster UDP packet이나 `PointCloud2`로 곧바로 취급할 수 없음
- 별도 해독 시도는 XYZ·intensity 복원에 이르지 못해 관련 probe 기능 제거
- 현재 유지 기능은 메시지 수, 크기, 시간, bytes 앞부분 등 색인·분석

파일명에 `compressed`가 있어도 구체적인 압축 알고리즘을 이름만으로 확정하지 않음.

### 14.2 Mask2d

```text
builtin_interfaces/Time stamp
uint16 rows
uint16 cols
uint32 reserved
uint8[] data
```

- schema 주석상 boolean을 1 bit에 저장한 2D 마스크
- 총 8,045개 메시지, 확인한 data 길이 4,096 bytes
- `rows`, `cols`, `data`를 이용한 마스크 구조
- 각 bit의 구체적인 선택·제외 의미와 scan 데이터 대응은 별도 검증 대상
- 마스크 자체는 XYZ 좌표나 intensity 배열이 아님

### 14.3 현재 LAS와 intensity의 출처

| 항목 | 실제 출처 |
|---|---|
| 뷰어의 XYZ 점군 | 프로젝트 `maps/.../*.las` |
| 점군 intensity | LAS의 `intensity` 필드 |
| 빠른 표시용 점군 | LAS에서 샘플링한 `outputs/lidar/preview_ply/*.ply` |
| 회색·높이 Z·intensity·Inspector 스타일 | 표시용 색상 변환 |
| Inspector 스타일 색상 | intensity 구간 기반 자체 구현. Inspector 내부 팔레트와 동일성 미검증 |

PLY preview는 LAS intensity를 정규화한 색상으로 보존.
원래 intensity 수치가 필요한 분석은 LAS를 기준으로 수행.
현재 MCAP의 `jfly` bytes에서 intensity를 해독한 결과는 없음.
MCAP만으로 LAS를 새로 생성하는 기능도 현재 미구현.

---

## 15. 카메라 종류와 4K 캘리브레이션

### 15.1 cam0~cam2와 스냅샷의 차이

| 구분 | MCAP camera_0~2 | Xacti 스냅샷 |
|---|---|---|
| 이미지 크기 | 디코딩한 샘플 기준 640 x 400 | 프로젝트 JPG 4000 x 3000 |
| 이미지 형식 | `mono8; jpeg compressed mono8` | 컬러 JPG |
| 용도 구분 | VIO 계열 카메라 스트림 | 검사 촬영용 사진 |
| 보정 파일 | `camera_calibration_0~2.yaml` 등 | Inspector의 Xacti CameraSpec |
| 현재 4K 프러스텀에 사용 | 사용하지 않음 | 사용 |

cam0~cam2 세 채널을 스냅샷 카메라 3대로 해석하지 않음.
`CameraState`의 실제 필드는 노출·gain·상태·시간 등이며, Xacti `K/D/P`가 아님.

### 15.2 보정값 출처

| 보정 정보 | 확보 경로 | 현재 활용 |
|---|---|---|
| VIO 카메라 내부 보정 | 프로젝트 `calib_files`의 YAML | 색인·분석 |
| IMU-camera / static / geoslam 변환 | 프로젝트 YAML | 좌표계 관계 정리 |
| 4000 x 3000 Xacti `K/D/P` | 로컬 Inspector 5 `robotspec.dll` 함수 호출 | 4K 왜곡 보정·프러스텀 |
| Snapshot 카메라 pose | MCAP Snapshot | 카메라 위치·방향 배치 |

확보한 함수: `flya::CameraSpecDatabase::xactiCamera(QSize)`

로컬 DLL:
`C:\Program Files\Flyability\Inspector 5\robotspec.dll`

저장 결과:
`outputs/calibration/xacti_4000x3000_camera_spec.json`

이는 설치된 Inspector의 CameraSpec에서 읽은 값.
MCAP에서 추출한 값이나 해당 기체 개별 실측 보정값이라고 단정하지 않음.
다른 Inspector 버전·카메라 모델에는 재확인 필요.

### 15.3 확보한 Xacti 값

아래는 가독성을 위해 반올림한 값. 계산에는 JSON의 전체 정밀도 사용.

```text
image_size = 4000 x 3000

K =
[1807.379659,    0.000000, 2071.817271]
[   0.000000, 1807.326508, 1154.223887]
[   0.000000,    0.000000,    1.000000]

D =
[0.0527161054, 0.0236137939, -0.0159405130, 0.0029765993]

P =
[1428.692945,    0.000000, 2129.714496]
[   0.000000, 1428.650930, 1029.394996]
[   0.000000,    0.000000,    1.000000]

lens_to_focal_point_distance_m = 0.02123
```

- `K`: 왜곡된 원본 이미지의 내부 행렬
- `D`: 현재 구현에서 사용하는 OpenCV fisheye 계수 4개
- `P`: 보정 출력 이미지의 투영 행렬
- `lens_to_focal_point_distance_m`: 추출해 저장한 값. 현재 snapshot pose에 추가 이동으로 적용하지 않음
- OpenCV fisheye 모델로 적용했을 때 `balance=0`의 P 계산값과 최대 약 0.0132 px 차이

보정 함수와 파라미터 정의: [OpenCV fisheye 문서](https://docs.opencv.org/doc/doxygen/html/d2/d14/namespacecv_1_1fisheye.html).

### 15.4 왜곡 보정·프러스텀·카메라 시점

```text
원본 JPG 픽셀 + K/D/P
  -> cv2.fisheye.initUndistortRectifyMap
  -> cv2.remap
  -> 보정 JPG

동일한 P + 이미지 크기
  -> 보정 이미지 네 모서리의 광선 방향
  -> MCAP camera_pose로 3D 배치
  -> 거리 기준으로 프러스텀 길이 결정
```

- 캘리브레이션이 있으면 처음부터 P 기반 프러스텀 생성
- 보정 체크박스는 원본·보정 사진 전환. 체크할 때마다 다른 프러스텀으로 바꾸는 구조는 아님
- 미보정 사진을 보정용 평면에 붙인 상태는 정확한 렌즈 투영 정합으로 보지 않음
- `wall_distance`가 0 또는 유효하지 않으면 선택 모드에 따라 LiDAR 추정 또는 고정 길이 사용
- Xacti 보정 JSON이 없으면 기존 FOV 기반 프러스텀으로 대체, 이미지 보정 비활성

현재 구현의 3D 광선은 local X 전방, local Y 측방, local Z 상방 기준.
Open3D 화면 좌표축과 이미지 좌표축의 차이는 투영 단계에서 처리.

---

## 16. Python 파싱 방법

### 16.1 현재 사용하는 라이브러리

환경: `D:\Conda_envs\ROS\python.exe`

| 라이브러리 | 역할 |
|---|---|
| `mcap` | MCAP Header·Summary·Channel·Message 읽기 |
| `mcap-ros2-support` / `mcap_ros2` | `ros2msg` schema와 `cdr` 메시지 decode |
| `PyYAML` | 프로젝트 보정 YAML 읽기 |
| `numpy` | 좌표·행렬·이미지 배열 처리 |
| `laspy` | 기존 LAS의 XYZ·intensity 읽기 |
| `open3d` | 점군·궤적·POI·사진 평면 표시 |
| `opencv-python` / `cv2` | JPEG decode, fisheye 왜곡 보정 |
| `Pillow` / `PIL` | EXIF 방향 정보 읽기·보정 JPG 저장 |
| 표준 `ctypes` | 설치된 Inspector DLL의 Xacti CameraSpec 읽기 |

ROS2 MCAP을 읽는 이번 도구에 ROS1 `mcap_ros1.DecoderFactory`를 적용하지 않음.
ROS 설치 없이도 내장 `ros2msg` schema를 이용해 현재 사용자 정의 메시지 decode 가능.
지원되는 schema·encoding 범위는 [MCAP ROS2 DecoderFactory 문서](https://mcap.dev/docs/python/mcap-ros2-apidoc/mcap_ros2.decoder) 참고.

### 16.2 최소 파싱 예시

```python
from pathlib import Path
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

project = Path(r"D:\Conda_envs\ROS\ROSM\sepoong")
mcap_path = next(project.glob("flights/**/*.mcap"))

with mcap_path.open("rb") as stream:
    reader = make_reader(stream, decoder_factories=[DecoderFactory()])
    print(reader.get_header())

    summary = reader.get_summary()
    if summary is not None:
        for channel in summary.channels.values():
            schema = summary.schemas.get(channel.schema_id)
            print(channel.topic, schema.name if schema else None)

    for schema, channel, message, snapshot in reader.iter_decoded_messages(
        topics=["/point_of_interest/snapshot"]
    ):
        print(
            snapshot.id,
            snapshot.camera_pose.position,
            snapshot.camera_pose.orientation,
            snapshot.wall_distance,
            message.log_time,
        )
```

최소 예시는 첫 MCAP 1개만 선택. 스냅샷이 없는 비행에서는 이벤트 출력 0건이 정상.
프로젝트 전체 파싱은 4개 MCAP을 각각 순회.

[MCAP CLI 문서](https://mcap.dev/guides/cli)

---

## 17. 현재 도구와 산출물

### 17.1 데이터 출처별 연결

```text
MCAP
  + Snapshot 이벤트 -> 카메라 pose, drone pose, ID, 시간, 거리
  + Odometry        -> 비행 궤적과 시간별 자세
  + Camera/IMU/TF   -> 센서 샘플과 구조 분석
  + LiDAR bytes    -> 메시지·바이너리 색인

Inspector 프로젝트
  + flights / maps / pois / snapshots JSON -> 파일 연결과 이름
  + JPG / 동영상                           -> 실제 촬영 미디어
  + LAS                                    -> 표시용 3D 점군과 intensity
  + calib_files YAML                       -> VIO·센서 변환 관계

설치된 Inspector 5
  + robotspec.dll -> Xacti 4000 x 3000 K/D/P

위 결과 결합
  -> outputs
  -> LiDAR + 궤적 + POI + 프러스텀 + 보정 사진 표시
```

MCAP 구조 자체를 읽는 데 프로젝트 전체가 필수는 아님.
프로젝트 색인은 사진·지도·POI·비행을 연결한 통합 뷰어를 만들기 위한 절차.

### 17.2 스크립트

기준 폴더: `D:\Conda_envs\ROS\ROSM\rosm_tools\scripts`

| 스크립트 | 역할 | 출력 폴더 |
|---|---|---|
| `01_scan_project.py` | 프로젝트 파일·관계 색인 | `outputs/index` |
| `02_list_mcap.py` | MCAP 파일·채널·schema·chunk·metadata 목록 | `outputs/mcap` |
| `03_sample_mcap_topics.py` | 관심 topic 메시지·필드 샘플 | `outputs/mcap_samples` |
| `04_index_snapshots.py` | MCAP 스냅샷 추출, JSON 비교·연결 | `outputs/snapshots` |
| `05_index_calibrations.py` | 프로젝트 YAML과 Inspector Xacti 보정값 색인 | `outputs/calibration` |
| `06_index_lidar_trajectory.py` | 기존 LAS preview, MCAP 궤적, 장면 목록 | `outputs/lidar` |
| `06_view_lidar_scene.py` | Open3D GUI | 기존 outputs 사용 |
| `07_index_raw_lidar.py` | LiDAR 메시지·payload 구조 색인 | `outputs/raw_lidar` |

GUI의 `MCAP 정보 추출`은 위 색인·추출 단계를 실행하는 기능.
MCAP 내부 LiDAR를 새 LAS로 변환한다는 의미는 아님.

### 17.3 접근 순서별 파일

| 질문 | 확인 파일 |
|---|---|
| 어떤 topic이 있고 몇 건인가 | `mcap/mcap_channels.csv` |
| 실제 ROS 메시지 구조와 값은 무엇인가 | `mcap_samples/mcap_topic_samples.jsonl`, `mcap_topic_fields.csv` |
| 스냅샷 카메라 위치·자세는 무엇인가 | `snapshots/mcap_snapshot_events.csv` |
| 프로젝트 JSON과 같은가 | `snapshots/snapshot_mcap_matches.csv` |
| 비행 전체 궤적·자세는 무엇인가 | `lidar/flight_trajectories.csv` |
| 뷰어의 비행·LAS·preview 연결은 무엇인가 | `lidar/scene_manifest.json` |
| Xacti 보정값은 무엇인가 | `calibration/xacti_4000x3000_camera_spec.json` |
| 보정 사진은 어디에 있는가 | `calibration/undistorted_images/*_undistorted_raw_v3.jpg` |
| LiDAR bytes에서 어디까지 읽었는가 | `raw_lidar/raw_lidar_messages.csv`, `raw_lidar_topic_stats.csv` |

위 상대 경로의 기준은 `D:\Conda_envs\ROS\ROSM\outputs`.
출력 위치를 따로 지정한 경우 해당 폴더가 기준.

---

## 18. 확인 완료와 남은 범위

| 상태 | 항목 |
|---|---|
| 확인 완료 | 4개 MCAP의 ROS2 profile, CDR encoding, Channel·Schema·메시지 통계 |
| 확인 완료 | Snapshot 7건의 카메라 pose와 프로젝트 JSON 비교값 일치 |
| 구현 완료 | MCAP snapshot pose와 odometry 추출·표시 |
| 구현 완료 | 프로젝트 LAS·JPG와 MCAP pose 연결 |
| 구현 완료 | 설치된 Inspector에서 Xacti K/D/P 확보·JSON 저장 |
| 구현 완료 | 왜곡 보정, 보정값 기반 프러스텀, 카메라 시점, 사진 평면 |
| 파싱 실패 | jfly LiDAR payload의 XYZ·intensity 복원, MCAP 단독 LAS 생성 |

---

## 19. 근거 자료

### 로컬 근거

- `outputs/mcap/mcap_inventory_summary.json`, `mcap_files_detail.csv`, `mcap_channels.csv`, `mcap_metadata.csv`
- `outputs/mcap_samples/mcap_sample_summary.json`, `mcap_topic_samples.jsonl`
- `outputs/snapshots/snapshot_pose_summary.json`, `snapshot_mcap_matches.csv`
- `outputs/lidar/lidar_trajectory_summary.json`, `las_files.csv`
- `outputs/raw_lidar/raw_lidar_summary.json`, `raw_lidar_topic_stats.csv`
- `outputs/calibration/calibration_summary.json`, `xacti_4000x3000_camera_spec.json`
- 원본 MCAP에 내장된 `elios_ros_msgs/msg/Snapshot`, `Odometry`, `PacketBytes`, `Mask2d`, `CameraState` schema
- `rosm_tools/rosm`의 추출기·뷰어 코드
- 2026-09-07 사용자 Inspector 5 비교 확인

### 웹 근거

웹 문서는 포맷·API·제품 동작 근거. 로컬 파일의 메시지 수나 pose 일치 결과를 입증하는 자료는 위 로컬 산출물.

| 내용 | 주소 |
|---|---|
| Elios 3 2025.01부터 BAG 대신 MCAP | https://knowledge.flyability.com/third-party-software/faro-connect |
| Inspector5 workspace 공유와 데이터별 export | https://knowledge.flyability.com/software-and-cloud/inspector/inspector-5/common-workflows/inspector-5-how-to-share-flight-data |
| ROS rosbag API | https://docs.ros.org/en/noetic/api/rosbag/html/c++/index.html |
| ROS bag storage record | https://docs.ros.org/en/lunar/api/rosbag_storage/html/c++/namespacerosbag.html |
| MCAP 파일·record 명세 | https://mcap.dev/spec |
| MCAP CLI | https://mcap.dev/guides/cli |
| MCAP Python reader | https://mcap.dev/docs/python/mcap-apidoc/mcap.reader |
| MCAP ROS2 decoder | https://mcap.dev/docs/python/mcap-ros2-apidoc/mcap_ros2.decoder |
| OpenCV fisheye 보정 | https://docs.opencv.org/doc/doxygen/html/d2/d14/namespacecv_1_1fisheye.html |
| OpenCV EXIF 방향 처리 | https://docs.opencv.org/4.13.0/d4/da8/group__imgcodecs.html |
