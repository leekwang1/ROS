# Elios 드론 데이터 파일 구조

Elios 드론 촬영 데이터의 각 폴더와 파일이 무엇인지 빠르게 이해하기 위한 정리

- 작성일: 2026-09-17
- 분석 기준: 참고 3개 및 세풍대교 데이터 셋
- Inspector 프로젝트 파일과 ROSM 도구가 새로 생성한 파일을 구분
- topic·schema·메시지 수 등의 상세 분석은 [BAG / MCAP 상세 문서](bag_mcap_file_structure.md) 참고

## 전체 차이

| 구분 | 촬영일 | 비고 |
|---|---|---|
| `106-inspection-Undefined-01` |23/05/18| Inspector4 계열 구조 |
| `230-inspection-GC-3-HandyHandle` |23/08/21| Inspector4 계열 구조 |
| `Ouster REV7(128채널 LiDAR)_하수관로` |25/11/28| Inspector5 workspace/export 또는 Asset Management 구조|
| `세풍대교 촬영 셋` |26/08/25| Inspector5 workspace/export 또는 Asset Management 구조|

Flyability 공식 문서 기준 Elios 3 2025.01 업데이트 이후 `.bag`를 `.mcap`으로 대체.
[공식 FARO Connect 안내](https://knowledge.flyability.com/third-party-software/faro-connect)

---

## 1. Inspector4 계열 구조

대상 폴더:

```text
106-inspection-Undefined-01/
└─ 106-inspection-Undefined-01/
   ├─ 106-inspection-Undefined-01.efly
   ├─ thumbnail.jpg
   ├─ 106_0116-003.MOV
   ├─ 106_0117.JPG
   ├─ 106_0118.JPG
   ├─ E300..._obcbag-001.bag
   ├─ E300..._livemap.las
   ├─ E300..._livetraj.csv
   ├─ E300..._thermal.mp4
   ├─ E300..._thermal.stn
   ├─ E300..._thermal.meta
   ├─ E300..._starnet.stn
   ├─ E300..._lidar_metadata.json
   ├─ E300..._nxstats.csv
   ├─ E300..._syslog.txt
   ├─ E300..._avdebug.LOG
   ├─ E300..._roslog/
   └─ E300..._calib_files/
```

### 주요 파일 설명

| 파일/폴더 | 쉽게 말하면 | 용도 |
|---|---|---|
| `.efly` | Inspector4 계열 프로젝트/비행 정보 | 어떤 영상, 로그, 지도 파일이 한 세트인지 기록 |
| `thumbnail.jpg` | 대표 이미지 | 미리보기용 |
| `.MOV` | 일반 카메라 원본 영상 | 사람이 보는 RGB 영상 |
| `.JPG` | 일반 카메라 사진 | 촬영 이미지 |
| `.bag` | ROS1 rosbag 원시 기록 파일 | 드론 센서값, 위치, 자세, LiDAR 데이터 등을 시간순으로 저장 |
| `.las` | LiDAR 포인트클라우드 지도 | 3D 점군 지도 |
| `livetraj.csv` | 비행 궤적 | 시간별 위치와 자세 |
| `thermal.mp4` | 열화상 영상 | 온도/열 영상 확인 |
| `thermal.stn` | 열화상 원시 로그 | 열화상 동기화/원시 데이터 |
| `thermal.meta` | 열화상 메타 정보 | 열화상 보정/설정 정보 |
| `starnet.stn` | 비행 로그 | 드론 비행 상태와 센서 로그 |
| `lidar_metadata.json` | LiDAR 설정 정보 | LiDAR 모델, 채널 수, 보정값 |
| `nxstats.csv` | 컴퓨터 상태 로그 | CPU, GPU, 메모리, 온도, 전원 상태 |
| `syslog.txt` | 시스템 로그 | 장비 내부 시스템 로그 |
| `avdebug.LOG` | 드론 디버그 로그 | 항전/제어 관련 디버그 기록 |
| `roslog/` | ROS 실행 로그 | ROS 노드별 실행 기록 |
| `calib_files/` | 보정 파일 모음 | 카메라, IMU, LiDAR 좌표 변환에 사용 |

### 보정 파일 폴더

```text
calib_files/
├─ imu_intrinsics/
├─ system/
└─ vio_module/
```

| 폴더 | 의미 |
|---|---|
| `imu_intrinsics/` | IMU 자체 보정값 |
| `system/` | 드론 몸체, LiDAR, 중심점 등의 좌표 변환값 |
| `vio_module/` | VIO 카메라 보정값과 IMU-카메라 위치/자세 관계. 4K 촬영 카메라 보정과 구분 |

---

## 2. Inspector5 프로젝트 구조

대상 폴더:

```text
Ouster REV7(128채널 LiDAR)_하수관로/
├─ orgs/
├─ assets/
├─ inspections/
├─ flights/
├─ maps/
├─ pois/
└─ snapshots/
```

### 최상위 폴더 설명

| 폴더 | 항목 | 내용 |
|---|---|---|
| `orgs/` | 회사/조직 정보 | 조직 이름, 색상 등 |
| `assets/` | 검사 대상 시설 정보 | 하수관로, 구조물 같은 대상 자산 |
| `inspections/` | 검사 프로젝트 정보 | 검사 이름, 연결된 자산 |
| `flights/` | 비행별 데이터와 관리 정보 | 영상, 센서 기록, 열화상, 보정파일, 비행 JSON |
| `maps/` | 지도 데이터 | LiDAR 점군, 궤적 |
| `pois/` | 관심 지점 | 결함 위치나 점검 포인트 |
| `snapshots/` | 스냅샷 이미지 | POI에 연결된 사진, 카메라 위치/자세 JSON |

---

## 3. `flights/` 폴더

비행 1개마다 UUID 폴더로 구분

```text
flights/
└─ <flight-id>/
   ├─ flight.json
   ├─ flight.json.*.backup
   ├─ *.MOV
   ├─ <flight-id>.jpg
   ├─ E3R..._obcbag.mcap
   ├─ E3R..._thermal.mp4
   ├─ E3R..._thermal.stn
   ├─ E3R..._thermal.meta
   ├─ E3R..._starnet.stn
   ├─ E3R..._lidar_metadata.json
   ├─ E3R..._logbook.json
   ├─ E3R..._nxstats.csv
   ├─ E3R..._gaston-stats.csv
   ├─ E3R..._syslog.txt
   ├─ E3R..._calib_files/
   └─ E3R..._roslog/
```

| 파일/폴더 | 설명 | 용도 |
|---|---|---|
| `flight.json` | 비행 설명 정보 | 비행 시간, 드론 정보, 연결된 파일 목록 |
| `.backup` | JSON 백업 | 보통 분석에서는 제외 가능 |
| `.MOV` | 일반 카메라 원본 영상 | 비행 중 촬영된 RGB 영상 |
| `<flight-id>.jpg` | 비행 썸네일 | 대표 이미지 |
| `.mcap` | 시간 정보가 붙은 메시지 기록 | 원시 센서뿐 아니라 계산된 비행 위치·자세, 스냅샷 이벤트도 포함 |
| `thermal.mp4` | 열화상 영상 | 열 영상 확인 |
| `thermal.stn` | 열화상 원시 로그 | 열화상 관련 원시 데이터 |
| `thermal.meta` | 열화상 메타 정보 | 열화상 보정/설정 |
| `starnet.stn` | 비행 로그 | 드론 상태, 위치, 센서 로그 |
| `lidar_metadata.json` | LiDAR 설정 정보 | 모델·채널 수·센서 설정 등. 채널 수는 파일 내용 기준 확인 |
| `logbook.json` | 비행 요약 로그 | 알림, 통계, 로봇 상태 |
| `nxstats.csv` | 컴퓨터 상태 로그 | CPU, GPU, 메모리, 온도 |
| `gaston-stats.csv` | ROS2 노드 상태 | 노드별 CPU 사용량 |
| `syslog.txt` | 시스템 로그 | 장비 내부 로그 |
| `calib_files/` | 센서 보정 파일 | VIO 카메라·IMU·LiDAR 보정과 좌표 변환. Xacti 4K 보정과 구분 |
| `roslog/` | ROS 로그 | ROS 노드 실행 로그 |

### 이번 MCAP에서 확인한 내용

- 기록 형식: ROS2 profile, `ros2msg` schema, `cdr` 메시지
- 비행 궤적: `/kalman_scan2map_node/odometry`
- 스냅샷 이벤트: `/point_of_interest/snapshot`
- LiDAR 바이너리: `/os_node/lidar_scan/compressed`
- VIO 이미지: `/camera_0/image_raw/compressed` ~ `/camera_2/image_raw/compressed`
- IMU·TF·장비 상태 등의 메시지도 포함

스냅샷 이벤트에는 카메라 위치·자세가 있지만 4000 x 3000 JPG 자체는 없음.

### 보정 정보의 구분

아래 해상도와 보정 출처는 이번 `sepoong` 분석 기준.

| 구분 | 위치 | 역할 |
|---|---|---|
| VIO 카메라 보정 | `calib_files/.../camera_calibration_0~2.yaml` | 640 x 400 카메라용 내부 보정 |
| 센서 간 변환 | `imu_cam_extrinsics.yaml`, `static_transforms.yaml`, `geoslam_transforms.yaml` | IMU·카메라·LiDAR·기체 좌표계 관계 |
| Xacti 촬영 카메라 보정 | 설치된 Inspector5의 `robotspec.dll` | 4000 x 3000 사진용 K/D/P 확보 |
| Xacti 보정 JSON | 도구가 생성한 `outputs/calibration/xacti_4000x3000_camera_spec.json` | 왜곡 보정과 프러스텀에 사용 |

4K 카메라 Xacti의 보정정보 K/D/P는 MCAP에서 읽은 값이 아니라 설치된 Inspector의 CameraSpec에서 확보한 값.

---

## 4. `maps/` 폴더

지도 1개마다 UUID 폴더가 하나씩 있음

```text
maps/
└─ <map-id>/
   ├─ map.json
   ├─ map.json.*.backup
   ├─ *.las
   ├─ *_livetraj.csv
   └─ thumbnail.jpg
```

| 파일 | 설명 | 용도 |
|---|---|---|
| `map.json` | 지도 설명 정보 | 어느 비행/검사에 연결되는지 기록 |
| `.las` | 포인트클라우드 | LiDAR로 만든 3D 점군 데이터 |
| `livetraj.csv` | 비행 궤적 | 지도 생성에 사용된 위치/자세 기록 |
| `thumbnail.jpg` | 지도 썸네일 | 미리보기 이미지 |
| `.backup` | 백업 파일 | - |

---

## 5. `pois/` 폴더

POI(Point Of Interest)
점검할 위치, 결함 위치, 관심 지점 관련 폴더

```text
pois/
└─ <poi-id>/
   ├─ poi.json
   └─ poi.json.*.backup
```

| 파일 | 쉽게 말하면 | 용도 |
|---|---|---|
| `poi.json` | POI 설명서 | POI 이름, ID, 연결된 자산 정보 |
| `.backup` | 백업 파일 | 보통 분석에서는 제외 가능 |

예시 POI 이름:

```text
POI-1009-1
POI-1010-5
POI-1012-2
```

---

## 6. `snapshots/` 폴더

스냅샷 1개마다 UUID 폴더가 하나씩 있음

```text
snapshots/
└─ <snapshot-id>/
   ├─ snapshot.json
   ├─ snapshot.json.*.backup
   ├─ 010_0557.JPG
   ├─ <snapshot-id>_rgb.jpg
   ├─ <snapshot-id>_thm.jpg
   ├─ result.jpg
   ├─ result_annotated.jpg
   └─ thumbnail.jpg
```

| 파일 | 쉽게 말하면 | 용도 |
|---|---|---|
| `snapshot.json` | 스냅샷 설명 정보 | 촬영 시간, 카메라 위치/자세, 드론 위치/자세, 연결된 POI |
| `010_0557.JPG` | 원본 사진 | 실제 촬영 이미지 |
| `<snapshot-id>_rgb.jpg` | RGB 이미지 | 스냅샷용 일반 이미지 |
| `<snapshot-id>_thm.jpg` | 열화상 이미지 | 스냅샷용 열화상 이미지 |
| `result.jpg` | 처리 결과 이미지 | Inspector5에서 만든 결과 이미지 |
| `result_annotated.jpg` | 주석 포함 이미지 | 표시/마킹이 들어간 결과 이미지 |
| `thumbnail.jpg` | 썸네일 | 미리보기 이미지 |
| `.backup` | 백업 파일 | - |

### `snapshot.json`에 들어있는 중요한 값

| 항목 | 의미 |
|---|---|
| `flight_id` | 비행 ID |
| `poi_id` | POI ID |
| `time` | 프로젝트가 저장한 촬영 시점 |
| `camera_pose` | 카메라 위치와 자세 |
| `drone_pose` | 드론 위치와 자세 |
| `wall_distance` | 대상물 까지의 거리 값 |
| `camera_image` | 연결된 사진 파일 경로 또는 이름 |
| `camera_model` | 카메라 모델. 스냅샷에서 Xacti 모델 확인 |
| `image_size` | 이미지 크기. 원본 사진은 4000 x 3000 |
| `undistorted` | 왜곡 보정 여부 표시. 원본 snapshot JSON에서 false 확인 |

### 스냅샷 정보의 실제 출처

| 표시 정보 | 현재 도구의 기준 |
|---|---|
| 카메라 위치·자세 | MCAP Snapshot의 `camera_pose` |
| 같은 이벤트의 드론 위치·자세 | MCAP Snapshot의 `drone_pose` |
| 거리 값 | MCAP Snapshot의 `wall_distance` |
| JPG 경로·이미지 이름 | 프로젝트 snapshot JSON |
| POI 이름·관계 | 프로젝트 POI·snapshot JSON |
| 프러스텀 모양·보정 이미지 투영 | Inspector에서 확보한 Xacti `P` |
| 사진 왜곡 보정 | 원본 JPG와 Xacti `K/D/P` |

스냅샷 7건은 ID로 연결했을 때 MCAP과 프로젝트 JSON의 카메라 위치·자세 차이 없음.
이 결과에 따라 현재 뷰어는 MCAP pose를 사용하고 프로젝트 JSON은 사진·이름 연결에 활용.

- MCAP의 `camera_pose`에 기체-카메라 변환이나 pitch를 다시 적용하지 않음
- 보정 사진은 입력 프로젝트의 `snapshots/`가 아닌 도구 출력 폴더에 별도 저장

---

## 7. `assets/`, `inspections/`, `orgs/`

### `assets/`

```text
assets/
└─ <asset-id>/
   ├─ asset.json
   ├─ asset.json.*.backup
   └─ *_chunk.las
```

| 파일 | 의미 |
|---|---|
| `asset.json` | 점검 대상 정보 |
| `*_chunk.las` | 자산 기준의 작은 포인트클라우드 조각 |
| `.backup` | 백업 파일 |

### `inspections/`

```text
inspections/
└─ <inspection-id>/
   ├─ inspection.json
   └─ inspection.json.*.backup
```

| 파일 | 의미 |
|---|---|
| `inspection.json` | 점검 프로젝트 정보 |
| `.backup` | 백업 파일 |

### `orgs/`

```text
orgs/
└─ <org-id>/
   ├─ organization.json
   └─ organization.json.*.backup
```

| 파일 | 의미 |
|---|---|
| `organization.json` | 회사/조직 정보 |
| `.backup` | 백업 파일 |

---

## 8. 프로젝트 입력과 도구 출력 구조

현재 작업 경로 기준.

```text
D:\Conda_envs\ROS\ROSM\
├─ sepoong/                     # 입력 Inspector5 프로젝트
│  ├─ orgs/
│  ├─ assets/
│  ├─ inspections/
│  ├─ flights/
│  ├─ maps/
│  ├─ pois/
│  └─ snapshots/
├─ rosm_tools/                  # 직접 작성한 Python 코드·문서
│  ├─ scripts/
│  └─ rosm/
└─ outputs/                     # 도구가 생성한 분석·표시용 파일
   ├─ index/
   ├─ mcap/
   ├─ mcap_samples/
   ├─ snapshots/
   ├─ calibration/
   │  ├─ xacti_4000x3000_camera_spec.json
   │  └─ undistorted_images/
   ├─ lidar/
   │  ├─ scene_manifest.json
   │  └─ preview_ply/
   └─ raw_lidar/
```

`outputs/`는 Inspector5 원래 프로젝트 규격에 포함된 폴더가 아니라 파이썬으로 생성한 ROSM 도구의 출력 폴더.
사용자가 출력 위치를 지정한 경우 위 예시와 다른 경로 사용.

| 출력 폴더 | 주요 내용 |
|---|---|
| `index/` | 프로젝트 파일·비행·지도·POI·사진의 연결 목록 |
| `mcap/` | MCAP 파일, Channel, Schema, Chunk, Metadata 목록과 통계 |
| `mcap_samples/` | 관심 topic의 메시지·필드·바이너리 샘플 |
| `snapshots/` | MCAP snapshot 이벤트, 프로젝트 JSON 비교, 사진 연결 목록 |
| `calibration/` | 센서 YAML 색인, 좌표 변환 표, Xacti 보정 JSON |
| `calibration/undistorted_images/` | 필요할 때 생성하는 보정 JPG 캐시 |
| `lidar/` | LAS 색인, MCAP 궤적 CSV, 비행 장면 목록 |
| `lidar/preview_ply/` | 기존 LAS에서 샘플링한 점군 |
| `raw_lidar/` | LiDAR bytes의 시간·길이·식별 바이트 등 분석. 복원된 LAS는 없음, MCAP 에서 라이다 복원은 실패 |

---

## 9. 입력에서 표시까지 연결

| 입력 파일 | 읽는 정보 | 생성 파일 또는 활용 |
|---|---|---|
| 프로젝트 JSON | ID, 이름, 파일 경로, 비행·지도·사진 관계 | `index/*.csv` |
| MCAP Header·Summary | topic, schema, 메시지 수, 기록 범위 | `mcap/*.csv`, 요약 JSON |
| MCAP Snapshot | 카메라 pose, 드론 pose, 시간, ID, 거리 | `snapshots/mcap_snapshot_events.csv` |
| snapshot·POI JSON | 사진 경로, POI 이름, 비교용 pose | `snapshots/snapshot_json_poses.csv`, 매칭 CSV |
| MCAP Odometry | 비행 중 위치·자세·속도 등 | `lidar/flight_trajectories.csv` |
| 프로젝트 LAS | XYZ·intensity | `lidar/preview_ply/*.ply`, 점군 표시 |
| 프로젝트 calib YAML | VIO 내부 보정, 센서 간 변환 | `calibration/*.csv` |
| 설치된 Inspector DLL | Xacti K/D/P | `calibration/xacti_4000x3000_camera_spec.json` |
| 원본 JPG + Xacti K/D/P | 왜곡 보정 | `calibration/undistorted_images/*_undistorted_raw_v3.jpg` |
| MCAP LiDAR bytes | payload 길이·시간·앞부분 분석 | `raw_lidar/*.csv`, 샘플 JSONL |

MCAP 구조만 확인하는데 프로젝트 전체가 필수는 아님.
사진·LAS·POI를 연결해 함께 보는 통합 뷰어에는 해당 프로젝트 파일 필요.

---

## 10. 보정 사진과 프러스텀 확인 사항

- 원본 JPG는 변경하지 않고 보정 JPG를 출력 폴더에 별도 저장
- 보정 입력은 EXIF 자동 회전 제외, 저장 JPG에는 원본 표시 방향 정보 보존
- EXIF 자동 회전시, 사진이 거꾸로 보임
- 캘리브레이션이 로드되면 처음부터 Xacti P 기반 프러스텀 생성
- 보정 체크박스는 사진을 원본·보정본으로 전환하는 기능
- 카메라 시점은 보정 프러스텀의 비대칭 투영과 창 비율 반영
- 사용자 시각 비교에서 보정 사진과 가장자리 잘림이 Inspector5 결과와 동일하다고 확인

---

## 11. 상세 문서와 근거

### 상세 분석

[BAG / MCAP 파일 구조 및 파싱 정리](bag_mcap_file_structure.md)

- 포맷 차이와 내부 record 구조
- 실제 MCAP 4개의 topic·schema·메시지 수
- Snapshot 7건의 pose 비교 결과
- LiDAR jfly 분석 한계
- Xacti 보정 수치·파싱 코드·스크립트별 산출물
- 완료한 기능과 추가 검증 범위

### 로컬 근거

경로 기준: `D:\Conda_envs\ROS\ROSM`

- `outputs/index/project_summary.json`: 프로젝트 파일 목록과 집계
- `outputs/mcap/mcap_inventory_summary.json`: MCAP 4개 통계
- `outputs/snapshots/snapshot_pose_summary.json`: 스냅샷 7건의 pose 일치 결과
- `outputs/lidar/lidar_trajectory_summary.json`: 기존 LAS와 MCAP 궤적 처리 결과
- `outputs/calibration/xacti_4000x3000_camera_spec.json`: 확보한 Xacti 값과 출처
- `rosm_tools/rosm/lidar_viewer.py`: 현재 데이터 출처와 표시·보정 구현

### 웹 근거

| 내용 | 주소 |
|---|---|
| Elios 3 BAG/MCAP 기록 형식 변경 | https://knowledge.flyability.com/third-party-software/faro-connect |
| Inspector5 workspace 공유와 데이터별 export | https://knowledge.flyability.com/software-and-cloud/inspector/inspector-5/common-workflows/inspector-5-how-to-share-flight-data |
| MCAP 파일 규격 | https://mcap.dev/spec |
| OpenCV fisheye 보정 | https://docs.opencv.org/doc/doxygen/html/d2/d14/namespacecv_1_1fisheye.html |
| OpenCV EXIF 방향 처리 | https://docs.opencv.org/4.13.0/d4/da8/group__imgcodecs.html |

웹 문서는 제품·포맷·API 동작 근거이며, 로컬 수치와 pose 일치의 근거는 위 분석 산출물.
