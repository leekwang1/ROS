# ROSM Tools

Inspector5 프로젝트의 MCAP·LAS·사진·보정 정보를 읽고, LiDAR 점군·비행 궤적·스냅샷 카메라를 함께 표시하는 Python Test 도구.

- 문서 갱신: 2026-09-17
- 실행 기준 코드: `D:\Conda_envs\ROS\ROSM\rosm_tools`
- 확인한 입력: `D:\Conda_envs\ROS\ROSM\sepoong`
- 기본 분석 출력: `D:\Conda_envs\ROS\ROSM\outputs`
- 기존 프로젝트 파일은 읽기 입력으로 사용. CSV·JSON·PLY·보정 JPG는 별도 출력

## 목차

1. [기능과 범위](#1-기능과-범위)
2. [폴더 구성](#2-폴더-구성)
3. [설치와 환경 확인](#3-설치와-환경-확인)
4. [사용 라이브러리](#4-사용-라이브러리)
5. [GUI로 시작](#5-gui로-시작)
6. [단계별 명령 실행](#6-단계별-명령-실행)
7. [입력 데이터와 연결](#7-입력-데이터와-연결)
8. [추출 단계 상세](#8-추출-단계-상세)
9. [점군과 색상 표시](#9-점군과-색상-표시)
10. [궤적과 POI 표시](#10-궤적과-poi-표시)
11. [캘리브레이션과 사진 투영](#11-캘리브레이션과-사진-투영)
12. [GUI 기능](#12-gui-기능)
13. [실행 옵션](#13-실행-옵션)
14. [출력 파일과 재실행](#14-출력-파일과-재실행)
15. [오류 확인](#15-오류-확인)
16. [검증 결과와 한계](#16-검증-결과와-한계)
17. [코드 구성과 참고 문서](#17-코드-구성과-참고-문서)

## 1. 기능과 범위

| 기능 | 현재 구현 |
|---|---|
| Inspector5 프로젝트 색인 | 비행·지도·POI·사진·미디어·보정 파일과 ID 연결 |
| MCAP 목록화 | Header, Channel, Schema, Chunk, Metadata, 메시지 수 |
| ROS2 메시지 decode | 관심 topic 샘플, snapshot 이벤트, odometry 등 |
| 스냅샷 카메라 pose | MCAP에서 추출, 프로젝트 JSON과 비교 |
| 점군 표시 | 기존 LAS에서 만든 preview PLY 표시 |
| 비행 경로 | MCAP odometry의 시간순 위치를 선으로 연결 |
| POI 선택 | 위치 마커의 hover·클릭, 이미지 열기 |
| 카메라 표시 | RGB 자세 축, 프러스텀, 사진 평면, 선택 카메라 시점 |
| 사진 보정 | Inspector에서 확보한 Xacti K/D/P로 fisheye 왜곡 보정 |
| LiDAR raw 분석 | PacketBytes·Mask2d의 시간·길이·바이너리 앞부분 색인 |

**현재 3D 점군은 MCAP에서 복원한 결과가 아니라 프로젝트 LAS에서 읽은 결과.**
MCAP LiDAR의 `jfly` payload를 XYZ·intensity로 해독하거나 MCAP만으로 LAS 파싱 실패함.

ROS1 BAG는 파일 종류로 식별 가능하지만, 현재 단계별 메시지 추출 파이프라인은 분석한 ROS2 MCAP 기준.
Inspector의 모든 기능을 재구현한 소프트웨어는 아님.

## 2. 폴더 구성

```text
D:\Conda_envs\ROS\
├─ python.exe
└─ ROSM/
   ├─ sepoong/                          # 입력 Inspector5 프로젝트
   │  ├─ orgs/
   │  ├─ assets/
   │  ├─ inspections/
   │  ├─ flights/
   │  ├─ maps/
   │  ├─ pois/
   │  └─ snapshots/
   ├─ rosm_tools/                       # 프로그램
   │  ├─ README.md
   │  ├─ scripts/                      # 실행 진입점 8개
   │  ├─ rosm/                         # 실제 구현 모듈
   │  ├─ tests/test_lidar_camera.py
   │  ├─ bag_mcap_file_structure.md
   │  └─ elios_drone_data_structure.md
   └─ outputs/                         # 도구 생성 결과
      ├─ index/
      ├─ mcap/
      ├─ mcap_samples/
      ├─ snapshots/
      ├─ calibration/
      │  └─ undistorted_images/
      ├─ lidar/
      │  └─ preview_ply/
      └─ raw_lidar/
```

기본 프로젝트는 코드 위치를 기준으로 `../sepoong`, 기본 출력은 프로젝트 상위 폴더의 `outputs`.
다른 PC에서는 코드·프로젝트 경로를 실제 위치에 맞게 지정.

## 3. 설치와 환경 확인

### 3.1 현재 확인한 환경

| 항목 | 확인 값 |
|---|---|
| 운영체제 | Windows 64-bit |
| Python | Conda Python 3.11.15 |
| 실행 파일 | `D:\Conda_envs\ROS\python.exe` |
| GUI | Open3D 0.19.0의 데스크톱 GUI |
| 한글 폰트 | `C:\Windows\Fonts\malgun.ttf` 우선 |
| Xacti 값 신규 추출 | 로컬 Inspector5 설치 필요 |
| 기존 Xacti JSON 활용 | 해당 JSON과 OpenCV가 있으면 DLL 재호출 없이 보정 가능 |

일반 점군·궤적 표시에 ROS 노드, ROS2 실행 환경, 웹 서버는 불필요.
Python 라이브러리로 파일을 직접 읽는 방식.
GUI는 그래픽 데스크톱과 Open3D 렌더링을 지원하는 드라이버 환경 필요.

### 3.2 기존 ROS 환경 사용

PowerShell 실행 예시. `&`는 따옴표로 감싼 실행 파일 경로를 호출하는 연산자.

```powershell
$PythonExe = 'D:\Conda_envs\ROS\python.exe'
& $PythonExe --version
& $PythonExe -m pip check
& $PythonExe -B -c "import mcap, mcap_ros2, numpy, open3d, laspy, yaml, cv2; from PIL import Image; print('imports OK')"
```

현재 환경이 이미 준비되어 있으면 새 환경 생성이나 패키지 재설치 없이 실행 가능.
`-B`는 새 `__pycache__` 생성을 억제하는 옵션. 분석 결과 출력과는 무관.

### 3.3 새 환경 설치 예시

Conda가 사용 가능한 터미널에서 실행.
기존 `ROS` 환경과 구분한 새 경로 예시이며, 코드와 데이터 폴더를 이 환경 안으로 옮길 필요는 없음.

```powershell
conda create --prefix D:\Conda_envs\ROSM_viewer python=3.11 pip -y
$PythonExe = 'D:\Conda_envs\ROSM_viewer\python.exe'
$Packages = @(
    'mcap==1.4.0'
    'mcap-ros2-support==0.5.7'
    'numpy==2.4.4'
    'open3d==0.19.0'
    'laspy==2.7.0'
    'PyYAML==6.0.3'
    'Pillow==12.2.0'
    'opencv-python==5.0.0.93'
    'lz4==4.4.5'
    'zstandard==0.25.0'
)
& $PythonExe -m pip install @Packages
& $PythonExe -m pip check
& $PythonExe -B -c "import mcap, mcap_ros2, numpy, open3d, laspy, yaml, cv2; from PIL import Image; print('imports OK')"
& $PythonExe -B 'D:\Conda_envs\ROS\ROSM\rosm_tools\scripts\06_view_lidar_scene.py' --help
```

버전은 문서 작성 시 로컬에서 확인한 설치 값. 새 환경 전체 설치를 별도로 재현한 검증 결과는 아님.
간접 의존 패키지는 pip가 함께 설치. 완전한 환경 목록 확인은 `python -m pip freeze` 사용.

환경 생성과 Open3D 설치의 공식 안내:
[Conda 환경 관리](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html),
[Open3D 설치](https://www.open3d.org/docs/release/getting_started.html).

### 3.4 Inspector5와 보정값

신규 Xacti 값 추출 시 기대하는 기본 DLL:

```text
C:\Program Files\Flyability\Inspector 5\robotspec.dll
```

- `05_index_calibrations.py`에서 `ctypes`로 CameraSpec 호출
- DLL 호출은 Windows 전용, 설치 버전의 export·메모리 구조에 의존
- 현재 Inspector 설치 경로를 바꾸는 CLI 옵션은 없음. 코드의 `DEFAULT_INSPECTOR_DIR` 기준
- 이미 확보한 `xacti_4000x3000_camera_spec.json`은 출력의 `calibration` 폴더에 두고 활용 가능
- DLL 확보 실패는 `calibration_errors.csv` 확인
- 보정 JSON이 없으면 FOV 기반 근사 프러스텀 사용, 이미지 보정 체크박스 비활성

## 4. 사용 라이브러리

2026-09-07 로컬 환경 확인 기준. 아래는 코드에서 직접 사용하는 패키지와 MCAP 압축 지원 패키지.

| 설치 패키지 | import 이름 | 버전 | 사용 목적 |
|---|---|---|---|
| `mcap` | `mcap` | 1.4.0 | 파일 구조·Summary·메시지 순회 |
| `mcap-ros2-support` | `mcap_ros2` | 0.5.7 | 내장 ros2msg schema를 이용한 CDR decode |
| `numpy` | `numpy` | 2.4.4 | quaternion·행렬·점군·이미지 배열 계산 |
| `open3d` | `open3d` | 0.19.0 | GUI, PointCloud, LineSet, TriangleMesh, 카메라 투영 |
| `laspy` | `laspy` | 2.7.0 | LAS header와 XYZ·intensity 읽기 |
| `PyYAML` | `yaml` | 6.0.3 | 센서 보정 YAML 읽기 |
| `opencv-python` | `cv2` | 5.0.0.93 | JPEG decode와 fisheye remap. 런타임 버전은 5.0.0 |
| `Pillow` | `PIL` | 12.2.0 | EXIF 방향 읽기와 보정 JPG 저장 |
| `lz4` | `lz4` | 4.4.5 | MCAP 라이브러리의 LZ4 압축 지원 |
| `zstandard` | `zstandard` | 0.25.0 | MCAP 라이브러리의 Zstandard 압축 지원 |

- 로컬 MCAP 4개의 Chunk는 무압축. LZ4/Zstandard 사용 가능성과 이번 파일의 실제 압축 상태는 구분
- `mcap-ros1-support` 0.7.4는 환경에 설치되어 있지만 현재 도구에서 사용하지 않음
- `rosbags`는 현재 설치·사용 확인 없음
- 표준 라이브러리 `pathlib/json/csv/argparse`: 파일 경로·입출력·CLI
- `ctypes`: Inspector DLL, `threading/subprocess`: GUI 추출 작업과 단계별 실행
- `hashlib`: 보정 파일 비교, `bisect/math`: 최근접 시간 검색·자세 계산
- Open3D의 간접 의존으로 설치되는 라이브러리는 직접 사용 라이브러리와 구분
- MCAP 파싱·JPEG decode·왜곡 보정·LAS 파싱·렌더링은 위 라이브러리 활용. 프로젝트 연결·CSV 설계·마커·색상·GUI 흐름은 직접 구현

MCAP Python API: [공식 문서](https://mcap.dev/docs/python/).

## 5. GUI로 시작

### 5.1 기본 실행

```powershell
& 'D:\Conda_envs\ROS\python.exe' -B 'D:\Conda_envs\ROS\ROSM\rosm_tools\scripts\06_view_lidar_scene.py'
```

명령 프롬프트에서는 `&` 없이 실행:

```bat
D:\Conda_envs\ROS\python.exe -B D:\Conda_envs\ROS\ROSM\rosm_tools\scripts\06_view_lidar_scene.py
```

기존 출력이 없으면 빈 장면으로 시작 가능.
`scene_manifest.json`이 없다는 이유만으로 시작을 중단하지 않는 구조.

### 5.2 첫 추출

1. 오른쪽 상단 `MCAP 정보 추출` 선택
2. `프로젝트 폴더 선택`으로 `sepoong` 지정, 또는 프로젝트 안의 `.mcap` 파일 선택
3. `출력 폴더 지정` 또는 `자동 출력` 선택
4. `추출 실행` 선택
5. 완료 후 출력 장면 목록 자동 로드
6. `비행 선택`, `스냅샷 선택`으로 확인

자동 출력은 인식한 프로젝트 폴더의 상위에 `outputs` 생성.
예: `ROSM/sepoong` 선택 시 `ROSM/outputs`.
입력을 다시 선택하면 사용자 지정 출력 경로가 초기화되므로 출력 경로는 입력 선택 후 확인.

### 5.3 프로젝트 전체와 단일 MCAP 선택

| 선택 방식 | 처리 범위 |
|---|---|
| 프로젝트 폴더 | 프로젝트 색인 후 해당 MCAP 전체 분석 |
| 프로젝트 내부 MCAP | 상위 프로젝트를 추정하고 색인, MCAP 목록을 선택 파일로 제한 |
| 프로젝트와 분리된 MCAP | 파일 목록·메시지 분석 가능 범위는 있으나 사진·LAS·비행 관계가 없어 통합 장면 보장 불가 |

단일 MCAP을 선택해도 프로젝트의 JSON·LAS 색인을 모두 그 비행 하나로 제한하는 것은 아님.
따라서 장면 목록에 다른 비행의 LAS가 남고, 해당 비행의 MCAP 궤적·스냅샷은 비어 있을 수 있음.

추출은 백그라운드 스레드에서 단계별 자식 프로세스를 순차 실행.
진행 상태는 GUI 주 스레드로 전달. Windows 자식 콘솔 창은 숨김.
현재 취소 버튼은 없으며 `닫기`는 대화상자를 닫는 기능. 추출 중 앱 종료는 작업 완료를 보장하지 않음.

### 5.4 다른 프로젝트·출력 사용

PowerShell 예시:

```powershell
$PythonExe = 'D:\Conda_envs\ROS\python.exe'
$ToolRoot = 'D:\Conda_envs\ROS\ROSM\rosm_tools'
$ProjectRoot = 'D:\InspectionData\ProjectA'
$OutputRoot = 'D:\InspectionData\ProjectA_outputs'
& $PythonExe -B "$ToolRoot\scripts\06_view_lidar_scene.py" $ProjectRoot --lidar-dir "$OutputRoot\lidar" --snapshot-dir "$OutputRoot\snapshots" --calibration-dir "$OutputRoot\calibration"
```

다른 프로젝트는 원본 프로젝트 경로와 출력 경로를 함께 지정해 재실행하는 방식 권장.
현재 GUI의 `출력 폴더 변경`은 시작 시의 원본 `project_root`까지 자동 변경하지 않음.
상대 JPG 경로 해석에 원본 경로가 필요하므로 outputs만 바꿀 경우 사진 연결 오류 가능.

## 6. 단계별 명령 실행

### 6.1 기본 경로의 전체 처리

한 단계씩 실행하거나 GUI의 추출 버튼 사용.

```powershell
$PythonExe = 'D:\Conda_envs\ROS\python.exe'
$Scripts = 'D:\Conda_envs\ROS\ROSM\rosm_tools\scripts'
& $PythonExe -B "$Scripts\01_scan_project.py"
& $PythonExe -B "$Scripts\02_list_mcap.py"
& $PythonExe -B "$Scripts\03_sample_mcap_topics.py"
& $PythonExe -B "$Scripts\04_index_snapshots.py"
& $PythonExe -B "$Scripts\05_index_calibrations.py"
& $PythonExe -B "$Scripts\06_index_lidar_trajectory.py"
& $PythonExe -B "$Scripts\07_index_raw_lidar.py"
& $PythonExe -B "$Scripts\06_view_lidar_scene.py"
```

앞 단계의 오류와 출력 생성 여부를 확인한 뒤 다음 단계 실행.
위 명령 나열 자체는 실패 시 자동 중단하는 배치 스크립트가 아님.

### 6.2 사용자 지정 경로의 전체 처리

```powershell
$PythonExe = 'D:\Conda_envs\ROS\python.exe'
$Scripts = 'D:\Conda_envs\ROS\ROSM\rosm_tools\scripts'
$ProjectRoot = 'D:\InspectionData\ProjectA'
$OutputRoot = 'D:\InspectionData\ProjectA_outputs'
& $PythonExe -B "$Scripts\01_scan_project.py" $ProjectRoot --output-dir "$OutputRoot\index"
& $PythonExe -B "$Scripts\02_list_mcap.py" $ProjectRoot --index-dir "$OutputRoot\index" --output-dir "$OutputRoot\mcap"
& $PythonExe -B "$Scripts\03_sample_mcap_topics.py" $ProjectRoot --mcap-dir "$OutputRoot\mcap" --output-dir "$OutputRoot\mcap_samples"
& $PythonExe -B "$Scripts\04_index_snapshots.py" $ProjectRoot --index-dir "$OutputRoot\index" --mcap-dir "$OutputRoot\mcap" --output-dir "$OutputRoot\snapshots"
& $PythonExe -B "$Scripts\05_index_calibrations.py" $ProjectRoot --index-dir "$OutputRoot\index" --output-dir "$OutputRoot\calibration"
& $PythonExe -B "$Scripts\06_index_lidar_trajectory.py" $ProjectRoot --index-dir "$OutputRoot\index" --mcap-dir "$OutputRoot\mcap" --snapshot-dir "$OutputRoot\snapshots" --output-dir "$OutputRoot\lidar"
& $PythonExe -B "$Scripts\07_index_raw_lidar.py" $ProjectRoot --mcap-dir "$OutputRoot\mcap" --output-dir "$OutputRoot\raw_lidar"
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" $ProjectRoot --lidar-dir "$OutputRoot\lidar" --snapshot-dir "$OutputRoot\snapshots" --calibration-dir "$OutputRoot\calibration"
```

### 6.3 옵션 확인과 대용량 조정

```powershell
& $PythonExe -B "$Scripts\03_sample_mcap_topics.py" --help
& $PythonExe -B "$Scripts\06_index_lidar_trajectory.py" --help
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" --help
& $PythonExe -B "$Scripts\07_index_raw_lidar.py" --help
```

| 단계 | 옵션 | 기본값·의미 |
|---|---|---|
| topic 샘플 | `--samples-per-topic` | 1, topic별 샘플 수 |
| topic 샘플 | `--roles` | lidar, pose_or_odometry, tf_transform, snapshot_or_poi, imu, camera |
| 장면 생성 | `--max-preview-points` | 200000, LAS별 preview 최대 점 수 |
| 장면 생성 | `--odometry-topics` | `/kalman_scan2map_node/odometry`, 쉼표로 복수 지정 가능 |
| Raw LiDAR | `--max-messages-per-topic` | 0이면 전량, 양수이면 topic별 상한 |
| Raw LiDAR | `--topics` | 기본 PacketBytes·Mask2d topic, 쉼표로 구분 |

고주기 궤적을 사용하려면 장면 생성에서 해당 topic을 추출한 뒤 뷰어의 `--topic`도 같은 값으로 지정.
GUI의 추출 버튼은 기본 추출 옵션 사용. preview 점 수 등은 개별 CLI로 조정.

## 7. 입력 데이터와 연결

| 입력 | 읽는 정보 | 표시·활용 |
|---|---|---|
| `flight.json` 등 프로젝트 JSON | UUID, 비행 식별자, 연결 경로 | 비행 목록·파일 연결 |
| MCAP `/point_of_interest/snapshot` | camera_pose, drone_pose, stamp, ID, wall_distance | POI·카메라 자세·프러스텀 위치 |
| MCAP `/kalman_scan2map_node/odometry` | 시간별 pose, twist, confidence | 비행 궤적 CSV·경로선 |
| MCAP camera_0~2·IMU·TF | 센서 메시지와 필드 | 샘플 분석, 현재 4K 사진으로 사용하지 않음 |
| MCAP LiDAR PacketBytes·Mask2d | 바이너리 길이·시간·앞부분·마스크 구조 | raw 분석 목록 |
| `snapshot.json`, `poi.json` | JPG 경로·이름·POI 관계, 비교용 pose | MCAP snapshot ID에 사진 연결 |
| `maps/.../*.las` | XYZ, intensity, header | 3D 점군과 색상 |
| 프로젝트 `*_livetraj.csv` | Inspector 궤적 기록 | 파일 요약, 현재 화면 경로선의 직접 출처는 아님 |
| `calib_files` YAML | VIO 내부 보정·센서 변환 | 보정·좌표계 색인 |
| Inspector `robotspec.dll` | Xacti 4000 x 3000 K/D/P | 4K 이미지 보정과 프러스텀 |
| 원본 JPG | 검사 사진 픽셀·EXIF | 외부 이미지 열기·사진 평면 |
| MOV·열화상·시스템 로그 | 파일 경로와 분류 | 목록 관리. 현재 영상 재생·열화상 분석 GUI는 미구현 |

```text
프로젝트 JSON ── ID·이름·파일 경로 ─┐
MCAP ─────────── pose·궤적 ──────┤
LAS ──────────── XYZ·intensity ──┤
JPG ──────────── 사진 ───────────┼─> outputs ─> Open3D 통합 표시
보정 YAML ────── 센서 관계 ──────┤
Inspector DLL ── Xacti K/D/P ────┘
```

MCAP 파일 구조를 읽는 것과 프로젝트 전체를 연결하는 것은 별도 작업.
통합 화면에 필요한 사진과 지도는 MCAP 이벤트에서 자동 복원되지 않음.

## 8. 추출 단계 상세

| 실행 파일 | 구현 모듈 | 처리 내용 |
|---|---|---|
| `01_scan_project.py` | `scan_project.py` | 파일 재귀 조사, JSON 엔터티와 ID 색인 |
| `02_list_mcap.py` | `mcap_inventory.py` | Header·Summary, channel/schema/chunk/metadata 통계 |
| `03_sample_mcap_topics.py` | `mcap_samples.py` | ROS2 decoder로 관심 메시지 샘플과 중첩 필드 정리 |
| `04_index_snapshots.py` | `snapshot_pose_index.py` | Snapshot 전량 추출, ID 매칭, pose·시간 차이 비교 |
| `05_index_calibrations.py` | `calibration_index.py` | YAML·변환 그래프·동일 파일 hash 비교, Xacti JSON 확보 |
| `06_index_lidar_trajectory.py` | `lidar_trajectory.py` | LAS preview, odometry 전량, 최근접 snapshot 시간 비교, manifest |
| `07_index_raw_lidar.py` | `raw_lidar_index.py` | LiDAR 메시지별 bytes 요약과 통계 |
| `06_view_lidar_scene.py` | `lidar_viewer.py` | 산출물을 읽는 Open3D GUI |

### MCAP decode 구분

```text
MCAP 컨테이너 읽기
  -> Channel의 schema_id와 message_encoding 확인
  -> ros2msg schema를 이용한 CDR decode
  -> Snapshot / Odometry / PacketBytes 객체
  -> PacketBytes.buf 내부 해독은 별도 단계
```

- `mcap.reader.make_reader`와 `mcap_ros2.decoder.DecoderFactory` 사용
- 파일에 등록된 schema가 있어도 메시지 수 0인 topic 존재
- sample 파일은 기본 topic당 1건. 전체 메시지를 JSON으로 전개한 결과는 아님
- raw CSV·JSONL에는 bytes 전체가 아닌 길이·prefix 중심으로 저장
- `jfly`는 확인한 payload 식별 문자열. XYZ 디코더는 없음

### 시간과 좌표

- MCAP `log_time`, `publish_time`, Snapshot `stamp`, Odometry `header.stamp`를 구분
- 작은 초 단위 값이 있으므로 바로 UTC 촬영 날짜로 해석하지 않음
- snapshot ID로 JSON과 매칭. 시간의 완전 일치를 전제하지 않음
- CSV에 quaternion과 파생 Euler 각도 저장. 직접 3D 회전에는 quaternion 사용
- `frame_id`와 `child_frame_id`를 보존
- 현재 통합 표시는 프로젝트 LAS와 MCAP pose를 같은 장면 좌표로 사용
- 센서 변환 YAML을 정리했다고 해서 모든 지도·frame 사이의 정렬을 자동 수행하는 것은 아님

## 9. 점군과 색상 표시

### 현재 뷰어에서의 사용

- 표시하는 3D 좌표와 intensity의 출처는 프로젝트 LAS
- 현재 표시용 비행 궤적은 MCAP odometry에서 추출
- `livetraj.csv`는 프로젝트의 별도 궤적 자료. MCAP odometry와 출처를 구분
- intensity 컬러와 Inspector 스타일 컬러는 LAS의 신호강도를 바탕으로 표시
- 현재 도구가 MCAP LiDAR 바이너리를 해독해 새 LAS를 생성한 것은 아님
- MCAP의 `jfly` 바이너리에서 XYZ·intensity를 복원하는 기능은 미완료

### 9.1 LAS에서 화면까지

1. `map.json`과 색인 CSV로 LAS 연결
2. laspy로 header, scale/offset이 적용된 XYZ와 intensity 읽기
3. 500,000점 단위로 읽으며 일정 간격 샘플링
4. 기본 최대 200,000점의 preview PLY 생성
5. Open3D로 PLY 로드, 필요하면 voxel 또는 표시 점 수 축소
6. 선택 색상과 점 크기로 렌더링

일정 간격 샘플링이라 preview 점 수가 정확히 200,000일 필요는 없음.
기본 상태의 표시 점 수 상한은 이미 생성된 preview의 점 수.
GUI의 `원본 LAS 전체 로드`를 켜면 선택 비행의 LAS를 백그라운드에서 모두 읽고 표시 점 수 상한을 원본 점 수까지 확장.
전체 로드 직후에는 기존 표시 점 수를 유지하며, 사용자가 슬라이더로 단계적으로 늘림.
옵션을 끄거나 비행을 변경하면 다시 가벼운 preview 사용.
원본 LAS 전체 로드는 데이터 크기에 따라 수 GB의 메모리와 GPU 자원을 사용할 수 있음.
항상 더 조밀한 preview가 필요하면 `--max-preview-points`를 늘려 6단계 재실행.

Preview에는 intensity를 최댓값으로 나눈 회색 RGB로 저장.
뷰어는 이 색상의 평균값을 1~99 percentile로 다시 정규화해 표시 강도로 사용.
원래 LAS intensity 수치의 정밀 분석에는 원본 LAS 사용. 일반 컬러 PLY를 가져오면 같은 의미로 해석할 수 없음.

### 9.2 색상 모드

| GUI 모드 | 실제 구현 |
|---|---|
| 회색 | 모든 점에 동일한 회색 |
| 높이 Z 컬러 | Z값의 2~98 percentile 정규화 후 파랑·청록·초록·노랑·흰색 |
| intensity 컬러 | 정규화 intensity에 0.55 지수 적용한 흑백 명암 |
| Inspector 스타일 컬러 | intensity 구간별 팔레트와 밝기 보정 |

Inspector 스타일 구간 경계: 0.12, 0.28, 0.45, 0.62, 0.78, 0.90.
약한 값부터 짙은 보라·보라·분홍·청록·밝은 청록·흰색·노랑 계열.
높이별 색상이 아니며 Inspector 내부 팔레트를 추출한 결과도 아님.
촬영 사진의 RGB를 점군에 입힌 결과와도 구분.

### 9.3 초기 화면과 화면 맞춤

- LiDAR 점군의 XYZ별 0.5~99.5 percentile 범위를 우선 사용
- 극단적인 먼 점의 영향을 줄이고 약 2% 여유 적용
- 원본 점을 삭제하는 필터가 아니라 카메라 맞춤 범위 계산
- 전역 장면 맞춤은 85도 FOV 사용
- 선택 카메라 시점에서는 이 FOV 대신 스냅샷 프러스텀에서 계산한 투영 사용

## 10. 궤적과 POI 표시

### 10.1 비행 경로

`flight_trajectories.csv`에서 선택 비행·topic을 필터링하고 시간순 XYZ를 연결한 주황색 LineSet.
위치·자세는 전 시간대에 추출하지만, GUI에서 비행 시간축 재생이나 모든 시점의 자세 축을 표시하는 기능은 없음.

처음 실행하면 모든 비행을 동시에 합쳐 표시하는 것이 아니라 한 비행 선택.
기본 선택은 스냅샷이 있는 첫 장면, 없으면 첫 장면.
다른 비행은 드롭다운 또는 `--flight-identifier`로 선택.

### 10.2 POI와 스냅샷

- 위치·자세·거리: `mcap_snapshot_events.csv`
- 이미지 경로·POI 이름: `snapshot_json_poses.csv`를 snapshot ID로 연결
- 카메라 pose가 없는 경우 프로젝트 JSON pose로 자동 대체하지 않음
- X축 빨강, Y축 녹색, Z축 파랑의 자세 축
- 현재 표시 기준은 local X 전방, local Y 측방, local Z 상방
- POI는 카메라 위치의 작은 구형 마커
- 미선택 녹색, 선택 노란색, hover 흰색
- 선택·hover 시 마커 크기는 유지
- hover 중 해당 프러스텀 선을 흰색·더 굵은 선으로 강조
- 선택 판정은 POI 중심의 2D 화면 거리 기준, 기본 반경 28px
- 프러스텀 선 자체를 클릭 대상으로 사용하지 않음
- 클릭 또는 `선택 이미지 보기`는 Windows 연결 프로그램으로 JPG 열기

스냅샷의 `camera_pose`는 카메라 pose 자체.
드론-카메라 offset이나 `camera_pitch`를 여기에 다시 적용하지 않음.

## 11. 캘리브레이션과 사진 투영

### 11.1 VIO와 Xacti 구분

| 정보 | 출처·용도 |
|---|---|
| cam0~cam2 YAML | 640 x 400 VIO용, 4K 사진에 사용하지 않음 |
| Xacti K/D/P | Inspector의 `CameraSpecDatabase::xactiCamera(QSize)` |
| 이미지 크기 | 4000 x 3000 |
| 저장 JSON | `calibration/xacti_4000x3000_camera_spec.json` |
| 카메라 pose | MCAP Snapshot |

JSON에는 K, D, P, 이미지 크기, 모델, DLL 출처, 관련 enum, 렌즈-초점 거리 등 저장.

### 11.2 왜곡 보정 흐름

1. 원본 JPG를 OpenCV로 읽되 EXIF 자동 회전 제외
2. 이미지 크기에 맞춰 K와 P 스케일 적용
3. `cv2.fisheye.initUndistortRectifyMap(K, D, I, P, size, CV_16SC2)`
4. `cv2.remap`으로 보정 이미지 생성
5. Pillow로 JPEG quality 95 저장, 원본 EXIF Orientation만 보존
6. 같은 보정 JPG를 사진 평면과 외부 이미지 열기에 사용

#### initUndistortRectifyMap 인자

이 함수는 사진을 바로 보정하는 함수가 아니라, **보정 사진의 각 픽셀을 원본 사진의 어디에서 읽어 올지 정하는 좌표표**를 만드는 함수.
실제 사진 생성은 다음 단계의 `cv2.remap`에서 수행.

| 인자 | 의미 | 현재 코드에서의 사용 |
|---|---|---|
| `K` | 원본 카메라 내부 행렬, 3 x 3 | Xacti JSON의 K를 입력 이미지 크기에 맞춰 사용. `fx/fy`는 픽셀 단위 초점거리, `cx/cy`는 주점 |
| `D` | fisheye 렌즈 왜곡 계수 4개 | Xacti JSON의 `[k1, k2, k3, k4]`. 일반 `cv2.undistort`의 방사·접선 왜곡 계수 배열과 구분 |
| `I` | 공식 인자 이름은 `R`, 보정 좌표계의 회전 행렬 | 현재 `np.eye(3, dtype=np.float64)`인 단위행렬 사용. 추가 시점 회전 없이 렌즈 왜곡 보정 |
| `P` | 보정된 출력 이미지의 투영 행렬 | Xacti JSON의 3 x 3 P 사용. 출력 화각·주점·촬영 범위를 결정하며 프러스텀 계산에도 동일하게 사용 |
| `size` | 출력 이미지 크기 `(width, height)` | 현재 원본과 같은 크기. 4K 스냅샷은 `(4000, 3000)`. NumPy 배열의 `(height, width)` 순서와 반대 |
| `cv2.CV_16SC2` | 첫 번째 좌표표의 저장 형식 | 16-bit 부호 있는 정수 2채널의 고정소수점 좌표표 표현. 사진의 비트 깊이나 컬러 채널 수를 지정하는 값은 아님 |

`I`는 이미지가 아니라 단위행렬을 나타내는 표기.
MCAP `camera_pose`의 3D 자세나 EXIF의 180도 표시 회전과 별개.
`size`가 원본과 같아도 `P`에 따라 원본 가장자리 일부가 출력 범위에서 제외될 수 있음.

반환값은 `map1, map2`.

| 반환값 | `CV_16SC2` 선택 시 내용 |
|---|---|
| `map1` | 원본에서 읽을 x·y 좌표의 정수 부분. 배열 크기 `(height, width, 2)`, 자료형 int16 |
| `map2` | 좌표의 소수 부분에 대응하는 보간 계수표 인덱스. 배열 크기 `(height, width)`, 자료형 uint16 |

이 형식의 `map2`를 단순한 y 좌표표로 해석하지 않음.

이미 원본 이미지 `image`와 크기에 맞춘 `K/D/P`가 준비된 경우의 연결 예시:

```python
import cv2
import numpy as np

height, width = image.shape[:2]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3, dtype=np.float64), P,
    (width, height), cv2.CV_16SC2,
)
undistorted = cv2.remap(
    image, map1, map2,
    interpolation=cv2.INTER_LINEAR,
    borderMode=cv2.BORDER_CONSTANT,
)
```

`INTER_LINEAR`는 주변 픽셀을 이용한 선형 보간, `BORDER_CONSTANT`는 원본 범위 밖을 기본값 0, 즉 검정으로 채우는 설정.

근거: [OpenCV fisheye 함수 인자](https://docs.opencv.org/4.13.0/db/d58/group__calib3d__fisheye.html),
[OpenCV remap과 고정소수점 좌표표](https://docs.opencv.org/4.13.0/da/d54/group__imgproc__transform.html).


원본 픽셀의 EXIF 회전 적용 여부가 Open3D와 OpenCV에서 달라 발생했던 180도 차이 수정.
Open3D에는 원본 픽셀 방향을 유지하고, 외부 뷰어에는 EXIF 방향 정보 제공.

캐시 이름:
`<snapshot-id>_<원본이름>_undistorted_raw_v3.jpg`

원본 JPG와 보정 JSON의 수정 시각보다 캐시가 최신이면 재사용.
현재 캐시는 모든 코드 변경을 자동 추적하는 방식이 아니므로 알고리즘 변경 시 버전 이름 변경 또는 해당 캐시 재생성 필요.
프로젝트 원본 이미지를 덮어쓰지 않음.

### 11.3 프러스텀 모양

P의 초점거리와 주점을 이용해 이미지 네 모서리를 정규화.

```text
x = (u - cx) / fx
y = (v - cy) / fy

corner_world =
    camera_origin
    + forward * length
    + side * (x * length)
    - up * (y * length)
```

- 네 모서리와 카메라 원점을 선으로 연결
- 이미지 평면은 동일한 네 모서리를 사용한 삼각형 2개
- 사진 전체 UV 범위를 평면에 배치, Open3D unlit texture로 표시
- P의 주점이 이미지 중심과 다르므로 프러스텀은 광축 기준 비대칭 가능
- 보정 체크 이전부터 P 기반 프러스텀 생성. 체크는 사진 원본·보정본 전환
- 왜곡된 원본 사진을 이 평면에 붙인 상태는 정확한 렌즈 투영 정합이 아님
- Xacti JSON이 없으면 `--frustum-fov-deg`와 4:3 비율로 근사

### 11.4 프러스텀 길이

| 선택 모드 | 거리 선택 순서 |
|---|---|
| `wall_distance` | 양수 wall_distance → 고정 길이 |
| `LiDAR 점군 추정` | LiDAR 추정 → 양수 wall_distance → 고정 길이 |
| `고정 길이` | `--frustum-length` |

LiDAR 추정은 카메라 전방의 후보 점 중 전방 거리 5 percentile 사용.
기본 검색 거리는 0.05~8 m.
Xacti 사용 시 P의 최대 측방·수직 경계를 이용한 대칭 후보 영역으로 근사.
정확한 광선-벽 교차나 메시 표면 교차 계산은 아님.

사진 평면은 프러스텀 끝의 평평한 면.
굴곡진 벽 전체에 사진을 맞춰 입히는 텍스처 매핑과 구분.

### 11.5 선택 카메라 시점

- 카메라 위치를 MCAP camera_pose 원점으로 이동
- X축 방향과 Z축 up을 이용해 시선 설정
- P 기반 평면 경계를 Open3D 화면 좌표로 변환
- 창 가로세로 비율에 맞춰 여백을 확보한 내부 행렬 적용
- 촬영 위치를 뒤로 옮기거나 프러스텀 크기를 임의 조절하는 방식은 사용하지 않음
- 선택 카메라 모드에서 창 크기·선택 snapshot 변경 시 투영 갱신
- 마우스 드래그·줌 시 고정된 snapshot 시점 추적 해제, `화면 맞춤`으로 전역 시점 복귀

원본 가장자리 일부가 보정 출력 범위에서 제외되는 현상은 화면 맞춤과 별개.
사용자 Inspector5 시각 비교에서 같은 보정 결과 확인. 현재 P의 출력 범위 유지.

## 12. GUI 기능

| 컨트롤 | 동작 |
|---|---|
| MCAP 정보 추출 | 프로젝트/MCAP 입력·출력 선택과 1~7단계 실행 |
| 출력 폴더 변경 | `outputs` 또는 `outputs/lidar` 장면 로드 |
| 비행 선택 | manifest에 등록된 장면 1개 선택 |
| 스냅샷 선택 | 선택 비행의 MCAP snapshot 선택 |
| 선택 이미지 보기 | 보정 체크 상태에 따라 원본 또는 보정 JPG 열기 |
| 선택 시점 보기 | 선택 카메라 위치·방향·투영으로 이동 |
| 사진 평면 표시 | 선택 스냅샷의 평면 texture 표시 |
| 이미지 왜곡 보정 | 보정 파일 생성·재사용 후 표시 전환 |
| 프러스텀 길이 기준 | wall, LiDAR 추정, 고정 길이 |
| 점군 색상 | 회색, Z, intensity, Inspector 스타일 |
| 비행 궤적 표시 | 궤적 보이기·숨기기 |
| LiDAR 점 크기 | 렌더링 점 크기 |
| 원본 LAS 전체 로드 | 선택 비행의 LAS 전량을 읽어 표시 점 수 상한 확장 |
| 표시 점 수 | preview 또는 로드한 원본 LAS에서 실제 렌더링할 점 수 |
| 궤적/카메라 선 굵기 | 궤적·축·프러스텀 선 굵기 |
| 화면 맞춤 | LiDAR 범위 중심 전역 시점 |
| 상태 표시 | 비행명, 원본 LAS·preview·표시 점 수, 궤적·snapshot 수, 보정 여부 |

선택한 사진 평면은 한 번에 1개.
POI 마커 반경은 기본 `axis_length * 0.09`, hover·선택에도 같은 크기.
점 크기·점 수·선 굵기 조절은 파일 내용을 변경하지 않는 표시 설정.

## 13. 실행 옵션

### 13.1 뷰어 전체 옵션

| 옵션 | 기본값 | 의미 |
|---|---|---|
| 위치 인수 `project_root` | 코드 상위의 sepoong | 원본 프로젝트 경로 |
| `--lidar-dir` | 프로젝트 상위 outputs/lidar | manifest·PLY·궤적 경로 |
| `--snapshot-dir` | 프로젝트 상위 outputs/snapshots | snapshot CSV 경로 |
| `--calibration-dir` | 해당 outputs/calibration | Xacti JSON·보정 캐시 경로 |
| `--flight-identifier` | 자동 선택 | 비행 identifier 또는 flight UUID |
| `--topic` | /kalman_scan2map_node/odometry | 표시할 추출 궤적 topic |
| `--axis-length` | 0.5 | 카메라 자세 축 기준 크기 |
| `--frustum-length` | 1.2 | 고정·대체 프러스텀 길이 |
| `--frustum-fov-deg` | 70 | 캘리브레이션 없을 때 수평 FOV |
| `--frustum-length-mode` | wall | wall / lidar / fixed |
| `--lidar-frustum-max-distance` | 8.0 | LiDAR 거리 추정 상한 |
| `--lidar-frustum-min-distance` | 0.05 | LiDAR 거리 추정 하한 |
| `--snapshot-hover-radius-px` | 28 | POI 선택 화면 반경 |
| `--point-size` | 2.0 | 초기 점 크기 |
| `--color-mode` | inspector | gray / height / intensity / inspector |
| `--trajectory-line-width` | 3.0 | 초기 선 굵기. 별칭 `--line-width` |
| `--no-trajectory` | 미지정 | 궤적 숨김. 별칭 `--hide-trajectory` |
| `--max-display-points` | 0 | 0이면 현재 preview 전체 |
| `--voxel-size` | 0.0 | 양수이면 voxel downsample |
| `--window-width` | 1280 | 창 너비, 최소 640 |
| `--window-height` | 840 | 창 높이, 최소 480 |
| `--font-path` | Windows 한글 폰트 탐색 | 사용할 폰트 파일 |
| `--help` | - | 옵션 도움말 |

### 13.2 실행 예시

아래 예시는 6절의 `$PythonExe`, `$Scripts` 변수 기준.

```powershell
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" --flight-identifier E3RDAA23400087_02011_340_1flight
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" --color-mode intensity --point-size 4 --trajectory-line-width 6
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" --no-trajectory --max-display-points 50000
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" --frustum-length-mode lidar --frustum-length 1.5
& $PythonExe -B "$Scripts\06_view_lidar_scene.py" --font-path 'C:\Windows\Fonts\malgun.ttf'
```

각 줄은 독립 실행 예시. 여러 GUI를 동시에 띄우는 배치 용도가 아님.
identifier는 폴더명으로 직접 찾는 것이 아니라 `scene_manifest.json`에서 찾는 선택 키.

## 14. 출력 파일과 재실행

### 14.1 주요 산출물

| 폴더 | 파일 | 내용 |
|---|---|---|
| index | project_summary.json, project_files.csv | 전체 파일·개수·크기 |
| index | flights.csv, maps.csv, snapshots.csv, pois.csv | 프로젝트 엔터티·ID·파일 관계 |
| index | mcap_files.csv, calibration_files.csv, media_files.csv | 단계별 입력 목록 |
| mcap | mcap_files_detail.csv, mcap_channels.csv, mcap_schemas.csv | 기록 파일·topic·schema |
| mcap | mcap_chunks.csv, mcap_metadata.csv | chunk·metadata |
| mcap | mcap_inventory_summary.json | 전체 통계 |
| mcap_samples | mcap_topic_samples.jsonl, mcap_topic_fields.csv | 메시지·필드 샘플 |
| mcap_samples | mcap_pose_samples.csv, mcap_binary_samples.csv | pose·바이너리 요약 |
| snapshots | mcap_snapshot_events.csv | 현재 표시용 MCAP snapshot pose |
| snapshots | snapshot_json_poses.csv | 프로젝트 JSON pose·사진 메타데이터 |
| snapshots | snapshot_mcap_matches.csv, snapshot_timeline.csv | ID 매칭·시간·오차 비교 |
| snapshots | snapshot_pose_summary.json | 7건 비교 등 요약 |
| calibration | camera_intrinsics.csv, camera_extrinsics.csv | VIO 카메라 보정 |
| calibration | static_transforms.csv, geoslam_transforms.csv, transform_edges.csv, frames.csv | 센서 변환 관계 |
| calibration | yaml_files.csv, calibration_summary.json | 보정 파일·hash·요약 |
| calibration | xacti_4000x3000_camera_spec.json | 4K 사진용 보정 |
| calibration/undistorted_images | *_undistorted_raw_v3.jpg | 보정 이미지 캐시 |
| lidar | las_files.csv, las_preview_files.csv | LAS와 preview 관계 |
| lidar/preview_ply | *.preview.ply | 표시용 점군 |
| lidar | processed_livetraj_summary.csv | Inspector 궤적 파일 요약 |
| lidar | odometry_topics.csv, flight_trajectories.csv, flight_trajectory_stats.csv | 추출 궤적·자세 |
| lidar | snapshot_nearest_odometry.csv | snapshot 최근접 odometry 비교 |
| lidar | scene_manifest.json | 비행별 LAS·PLY·궤적·snapshot 장면 연결 |
| raw_lidar | raw_lidar_topics.csv, raw_lidar_topic_stats.csv | LiDAR topic과 통계 |
| raw_lidar | raw_lidar_messages.csv, raw_lidar_samples.jsonl | bytes 요약 |
| raw_lidar | raw_lidar_summary.json | raw 분석 집계 |

추출 CSV는 주로 UTF-8 BOM으로 저장하여 한글 표 확인에 활용.
JSON·JSONL은 UTF-8.
단계별 `*_errors.csv`와 summary의 errors 항목도 함께 확인.

### 14.2 재실행·이동

- 같은 출력 경로로 실행하면 CSV·요약 JSON과 같은 이름의 PLY 등을 갱신
- 출력 폴더 전체를 초기화하거나 사용하지 않는 이전 캐시를 모두 삭제하는 기능은 아님
- 중간 실패 시 이미 생성한 앞 단계 파일은 남음. 자동 rollback 없음
- GUI 완료 표시는 단계별 프로세스 종료 기준. 개별 파일 오류를 수집하고 계속하는 단계도 있어 errors CSV 확인 필요
- 프로젝트별 출력 경로를 나눠 사용하면 데이터 혼합 방지에 유리
- outputs를 옮길 때는 lidar만이 아니라 snapshots·calibration도 함께 유지
- 이미지 원본은 outputs 안으로 자동 복사하지 않으므로 원본 프로젝트도 유지
- 보정 JPG는 표시 또는 이미지 열기가 필요할 때 생성. 모든 snapshot을 미리 일괄 생성하는 기능은 아님
- 입력을 바꿔 재추출한 뒤에도 오래된 preview·캐시 파일이 남을 수 있으므로 목록은 manifest와 현재 CSV 기준

## 15. 오류 확인

| 현상 | 확인·조치 |
|---|---|
| Python 모듈 없음 | 실행한 python.exe 경로와 그 환경의 pip 설치 확인 |
| 출력 삭제 후 화면이 비어 있음 | 정상 초기 상태. MCAP 정보 추출 실행 |
| scene_manifest 없음 | 출력 폴더 확인, 6단계 재실행 |
| manifest는 있지만 PLY 없음 | 생성 도중 실패·파일 이동 확인, 6단계 재실행 |
| 비행 ID를 찾지 못함 | manifest의 identifier 또는 UUID 확인 |
| 다른 출력은 열리는데 사진 없음 | 원본 project_root와 snapshot-dir 일치 여부 확인 |
| 스냅샷 없는 비행 | 촬영 이벤트가 없는 비행은 정상적으로 0개 |
| 보정 체크 비활성 | OpenCV import와 Xacti JSON 존재 확인 |
| robotspec.dll 또는 export 오류 | Inspector 설치 위치·버전 확인, calibration_errors.csv 확인 |
| 보정 사진이 이전 상태 | raw_v3 파일을 보고 있는지 확인, 원본·보정 JSON 수정 시각과 캐시 확인 |
| 보정 후 가장자리 내용 감소 | 보정 P의 출력 범위 문제. Inspector 비교에서 동일 현상 확인 |
| 한글 깨짐 | 맑은 고딕 경로 확인, --font-path 지정 후 재실행 |
| 점군이 듬성듬성 | preview 최대 점 수와 GUI 표시 점 수를 구분 |
| 전체 LAS 로드가 느리거나 종료됨 | 원본 점 수와 시스템 메모리·GPU 메모리 확인, 표시 점 수를 단계적으로 증가 |
| 색상이 실제 사진과 다름 | 현재 색상은 Z·intensity 기반, 사진 RGB 색칠이 아님 |
| GPU·그래픽 초기화 실패 | 로컬 데스크톱·그래픽 드라이버·Open3D 설치 상태 확인 |
| 키보드·마우스 중 비정상 종료 | 실행 콘솔 traceback 또는 종료 시 조작 기록 확보. 현재 테스트가 모든 드라이버·입력 조합을 보장하지 않음 |

한글 폰트 등록은 `ko`, `ko_KR`, 한글 자모·음절 Unicode 범위를 포함.
기본 탐색 순서: 맑은 고딕 일반 → 굵게 → 굴림.

Windows의 현재 Open3D 빌드에서는 EGL 기반 OffscreenRenderer가 지원되지 않는 오류를 확인.
렌더링 검증은 실제 GUI 창 경로로 수행. 웹 브라우저·Playwright 기반 앱은 아님.

## 16. 검증 결과와 한계

### 16.1 실제 데이터 확인

대상은 sepoong MCAP 4개. 다른 프로젝트 전체에 일반화하지 않음.

| 확인 항목 | 결과 |
|---|---|
| MCAP 전체 | 2,714,164개 메시지, 약 3.97 GB |
| profile / encoding | ros2 / cdr, schema는 ros2msg |
| 등록 channel / schema | 파일마다 97 / 54, 실제 메시지 없는 항목 존재 |
| 기본 odometry | 총 7,821건 |
| Snapshot | 7건, JSON과 ID 매칭 7/7 |
| 카메라 pose 차이 | 비교한 7건 모두 위치 0 m, 자세 0 deg |
| Snapshot stamp와 JSON 시간 | 최대 약 40.88 ms 차이 |
| 최근접 odometry 비교 | 최대 약 37.06 ms, drone 위치 약 0.017 m 차이 |
| LiDAR raw | jfly PacketBytes 8,045건, Mask2d 8,045건 |
| 보정 사진 | 사용자 Inspector5 시각 비교에서 같은 결과 확인 |

카메라 pose 일치와 시간값 일치는 서로 다른 결과.
Inspector 시각 비교는 픽셀 단위 정량 검증이나 실제 벽과의 3D 정합 검증을 의미하지 않음.

### 16.2 자동·수동 검증

```powershell
& 'D:\Conda_envs\ROS\python.exe' -B -m unittest discover -s 'D:\Conda_envs\ROS\ROSM\rosm_tools\tests' -v
& 'D:\Conda_envs\ROS\python.exe' -m pip check
```

현재 `test_lidar_camera.py`의 자동 테스트 2개:

- 회전·이동된 카메라의 비대칭 모서리가 가로·세로 viewport에 들어오는지, 사진 평면과 프러스텀 모서리가 같은지 검증
- EXIF Orientation=3 입력에서 보정 전 자동 회전 제외, 출력 EXIF 보존, 이전 이름의 캐시를 재사용하지 않는지 검증

개발 중 실제 Open3D GUI에서 가로·세로 viewport 렌더링과 실제 스냅샷 보정 전후 방향 확인.
2026-09-07 README 갱신 시 실행 진입점 8개의 `--help` 정상 종료, `pip check` 통과.
새 환경 설치·다른 PC·전체 추출 파이프라인을 이번 README 갱신 작업에서 새로 재검증한 것은 아님.

### 16.3 현재 범위 밖

- jfly 내부 XYZ·intensity 복원, MCAP 단독 LAS 생성
- 제거한 `08_probe_jfly.py` / `jfly_probe.py` 기능
- ROS1 BAG의 동등한 수준 일괄 decode
- 영상 재생, 열화상 온도 분석, 궤적 시간축 재생
- 모든 비행의 점군을 한 장면에 자동 병합
- 사진 특징점 기반 자동 지도 정합, 굴곡 표면 텍스처 매핑
- Inspector와 점군 색상·보정 픽셀·3D 프러스텀의 완전한 수치 일치 보장
- 임의 카메라 모델이나 펌웨어의 보정값 자동 식별
- GUI 추출 작업 취소, 추출 설정 영구 저장

## 17. 코드 구성과 참고 문서

`scripts`는 명령 진입점, `rosm`은 실제 구현.
MCAP 구조를 더 분석할 때는 inventory·samples 모듈, 화면 기능은 lidar_viewer 모듈 기준으로 확인.
색인 CSV의 필드와 viewer 소비 필드는 함께 유지해야 하는 연결 규약.

| 문서 | 내용 |
|---|---|
| [elios_drone_data_structure.md](D:/Conda_envs/ROS/ROSM/rosm_tools/elios_drone_data_structure.md) | 프로젝트 폴더·파일 역할, 원본과 outputs 구분 |
| [bag_mcap_file_structure.md](D:/Conda_envs/ROS/ROSM/rosm_tools/bag_mcap_file_structure.md) | BAG/MCAP 구조, 실제 topic·schema, pose 비교, 보정 수치·근거 |

공식 API와 제품 문서:

- [MCAP 규격](https://mcap.dev/spec)
- [MCAP ROS2 decoder](https://mcap.dev/docs/python/mcap-ros2-apidoc/mcap_ros2.decoder)
- [Open3D SceneWidget](https://www.open3d.org/docs/release/python_api/open3d.visualization.gui.SceneWidget.html)
- [OpenCV fisheye](https://docs.opencv.org/doc/doxygen/html/d2/d14/namespacecv_1_1fisheye.html)
- [OpenCV 이미지·EXIF 처리](https://docs.opencv.org/4.13.0/d4/da8/group__imgcodecs.html)
- [Flyability 기록 형식 변경](https://knowledge.flyability.com/third-party-software/faro-connect)
- [Inspector5 데이터 공유·export](https://knowledge.flyability.com/software-and-cloud/inspector/inspector-5/common-workflows/inspector-5-how-to-share-flight-data)

기능 설명은 현재 코드와 로컬 분석 산출물 기준. 웹 문서는 포맷·API·제품 동작의 근거.
