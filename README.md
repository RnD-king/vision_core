# 공통 비전 코어 사용 설명서

`vision_core`는 실제 카메라용 ROS2 `vision` 패키지와 G1 시뮬레이터가
같은 비전 계산과 속도 명령 계산을 사용하도록 만든 독립 C++ 라이브러리다.

별도 노드로 실행하는 프로그램이 아니다. `vision`과 G1의 프로세스 안에서
공유 라이브러리 함수가 바로 호출되므로, 코어를 분리한 것 때문에 ROS2 토픽이
추가되거나 직렬화 비용이 생기지 않는다.

## 역할 구분

### 실제 카메라용 `vision` 패키지

`/home/noh/my_cv/src/vision`에 있으며 장치와 ROS2에 관련된 일을 담당한다.

- 카메라 영상과 IMU 토픽 수신
- CUDA 전처리와 TensorRT YOLO 추론
- YOLO 검출 결과를 공통 코어 자료형으로 변환
- 화면 표시와 진단 로그
- 코어가 선택한 최종 속도를 ROS2 `Twist`로 변환
- `/g1_vision/cmd_vel` 토픽 발행

### 공통 `vision_core`

`/home/noh/vision_core`에 있으며 실제 판단과 제어 계산을 담당한다.

- IMU roll/pitch를 이용한 픽셀 좌표 보정
- 점선 중심점에서 8차원 특징 계산
- 점선 추종 속도 계산
- 점선을 놓쳤을 때 최근 방향과 이동 경로 기억을 이용한 복구
- 공·골대·백보드·허들 중 사용할 검출 대상 선택
- 공·허들·골대 검출 안정화와 접근 상태 전환
- 공·허들·골대 접근 속도 및 임시 동작 요청 계산
- 골대 > 공 > 허들 > 점선 우선순위로 최종 명령 선택

### G1 시뮬레이터

G1은 Python `ctypes` 연결층을 통해 같은
`libshared_vision_core.so`를 호출한다. 따라서 특징 수식이나 규칙기반 속도
수식을 코어에서 변경하면 실제 비전과 G1에 같은 계산을 적용할 수 있다.

현재 G1의 점선 계산은 코어에 연결되어 있다. PID 경기장 실행에서는 현재 순번의
농구공 하나를 가상 카메라 bbox로 만들어 공 C API에도 전달한다. 골대·허들
컨트롤러와 C API는 준비됐지만 G1 경기장 객체/bbox 입력 생성은 아직 연결 전이다.

## 실제 비전 노드의 처리 순서

```text
[vision] ROS2 realsense 영상, IMU 토픽 수신
    ↓
[vision] CUDA 영상 전처리
    ↓
[vision] TensorRT YOLO 추론 → 객체별 bbox 생성
    ↓
[vision] 라인 bbox → 라인 중심점 목록 생성
[vision] 미션 물체 bbox → 공·골대·백보드·허들 좌표 목록 생성
    ↓
[vision_core] 객체별 좌표 목록 중 필요한 좌표 선택
    ↓
[vision_core] IMU 기반 카메라 좌표 보정
    ↓
[vision_core] 라인 8차원 특징 계산
    ↓
[vision_core] 라인 추종 명령 계산
[vision_core] 미션 물체 접근 명령 계산
    ↓
[vision] 객체별 우선순위 따라 선택된 vx, vy, wz를 토픽으로 발행
```

## `line_detection_adapter`가 라인에만 있는 이유

YOLO는 라인도 사각형 bbox로 검출한다. 점선 특징 계산에는 사각형 자체보다
여러 점의 중심 좌표가 필요하기 때문에 `line_detection_adapter`가 다음 변환을
담당한다.

```text
라인 bbox 여러 개 → std::vector<cv::Point2f> 중심점 목록
```

Python 딕셔너리가 아니라 C++ `std::vector` 목록이다.

공·골대·백보드·허들은 별도의 어댑터 파일이 없다.
`line_perception_node.cpp`의 `ToCoreDetections()`가 YOLO의 모든 bbox를
`std::vector<vision_core::Detection>`으로 한꺼번에 변환하고,
`vision_core::ExtractObjectTargets()`가 class별 대상을 선택한다.

## 파일별 역할

```text
vision_core/
├── include/vision_core/
│   ├── types.hpp
│   ├── coordinate_rectifier.hpp
│   ├── line_feature_extractor.hpp
│   ├── line_velocity_controller.hpp
│   ├── object_target_extractor.hpp
│   ├── ball_controller.hpp
│   ├── hurdle_controller.hpp
│   ├── goal_controller.hpp
│   ├── motion_command_selector.hpp
│   └── c_api.h
├── src/
│   ├── coordinate_rectifier.cpp
│   ├── line_feature_extractor.cpp
│   ├── line_velocity_controller.cpp
│   ├── object_target_extractor.cpp
│   ├── ball_controller.cpp
│   ├── hurdle_controller.cpp
│   ├── goal_controller.cpp
│   ├── motion_command_selector.cpp
│   └── c_api.cpp
└── CMakeLists.txt
```

- `types`: 점, bbox, 검출 결과, 특징, 속도 명령 등 공통 자료형
- `coordinate_rectifier`: IMU 기반 좌표 보정식
- `line_feature_extractor`: 점선 중심점에서 8차원 특징 계산
- `line_velocity_controller`: 점선 추종과 점선 누락 복구 속도 계산
- `object_target_extractor`: class별 신뢰도 조건을 적용하고 사용할 객체 선택
- `ball_controller`: 공 검출 안정화, 원거리 접근, 카메라 하향, 임시 정지 시퀀스
- `hurdle_controller`: 허들 접근, 하향 감속, 잔발/넘기 placeholder 시퀀스
- `goal_controller`: 집기 후 대기, 골대 탐색/접근, 미세조정/슛 placeholder 시퀀스
- `motion_command_selector`: 골대·공·허들·점선 후보 중 최종 명령 선택
- `c_api`: G1 Python에서 C++ 코어를 호출하기 위한 연결 인터페이스

## 현재 공 접근 임시 시퀀스

실제 pickup/미세 한걸음 모션이 준비되기 전까지 공 제어기는 다음 상태만
실행한다.

```text
LINE_FOLLOW
  -> BALL_APPROACH
  -> CAMERA_TILT_DOWN_AND_APPROACH
  -> BALL_FINE_ADJUST (현재 1초 저속 직진 placeholder)
  -> BALL_PICKUP (현재 3초 정지 placeholder)
  -> BALL_PICKUP_VERIFY (향후 실제 집기 확인)
  -> BALL_STAND_UP (향후 일어나기 모션)
  -> CAMERA_RETURN_TO_LINE
  -> LINE_FOLLOW (공 검출 30초 무시)
```

- line 후보명령은 ball 활성 여부와 관계없이 별도로 계산·보관한다.
- line 후보와 함께 `line_reference_valid`를 전달한다. 이 값은 정상 `TRACK`일
  때만 true이고, 양수 속도를 내는 `RECOV`/coast/search에서도 false다. 코어는
  가장 최근의 유효한 양수 line `vx`를 `BALL_APPROACH` 전이에 한 번
  고정한다. 원거리 접근 `vx`는 `고정한 line_vx * far_speed_scale`이며 이후
  백그라운드 line controller의 복구속도와 무관하다.
- 공의 raw 화면 중심이 영상 높이의 75% 아래인 판정이 최근 10프레임 중
  7프레임 이상일 때 카메라 하향 상태로 들어간다. 조건을 벗어난 프레임이나
  공을 놓친 프레임은 실패 프레임으로 기록하되 누적 판정을 즉시 초기화하지 않는다.
- 공 자체의 최초 안정 검출도 최근 10프레임 중 7프레임을 사용한다.
- 카메라 전환 중에는 직전 접근속도를 축소한 값으로 직진하고 `wz=0`을 쓴다.
- feedback API에서는 카메라가 실제 DOWN+settled를 보고한 뒤에만 탑뷰
  `BALL_FINE_ADJUST`를 시작한다. 현재는 실제 미세보행 대신 1초 저속 직진을 쓰며,
  기존 API는 호환을 위해 설정된 시간으로 카메라 도달을 추정한다.
- 30초 무시 시간은 전방 카메라가 실제로 복귀·안정화되어 line 추종이 다시
  시작되는 순간부터 센다. 공 tracker만 무시하며 정상 line 기준속도는 다음 공을
  위해 계속 갱신한다.
- G1의 90° 가상 카메라는 몸통 roll/pitch를 점진적으로 상쇄하여 endpoint의
  광축을 월드 바닥 수직(-Z)에 맞춘다.
- `BALL_PICKUP_VERIFY`와 `BALL_STAND_UP`은 현재 0초 placeholder라 각 한 프레임씩
  상태를 표시한 뒤 넘어간다. 이후 실제 집기 확인/일어나기 완료 피드백으로 바꾼다.
- RealSense depth 교차검증, 시뮬레이터 거리 노이즈, 실제 pickup, pickup 후
  전용 line 재진입은 의도적으로 이번 구현 범위에서 제외했다.

G1 Python은 기존 `VisionBallResult`와 기존 함수 ABI를 바꾸지 않고 새
`vision_ball_controller_compute_v3()`로 line 속도, 정상 TRACK 기준값 여부,
실제 카메라 상태를 전달한다. v2도 호환을 위해 남아 있다.

C++의 `camera_request`는 `NONE`, `DOWN`, `FORWARD`, `GOAL` 네 상태다. 평소와
카메라 자세 유지 중에는 `NONE`이고, 자세를 바꿀 때만 목표 자세 요청을
실제 도달 피드백이 올 때까지 매 프레임 반환한다. ROS2 연결 코드는 `NONE`일 때
토픽을 발행하지 않는다. 기존 G1 C API의 `request_camera_down`은 ABI 호환을 위해
종전의 자세 level 신호(`1=DOWN`, `0=FORWARD`)로 변환해서 유지한다.

## 현재 허들 임시 시퀀스

```text
LINE_FOLLOW
  -> HURDLE_APPROACH
  -> HURDLE_CAMERA_TILT_DOWN_AND_SLOW
  -> HURDLE_CONTACT_WALK (현재 2초 저속 직진 placeholder)
  -> HURDLE_CROSS (현재 3초 정지 placeholder)
  -> HURDLE_CAMERA_RETURN_TO_LINE
  -> LINE_FOLLOW
```

- bbox 아래쪽이 영상 높이의 82%를 넘는 판정이 최근 10프레임 중 7프레임이면
  카메라 하향/감속 단계로 들어간다.
- 허들 자체의 최초 안정 검출도 최근 10프레임 중 7프레임을 사용한다.
- `action_request=CONTACT_WALK/CROSS`는 이후 G1 모션 패키지가 실행할 동작 종류다.
- 실제 완료 피드백이 연결되기 전에는 각각 설정된 placeholder 시간으로 넘어간다.

## 현재 골대 임시 시퀀스

```text
StartAfterPickup
  -> GOAL_POST_PICKUP_WAIT (2초)
  -> CAMERA_TILT_TO_GOAL_VIEW
  -> GOAL_SEARCH
  -> GOAL_APPROACH
  -> GOAL_FINE_ADJUST (현재 2초 정지 placeholder)
  -> GOAL_SHOOT (현재 3초 정지 placeholder)
  -> CAMERA_RETURN_TO_LINE_VIEW
  -> GOAL_HEADING_RECOVERY
  -> LINE_FOLLOW
```

- 실제 집기 확인 뒤 `StartAfterPickup()`을 한 번 호출해야 골대 미션이 시작된다.
- 골대 최초 검출과 미세조정 진입은 각각 최근 10프레임 중 7프레임을 사용한다.
- `GOAL_VIEW`는 골대를 보는 수평 시야, `FORWARD`는 기존 라인용 평소 시야다.
- 카메라가 평소각에 도달하면 골대 제어는 속도를 덮어쓰지 않고 line controller의
  RECOV 명령을 사용한다. `line_reference_valid=true`가 되면 정상 트래킹으로 끝난다.
- 미세조정과 슛은 현재 `v=0`이지만 `action_request`와 상태는 실제 모션 연결에
  사용할 수 있도록 분리돼 있다.

라인도 동일하게 최근 10프레임 중 중앙에 안정적으로 잡힌 프레임이 7개 이상일
때만 복구를 종료한다. 이후 최대 3프레임 누락은 정상 추종을 유지하고, 네 번째
실패 프레임부터 다시 복구 상태로 들어간다.

여기서 보존한다고 말하는 ABI는 C API의 기존 함수와 `VisionBallResult` 배치다.
C++ 공개 class/struct가 바뀌었으므로 C++ 소비자는 코어 설치 후 반드시 다시
빌드해야 한다. 이미 line 후보를 계산한 C 호출자는 상태를 두 번 갱신하지 않도록
선택만 수행하는 `vision_select_motion_command_v2()`를 사용한다.

ROS2 `line_perception_node`는 실제 카메라 actuator 요청/도달 feedback 토픽이
아직 없으므로 `enable_ball_controller=false`가 기본이다. 이를 수동으로 켜면
시간 추정 compatibility 경로로 알고리즘 화면 확인은 가능하지만 실제 로봇용
카메라 동작 연결로 간주하면 안 된다. 실제 endpoint feedback을 쓰는 현재 소비자는
G1 Python bridge다.

## 코어 빌드와 설치

코어를 수정한 뒤에는 단순히 `cmake --build`만 하지 말고, 아래 순서를
사용하는 것이 가장 안전하다.

```bash
cd /home/noh/vision_core

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install

cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build
```

각 명령의 의미는 다음과 같다.

1. `cmake -S . -B build`: 소스 파일과 `CMakeLists.txt` 변경을 빌드 설정에 반영
2. `cmake --build build -j`: 공유 라이브러리와 회귀 테스트 빌드
3. `ctest --test-dir build --output-on-failure`: 공·허들·골대 상태 전이, 시간 경계,
   C API와 명령 선택 회귀 검사
4. `cmake --install build`: 새 라이브러리와 헤더를 실제 사용 위치인
   `/home/noh/vision_core/install`에 설치

마지막 `cmake --install`을 생략하면 `vision`과 G1이 계속 이전에 설치된
라이브러리를 사용할 수 있으므로 반드시 실행한다.

## 어떤 것을 다시 빌드해야 하는가

먼저 아래 표로 판단한다.

| 변경 내용 | 코어 빌드·설치 | `vision` 빌드 | G1 쪽 처리 |
|---|---:|---:|---|
| 특징값의 계산식만 변경, 특징 개수·이름·순서는 동일 | 필요 | 불필요 | 실행 중이면 재시작 |
| 점선·공 속도 수식이나 복구·명령 선택 조건만 변경 | 필요 | 불필요 | 실행 중이면 재시작 |
| 특징 개수·이름·순서 또는 C++ 자료형 변경 | 필요 | 필요 | `core_bridge.py`와 G1 특징 구성 코드 수정 후 재시작 |
| 코어 함수 추가·삭제·인자 변경 또는 C API 변경 | 필요 | 필요 | 사용하는 C API라면 `core_bridge.py`도 수정 |
| 코어에 `.cpp` 파일 추가 또는 `CMakeLists.txt` 변경 | 필요 | 새 기능을 `vision`이 사용하면 필요 | 새 기능을 G1이 사용하면 연결 코드 수정 |
| 카메라·IMU 토픽, YOLO, 화면 표시, ROS 발행만 변경 | 불필요 | 필요 | 불필요 |
| G1 경기장·센서 모사·Python 연결 코드만 변경 | 불필요 | 불필요 | G1 프로세스 재시작 |

여기서 **코어 빌드·설치**는 항상 다음 세 명령 전체를 의미한다.

```bash
cd /home/noh/vision_core

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install

cmake --build build -j
cmake --install build
```

### 코어의 계산식 `.cpp`만 수정한 경우

예를 들어 좌표 보정식, 특징 수식, 점선 속도 수식, 공 접근 수식 또는 명령
선택 조건만 수정했다면 코어만 빌드하고 설치하면 된다.

```bash
cd /home/noh/vision_core

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/noh/vision_core/install

cmake --build build -j
cmake --install build
```

함수 이름과 공개 자료형이 그대로라면 `vision` 패키지는 다시 빌드하지 않아도
새로 설치된 공유 라이브러리를 실행 시 불러온다. 이미 실행 중인 ROS2 노드나
G1 시뮬레이터는 종료한 뒤 다시 실행해야 새 라이브러리가 적용된다.

### 특징 벡터의 수식만 변경한 경우

현재 특징의 개수·이름·순서를 유지하면서 각 특징값의 계산식만 바꾼다면
`line_feature_extractor.cpp`를 수정하고 코어만 빌드·설치한다. `vision`은
다시 빌드하지 않아도 되며, `vision` 노드와 G1은 종료 후 다시 실행한다.

다만 BC와 PPO 모델은 이전 특징값 분포로 학습되어 있다. 특징 차원은 같아도
값의 의미나 범위가 크게 달라지면 기존 체크포인트와 정규화 통계가 더 이상 잘
맞지 않을 수 있으므로 성능을 다시 확인하고 필요하면 데이터 생성과 학습을 다시 한다.

### 특징 벡터의 개수·이름·순서를 변경한 경우

이 경우는 단순한 수식 변경이 아니라 인터페이스 변경이다. 현재 G1은 기본 특징
8개와 `history_state`를 이용해 관측 차원을 만들고, BC 체크포인트의
`feature_mean`, `feature_std`, `feature_names`도 그 구조를 기준으로 사용한다.

따라서 다음 항목을 함께 맞춰야 한다.

1. `vision_core/include/vision_core/types.hpp`의 `Features`
2. `vision_core/include/vision_core/c_api.h`의 `VisionLineFeatures`
3. `vision_core/src/c_api.cpp`의 C++·C 자료형 변환
4. G1 `core_bridge.py`의 `ctypes` `_Features`
5. G1 `perception/features.py`의 `BASE_FEATURE_NAMES`와 특징 묶음
6. G1 `vision_rl/config.py`의 관측 차원과 관련 설정
7. 필요하면 `vision` 노드에서 해당 특징을 사용하는 로그·제어 코드

변경 후 순서는 다음과 같다.

```text
코어 소스·헤더·C API 수정
    ↓
vision_core 빌드·설치
    ↓
vision 패키지 colcon 빌드
    ↓
G1 Python 연결 및 관측 차원 수정
    ↓
기존 데이터셋·BC·PPO 모델 호환성 확인
```

특징 개수나 순서가 달라지면 기존 데이터셋, BC 체크포인트, PPO 체크포인트는
입력 차원이나 의미가 달라져 그대로 사용할 수 없다. 새 특징 기준으로 데이터셋을
다시 만들고 BC·PPO 모델을 다시 학습하는 것이 원칙이다.

### 코어 헤더·함수·C API 또는 `CMakeLists.txt`를 수정한 경우

코어를 먼저 빌드·설치하고, 그다음 `vision` 패키지도 다시 빌드한다.

```bash
cd /home/noh/my_cv
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select vision \
  --allow-overriding vision

source install/setup.bash
```

G1은 ROS2 패키지가 아니므로 `colcon build`하지 않는다. C API의 자료형이나
함수 인자가 바뀌었다면 `/home/noh/G1/g1_rl_workspace/legged_gym/vision/g1/core_bridge.py`
의 `ctypes` 선언을 맞춘 뒤 G1 실행을 다시 시작한다.

### `vision` 노드나 YOLO 연결 코드만 수정한 경우

코어 계산을 바꾸지 않았다면 `vision` 패키지만 다시 빌드하면 된다.

```bash
cd /home/noh/my_cv
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select vision \
  --allow-overriding vision

source install/setup.bash
```

## 수정 위치 기준

- 좌표 보정식: `src/coordinate_rectifier.cpp`
- 점선 특징 종류와 수식: `src/line_feature_extractor.cpp`
- 점선 추종·복구 수식: `src/line_velocity_controller.cpp`
- 객체 선택 조건: `src/object_target_extractor.cpp`
- 공 접근 상태와 속도 수식: `src/ball_controller.cpp`
- 허들 접근 상태와 임시 동작: `src/hurdle_controller.cpp`
- 골대 접근 상태와 임시 동작: `src/goal_controller.cpp`
- 골대·공·허들·점선 최종 우선순위: `src/motion_command_selector.cpp`
- G1에 새 기능 공개: `include/vision_core/c_api.h`, `src/c_api.cpp`
- 카메라·IMU 토픽과 YOLO 추론: `/home/noh/my_cv/src/vision`

특징 종류나 속도 수식을 바꿀 때는 가능하면 `vision`과 G1에 각각 같은 계산을
복사하지 말고 코어에서 한 번만 변경한다.

## 마무리 전 주의사항

- 특징·좌표 보정·점선 제어·공 제어·최종 명령 우선순위의 기준 코드는
  `vision_core`다. 같은 계산을 `vision`과 G1에 각각 복사하지 않는다.
- 코어 헤더, C API 또는 `CMakeLists.txt`까지 변경했다면
  `vision_core` 빌드·설치 후 `vision`도 `colcon build`한다.
- G1 Python 파일만 변경했다면 별도 빌드는 필요 없고 실행 중인 G1을 종료한 뒤
  다시 실행하면 된다.
- 특징 개수나 순서를 변경하면 `core_bridge.py`, G1 관측 차원, 데이터셋,
  정규화 통계, BC·PPO 체크포인트의 호환성을 모두 다시 확인한다.
- 현재 G1 PID 경기장에는 점선과 현재 순번 농구공 하나가 시뮬레이션 입력으로
  연결되어 있다. 골대·백보드·허들용 C API는 준비되어 있지만 G1 입력 생성은
  아직 연결하지 않았다.
- 공은 접근 상태와 속도 계산이 구현되어 있다. 골대·백보드·허들은 대상 선택과
  좌표 보정까지만 있고 행동·속도 수식은 아직 없다.
- 현재 코스 완주 판정은 없다. 마지막 점선을 지나면 점선 누락으로 판단해 복구
  상태에 들어가며, 선호 방향 두 구간과 반대 방향 한 구간의 탐색을 반복한다.
- 실제 ROS2 비전 노드는 아직 로봇 위치와 경기장 기준 경로를 코어에 전달하지
  않는다. 따라서 G1의 위치·경로 기반 복구와 달리 누적된 점선 좌우 방향 기억을
  이용한 복구만 사용할 수 있다.
- 코어 변경 후 `cmake --install build`를 생략하면 `vision`과 G1이 이전에
  설치된 공유 라이브러리를 계속 사용할 수 있다.
- 변경을 마무리할 때는 G1 PID 실행과 실제 `line_perception_node` 실행을 각각
  확인한다.
