# Gemini Robot

Gemini AI 기반 자율주행 로봇 시스템 - OTA(Observe-Think-Act) 루프 구현

## 개요

이 패키지는 Google Gemini AI를 활용한 자율주행 로봇 시스템입니다. 로봇은 센서 데이터를 관찰하고(Observe), AI로 행동을 계획하며(Think), 실행합니다(Act).

## 아키텍처

### 노드 구성

1. **gemini_planner**: 고수준 목표를 구체적인 태스크로 변환
2. **gemini_vision**: 카메라/센서 데이터를 요약하여 시각 정보 생성
3. **gemini_agent**: OTA 루프를 수행하는 핵심 에이전트

### 토픽 구조

```
/agent_goal (String)      → [planner] → /agent_task (String)
                                      ↘
/camera/image_raw (Image) → [vision] → /vision_digest (String)
/scan (LaserScan)         ↗                            ↓
                                            [agent] → /cmd_vel (Twist)
                                                   → /agent_status (String)
```

## 설치 및 빌드

### 1. Python 의존성 설치

```bash
cd ~/ros2_ws/src/my_gemini_robot
pip install -r requirements.txt
```

### 2. Gemini API 키 설정

```bash
# .env 파일 복사
cp env.example .env

# API 키 설정 (에디터로 열어서 수정)
nano .env
```

또는 환경 변수로 직접 설정:

```bash
export GEMINI_API_KEY="your_api_key_here"
```

API 키 발급: [Google AI Studio](https://aistudio.google.com/apikey)

### 3. ROS2 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select my_gemini_robot
source install/setup.bash
```

## 실행 방법

### 전체 시스템 실행

```bash
ros2 launch my_gemini_robot gemini_robot.launch.py
```

### 에이전트만 실행 (테스트용)

```bash
ros2 launch my_gemini_robot gemini_agent_only.launch.py
```

### 개별 노드 실행

```bash
# 플래너
ros2 run my_gemini_robot gemini_planner

# 비전
ros2 run my_gemini_robot gemini_vision

# 에이전트
ros2 run my_gemini_robot gemini_agent
```

## 사용 예제

### 목표 전송

```bash
ros2 topic pub /agent_goal std_msgs/msg/String "data: '앞으로 5미터 이동'"
```

### 상태 확인

```bash
ros2 topic echo /agent_status
```

## 설정

설정 파일: `config/gemini_params.yaml`

주요 파라미터:
- `timer_period_sec`: 에이전트 루프 주기 (기본: 0.8초)
- `safety_stop_distance_m`: 안전 정지 거리 (기본: 0.5m)
- `max_steps`: 최대 스텝 수 (기본: 300)
- `max_runtime_sec`: 최대 실행 시간 (기본: 600초)
- `max_linear_speed`: 최대 선속도 (기본: 0.5 m/s)
- `max_angular_speed`: 최대 각속도 (기본: 1.0 rad/s)

## 지원 행동

- `move`: 전진/후진/좌우 이동
- `rotate`: 좌우 회전
- `wait`: 대기
- `finish`: 작업 완료

## Gemini API 기능

### 통합된 기능

✅ **행동 계획 (Function Calling)**
- 로봇이 실행할 행동을 AI가 계획
- 관찰 데이터 기반 의사결정
- 안전 제약 조건 고려

✅ **객체 감지 (Object Detection)**
- 이미지에서 객체 찾기
- 2D 좌표 및 라벨 반환
- 경계 상자(Bounding Box) 지원

✅ **Fallback 시스템**
- API 사용 불가 시 규칙 기반 플래너로 자동 전환
- 안정적인 작동 보장

### API 사용 방법

```python
from my_gemini_robot.gemini_api import GeminiRoboticsClient, GeminiConfig

# 클라이언트 초기화
config = GeminiConfig(api_key="your_key")
client = GeminiRoboticsClient(config)

# 객체 찾기
objects = client.find_objects_in_image(
    image_path="scene.jpg",
    queries=["banana", "apple"]
)

# 행동 계획
action = client.plan_robot_action(
    goal="바나나를 집어서 그릇에 넣기",
    observation={"front_range": 1.5, ...},
    constraints={"safety_distance_m": 0.5},
    available_actions=["move", "rotate", "wait", "finish"]
)
```

## TODO

- [ ] 비전 모델 추론 통합 (카메라 입력 → Gemini Vision)
- [ ] 고급 내비게이션 기능
- [ ] 충돌 회피 알고리즘 개선
- [ ] 멀티 로봇 협업 기능
- [ ] 궤적 계획(Trajectory Planning) 통합

## 라이선스

Apache-2.0

## 개발자

robo

