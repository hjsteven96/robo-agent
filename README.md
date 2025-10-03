# ROS2 Gemini Robot Workspace

Google Gemini AI를 활용한 ROS2 자율주행 로봇 시스템

## 📋 프로젝트 개요

이 워크스페이스는 Google Gemini AI API를 ROS2와 통합하여 지능형 자율주행 로봇을 구현합니다. 
OTA(Observe-Think-Act) 루프를 통해 로봇이 주변 환경을 인식하고, AI로 행동을 계획하며, 실행합니다.

## 🚀 주요 기능

- **🧠 AI 기반 행동 계획**: Gemini Function Calling을 활용한 실시간 의사결정
- **👁️ 객체 인식**: Gemini Vision API를 통한 이미지 기반 객체 감지
- **🔄 OTA 루프**: 관찰(Observe) → 사고(Think) → 행동(Act) 사이클
- **🛡️ 안전 메커니즘**: 장애물 감지 및 자동 정지 기능
- **📡 ROS2 통합**: 표준 ROS2 토픽/서비스를 통한 완전한 통합

## 📁 프로젝트 구조

```
ros2_ws/
├── src/
│   └── my_gemini_robot/        # 메인 ROS2 패키지
│       ├── my_gemini_robot/     # Python 모듈
│       │   ├── gemini_api.py    # Gemini API 클라이언트
│       │   └── gemini_bridge.py # ROS2-Gemini 브릿지
│       ├── config/              # 설정 파일
│       ├── launch/              # 런치 파일
│       └── README.md            # 패키지 상세 문서
├── build/                       # 빌드 아티팩트 (gitignore)
├── install/                     # 설치 파일 (gitignore)
└── log/                         # 로그 (gitignore)
```

## 🛠️ 설치 및 실행

### 사전 요구사항

- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Python 3.10+
- Google Gemini API 키

### 빠른 시작

```bash
# 1. 워크스페이스 클론
git clone https://github.com/YOUR_USERNAME/ros2_gemini_robot.git
cd ros2_gemini_robot

# 2. 의존성 설치
./install_dependencies.sh

# 3. 환경 설정
cd src/my_gemini_robot
cp env.example .env
# .env 파일에 GEMINI_API_KEY 입력

# 4. 빌드
cd ~/ros2_ws
colcon build --packages-select my_gemini_robot
source install/setup.bash

# 5. 실행
ros2 launch my_gemini_robot gemini_robot.launch.py
```

### API 키 발급

[Google AI Studio](https://aistudio.google.com/apikey)에서 무료 API 키를 발급받으세요.

## 📖 상세 문서

- [패키지 상세 문서](src/my_gemini_robot/README.md)
- [Gemini API 설정 가이드](src/my_gemini_robot/GEMINI_API_SETUP.md)
- [카메라 비전 통합](src/my_gemini_robot/CAMERA_VISION_INTEGRATION.md)
- [빠른 비전 설정](src/my_gemini_robot/QUICK_VISION_SETUP.md)

## 🎯 사용 예제

```bash
# 로봇에게 목표 전송
ros2 topic pub /agent_goal std_msgs/msg/String "data: '앞으로 3미터 이동 후 오른쪽으로 회전'"

# 상태 모니터링
ros2 topic echo /agent_status

# 속도 명령 확인
ros2 topic echo /cmd_vel
```

## 🏗️ 아키텍처

```
┌─────────────┐
│ agent_goal  │ 사용자 입력
└──────┬──────┘
       ↓
┌─────────────┐
│   planner   │ 고수준 → 저수준 변환
└──────┬──────┘
       ↓
┌─────────────┐    ┌─────────────┐
│   vision    │ ← →│    agent    │ OTA 루프
└─────────────┘    └──────┬──────┘
                          ↓
                   ┌─────────────┐
                   │   cmd_vel   │ 로봇 제어
                   └─────────────┘
```

## 🧪 테스트

```bash
# Python 테스트
cd src/my_gemini_robot
python3 -m pytest test/

# ROS2 테스트
colcon test --packages-select my_gemini_robot
```

## 🤝 기여

기여를 환영합니다! 이슈나 풀 리퀘스트를 자유롭게 제출해주세요.

## 📄 라이선스

이 프로젝트는 Apache-2.0 라이선스 하에 배포됩니다.

## 👤 개발자

robo

## 🔗 관련 링크

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Google Gemini API](https://ai.google.dev/)
- [Gemini Function Calling](https://ai.google.dev/docs/function_calling)

---

⚠️ **참고**: 이 프로젝트는 개발 중이며, API 키가 필요합니다.

