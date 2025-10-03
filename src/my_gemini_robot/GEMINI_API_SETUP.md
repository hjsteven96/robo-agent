# Gemini API 연동 가이드

## 개요

이 문서는 Gemini Robotics-ER 1.5 API를 ROS2 로봇 시스템에 통합하는 방법을 설명합니다.

## 1. API 키 발급

1. [Google AI Studio](https://aistudio.google.com/apikey) 방문
2. Google 계정으로 로그인
3. "Create API Key" 클릭
4. API 키 복사

## 2. 환경 설정

### 방법 1: 환경 변수 (권장)

```bash
export GEMINI_API_KEY="your_api_key_here"
```

영구 설정 (`.bashrc` 또는 `.zshrc`에 추가):

```bash
echo 'export GEMINI_API_KEY="your_api_key_here"' >> ~/.bashrc
source ~/.bashrc
```

### 방법 2: .env 파일

```bash
cd ~/ros2_ws/src/my_gemini_robot
cp env.example .env
nano .env  # API 키 입력
```

## 3. 의존성 설치

```bash
cd ~/ros2_ws/src/my_gemini_robot
pip install -r requirements.txt
```

필수 패키지:
- `google-genai>=0.1.0` - Gemini API 클라이언트
- `Pillow>=10.0.0` - 이미지 처리
- `python-dotenv>=1.0.0` - 환경 변수 관리

## 4. 설정 확인

```bash
python3 -c "
from my_gemini_robot.gemini_api import GeminiRoboticsClient
client = GeminiRoboticsClient()
print('API 사용 가능:', client.is_available())
"
```

## 5. 파라미터 설정

`config/gemini_params.yaml`:

```yaml
use_gemini_api: true              # API 사용 여부
gemini_model_id: "gemini-robotics-er-1.5-preview"
gemini_temperature: 0.5           # 창의성 (0.0-1.0)
gemini_thinking_budget: 0         # 사고 예산 (0=빠름, 높은값=정확)
```

## 6. 기능 설명

### Function Calling (행동 계획)

Gemini API가 로봇의 다음 행동을 결정합니다:

- **입력**: 목표, 관찰 데이터, 제약 조건
- **출력**: `{"function": "move", "args": {"direction": "forward", "speed": 0.2}}`
- **지원 행동**: `move`, `rotate`, `wait`, `finish`

### Object Detection (객체 감지)

이미지에서 객체를 찾아 좌표를 반환합니다:

- **입력**: 이미지 파일
- **출력**: `[{"point": [y, x], "label": "banana"}, ...]`
- **좌표**: 0-1000 정규화된 [y, x] 형식

### Thinking Budget (사고 예산)

응답 속도와 정확도 조절:

- `0`: 빠른 응답 (기본, 공간 이해 작업용)
- `1-10`: 더 깊은 추론 (복잡한 계획 작업용)

## 7. 사용 예제

### 에이전트 노드에서 자동 사용

```bash
ros2 run my_gemini_robot gemini_agent
```

에이전트가 자동으로 Gemini API를 사용하여 행동을 계획합니다.

### Python 스크립트에서 직접 사용

```python
from my_gemini_robot.gemini_api import GeminiRoboticsClient, GeminiConfig

# 클라이언트 초기화
config = GeminiConfig(
    api_key="your_key",
    temperature=0.5,
    thinking_budget=0
)
client = GeminiRoboticsClient(config)

# 행동 계획
action = client.plan_robot_action(
    goal="앞으로 3미터 이동",
    observation={
        "front_range": 2.5,
        "left_range": 1.0,
        "right_range": 1.2
    },
    constraints={
        "safety_distance_m": 0.5,
        "max_runtime_sec": 600
    },
    available_actions=["move", "rotate", "wait", "finish"]
)

print(f"계획된 행동: {action}")
```

## 8. 문제 해결

### API 키 오류

```
경고: GEMINI_API_KEY 환경 변수가 설정되지 않았습니다.
```

**해결**: 환경 변수 설정 확인

```bash
echo $GEMINI_API_KEY
```

### 패키지 import 오류

```
Warning: google-genai 패키지가 설치되지 않았습니다.
```

**해결**: 의존성 재설치

```bash
pip install --upgrade google-genai
```

### API 호출 실패

로그에 표시되는 오류:

```
Gemini API 호출 실패: [오류 메시지]. Fallback 사용.
```

**대응**: 시스템이 자동으로 규칙 기반 플래너로 전환됩니다.

## 9. Fallback 시스템

API를 사용할 수 없을 때 자동으로 규칙 기반 플래너가 작동합니다:

1. 전방 장애물 감지 → 우회전
2. 목표 완료 키워드 감지 → 종료
3. 기본 행동 → 전진

## 10. 비용 및 제한

- **무료 할당량**: Google AI Studio에서 제공
- **요금제**: [Google AI Pricing](https://ai.google.dev/pricing) 참고
- **Rate Limit**: API 호출 빈도 제한 주의

## 11. 보안 주의사항

⚠️ **API 키를 Git에 커밋하지 마세요!**

`.gitignore`에 포함된 항목:
- `.env`
- `*.key`

## 12. 추가 리소스

- [Gemini Robotics-ER 문서](https://ai.google.dev/gemini-api/docs/robotics)
- [Function Calling 가이드](https://ai.google.dev/gemini-api/docs/function-calling)
- [로보틱스 쿡북](https://github.com/google-gemini/gemini-api-cookbook)

## 문의

문제가 발생하면 이슈를 등록하거나 개발자에게 문의하세요.

