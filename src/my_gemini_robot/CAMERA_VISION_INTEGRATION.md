# 카메라 비전 통합 가이드

## 📸 **현재 상태**

### ✅ 이미 구현됨:
1. **GeminiVisionNode** - 카메라 이미지 수신
2. **vision_digest** 토픽 - 비전 정보 전달
3. **Observation.seen** - 보인 객체 추적

### ❌ 미구현:
1. **실제 이미지 분석** - 현재는 더미 데이터만 반환
2. **Gemini Vision API** - 이미지를 Gemini에게 전달하여 분석
3. **목표 기반 객체 탐지** - "우체통", "문" 등 탐지

## 🎯 **개선 방안**

### 방법 1: Gemini Vision API 통합 (권장)

Gemini 2.0은 멀티모달 모델로 이미지를 직접 분석할 수 있습니다.

```python
def plan_robot_action_with_vision(
    self,
    goal: str,
    observation: Dict[str, Any],
    image_data: Optional[bytes] = None,  # ← 이미지 추가
    constraints: Dict[str, Any] = {},
    available_actions: List[str] = []
) -> Dict[str, Any]:
    """
    이미지와 센서 데이터를 함께 분석하여 행동 계획
    """
    contents = []
    
    # 텍스트 프롬프트
    prompt = f"""
    Goal: {goal}
    
    Current sensor data:
    - Front: {observation.get('front_range')}m
    - Left: {observation.get('left_range')}m  
    - Right: {observation.get('right_range')}m
    
    **ANALYZE THE CAMERA IMAGE BELOW:**
    - What objects do you see?
    - Is there a door? mailbox? person?
    - Which direction should I move to reach the goal?
    
    Choose ONE action based on both sensor data AND visual information.
    """
    contents.append(prompt)
    
    # 이미지 추가
    if image_data:
        contents.append({
            "mime_type": "image/jpeg",
            "data": image_data  # base64 또는 bytes
        })
    
    response = self.client.models.generate_content(
        model=self.config.model_id,
        contents=contents,
        config=types.GenerateContentConfig(
            temperature=self.config.temperature,
            thinking_config=types.ThinkingConfig(
                thinking_budget=self.config.thinking_budget,
                include_thoughts=True
            ),
            tools=[types.Tool(function_declarations=function_declarations)]
        )
    )
    
    return self._parse_response(response)
```

### 방법 2: 주기적 비전 체크 (효율적)

매 스텝마다 이미지 분석은 비용이 큽니다. 대신:

```python
# 특정 조건에서만 비전 활성화
if self.step_count % 5 == 0:  # 5스텝마다
    vision_analysis = self.gemini_client.analyze_image(
        self.last_image,
        query="What objects do you see? Any doors or mailboxes?"
    )
    self.vision_digest.update(vision_analysis)
```

### 방법 3: 객체 탐지 파이프라인 (최적)

1. **빠른 객체 탐지 모델** (YOLO, MobileNet) - 로컬 실행
2. **탐지된 객체를 Gemini에게 전달** - "door detected at (x,y)"
3. **Gemini가 전략 수립** - "move towards the door"

## 🔧 **구현 단계**

### Step 1: 이미지 데이터 수집

```python
# gemini_bridge.py - GeminiAgentNode

from cv_bridge import CvBridge
import cv2
import base64

class GeminiAgentNode(Node):
    def __init__(self):
        # ...
        self.bridge = CvBridge()
        self.last_image: Optional[Image] = None
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_best_effort
        )
    
    def on_image(self, msg: Image):
        """카메라 이미지 저장"""
        self.last_image = msg
    
    def get_image_bytes(self) -> Optional[bytes]:
        """이미지를 JPEG bytes로 변환"""
        if self.last_image is None:
            return None
        
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
            
            # OpenCV → JPEG bytes
            _, buffer = cv2.imencode('.jpg', cv_image)
            return buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return None
```

### Step 2: Gemini에게 이미지 전달

```python
# agent_loop()에서
obs = self.build_observation()
image_bytes = self.get_image_bytes()

action = self.gemini_client.plan_robot_action_with_vision(
    goal=self.current_task.get("task", ""),
    observation=obs.to_json(),
    image_data=image_bytes,  # ← 이미지 추가
    constraints=self.current_task.get("constraints", {})
)
```

### Step 3: 비용 최적화

```python
# 이미지 분석은 5스텝마다 또는 필요시에만
use_vision = (
    self.step_count % 5 == 0 or  # 주기적
    "찾" in self.current_task.get("task", "") or  # 탐색 태스크
    self.last_action_result.startswith("rotate")  # 회전 후
)

if use_vision:
    image_bytes = self.get_image_bytes()
else:
    image_bytes = None
```

## 📊 **예상 효과**

### Before (센서만):
```
[agent] 전방 2.5m, 좌 1.8m, 우 3.2m
→ 전진 (목표 방향 모름)
```

### After (비전 통합):
```
[agent] 전방 2.5m, 좌 1.8m, 우 3.2m
[vision] 🎥 왼쪽에 우체통 발견, 정면에 문 보임
→ 왼쪽 회전 후 전진 (우체통 향해)
```

## ⚠️ **주의사항**

1. **API 비용**: 이미지 분석은 텍스트보다 비쌈
   - 해결: 주기적 분석 (5-10스텝마다)
   - 해결: 이미지 해상도 축소 (640x480 이하)

2. **레이턴시**: 이미지 전송 + 분석 시간
   - 해결: 비동기 분석
   - 해결: 로컬 객체 탐지 + Gemini 전략 수립

3. **네트워크**: 이미지 크기
   - 해결: JPEG 압축 (quality=50-70)
   - 해결: 이미지 리사이즈

## 🚀 **빠른 테스트**

```python
# gemini_api.py에 추가
def analyze_scene(self, image_bytes: bytes, query: str) -> str:
    """
    이미지 장면 분석
    
    Args:
        image_bytes: JPEG 이미지 데이터
        query: 질문 ("What do you see?")
    
    Returns:
        분석 결과 텍스트
    """
    if not self.is_available():
        return "Vision not available"
    
    try:
        import base64
        image_b64 = base64.b64encode(image_bytes).decode('utf-8')
        
        response = self.client.models.generate_content(
            model=self.config.model_id,
            contents=[
                query,
                {
                    "mime_type": "image/jpeg",
                    "data": image_b64
                }
            ]
        )
        
        return response.text
    except Exception as e:
        print(f"이미지 분석 실패: {e}")
        return f"Error: {e}"
```

## 📝 **다음 단계**

원하시는 방법을 선택해주세요:

1. **Full Vision Integration** - 매 스텝마다 이미지 분석 (비용 높음, 정확도 최고)
2. **Periodic Vision** - 5-10스텝마다 이미지 분석 (비용 중간, 효율적)
3. **On-Demand Vision** - 특정 태스크/상황에서만 (비용 낮음, 선택적)
4. **Hybrid** - 로컬 객체 탐지 + Gemini 전략 (비용 최소, 최적)

어떤 방법을 구현할까요?

