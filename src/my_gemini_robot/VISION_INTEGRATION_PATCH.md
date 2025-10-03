# 비전 통합 패치 가이드

## 📝 **수정해야 할 파일: gemini_bridge.py**

### 1. Import 추가 (파일 상단)

```python
# 기존 imports 뒤에 추가
try:
    from cv_bridge import CvBridge
    import cv2
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    print("Warning: cv_bridge not available. Vision features disabled.")
```

### 2. GeminiAgentNode.__init__ 수정

#### 2-1. 파라미터 추가 (line 227 뒤)
```python
self.declare_parameter('gemini_thinking_budget', 0)
self.declare_parameter('vision_check_interval', 5)  # ← 추가: N스텝마다 비전 체크
```

#### 2-2. 이미지 구독 추가 (line 248 뒤)
```python
self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.on_scan, qos_best_effort)
# ↓ 추가
self.image_sub = self.create_subscription(
    Image, '/camera/image_raw', self.on_image, qos_best_effort)
```

#### 2-3. 관찰 데이터 변수 추가 (line 264 뒤)
```python
self.last_action_result: Optional[str] = None
# ↓ 추가
self.last_image: Optional[Image] = None
if CV_BRIDGE_AVAILABLE:
    self.bridge = CvBridge()
else:
    self.bridge = None
    self.get_logger().warn('cv_bridge 없음. 비전 기능 비활성화.')
```

### 3. 콜백 메서드 추가 (line 344 뒤, on_scan 메서드 뒤)

```python
def on_scan(self, msg: LaserScan) -> None:
    """레이저 스캔 수신 콜백"""
    try:
        self.last_scan = msg
    except Exception as e:
        self.get_logger().error(f'[agent] 스캔 데이터 처리 중 오류: {e}')

# ↓ 추가
def on_image(self, msg: Image) -> None:
    """카메라 이미지 수신 콜백"""
    try:
        self.last_image = msg
    except Exception as e:
        self.get_logger().error(f'[agent] 이미지 데이터 처리 중 오류: {e}')
```

### 4. 이미지 변환 메서드 추가 (build_observation 메서드 앞)

```python
def get_image_bytes(self) -> Optional[bytes]:
    """
    ROS Image 메시지를 JPEG bytes로 변환
    
    Returns:
        JPEG 이미지 bytes 또는 None
    """
    if self.last_image is None or self.bridge is None or not CV_BRIDGE_AVAILABLE:
        return None
    
    try:
        # ROS Image → OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
        
        # 이미지 리사이즈 (비용 절감: 640x480 이하로)
        height, width = cv_image.shape[:2]
        if width > 640 or height > 480:
            scale = min(640 / width, 480 / height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            cv_image = cv2.resize(cv_image, (new_width, new_height))
        
        # OpenCV → JPEG bytes (품질 60으로 압축)
        _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
        return buffer.tobytes()
    except Exception as e:
        self.get_logger().error(f'이미지 변환 실패: {e}')
        return None
```

### 5. plan_one_step 메서드 수정 (line 638 근처)

```python
def plan_one_step(self, goal: str, observation_json: str, constraints: Dict[str, Any]) -> Dict[str, Any]:
    """
    한 스텝의 행동을 계획 (Gemini AI 호출)
    """
    # Gemini API 사용 가능 시 API 호출
    if self.gemini_client and self.gemini_client.is_available():
        try:
            observation_dict = json.loads(observation_json)
            available_actions = ["move", "rotate", "wait", "finish"]
            
            # ↓ 추가: 주기적 비전 체크
            vision_interval = self.get_parameter('vision_check_interval').value
            use_vision = (
                self.step_count % vision_interval == 0 or
                "찾" in goal or "탐색" in goal or "look" in goal.lower()
            )
            
            image_bytes = self.get_image_bytes() if use_vision else None
            
            if image_bytes:
                self.get_logger().info(f'[agent] 📸 비전 활성화 (스텝 {self.step_count})')
            
            # ↓ 수정: image_bytes 파라미터 추가
            action = self.gemini_client.plan_robot_action(
                goal=goal,
                observation=observation_dict,
                constraints=constraints,
                available_actions=available_actions,
                image_bytes=image_bytes  # ← 추가
            )
            
            return action
            
        except Exception as e:
            self.get_logger().error(f'Gemini API 호출 실패: {e}. Fallback 사용.')
    
    # Fallback: 규칙 기반 플래너
    return call_gemini_one_step(goal, observation_json, constraints)
```

## 📦 **의존성 추가**

### package.xml에 추가

```xml
<exec_depend>cv_bridge</exec_depend>
<exec_depend>opencv-python</exec_depend>
```

### requirements.txt에 추가 (이미 있을 수도 있음)

```txt
opencv-python>=4.5.0
```

## 🔧 **config/gemini_params.yaml 수정**

```yaml
/**:
  ros__parameters:
    # ... 기존 설정 ...
    
    # 비전 설정 추가
    vision_check_interval: 5  # 5스텝마다 비전 활성화
```

## 🚀 **빌드 및 테스트**

```bash
cd ~/ros2_ws
colcon build --packages-select my_gemini_robot
source install/setup.bash

# cv_bridge 설치 (필요시)
sudo apt-get install ros-jazzy-cv-bridge

# Python opencv 설치 (필요시)
pip3 install opencv-python

# 실행
export GEMINI_API_KEY="your_key"
export TURTLEBOT3_MODEL=waffle
ros2 run my_gemini_robot gemini_agent

# 목표 전달
ros2 topic pub --once /agent_goal std_msgs/msg/String "{data: '우체통을 찾아서 문으로 가세요'}"
```

## 📊 **예상 동작**

```
[agent] 스텝=1 | 전방 2.5m (비전 없음)
[agent] 스텝=2 | 전방 2.3m (비전 없음)
[agent] 스텝=3 | 전방 2.0m (비전 없음)
[agent] 스텝=4 | 전방 1.8m (비전 없음)
[agent] 스텝=5 | 📸 비전 활성화 (스텝 5)
         🤖 Gemini: "왼쪽에 빨간 우체통 발견!"
         → 왼쪽 회전
[agent] 스텝=6-9 | (비전 없음, 전진)
[agent] 스텝=10 | 📸 비전 활성화 (스텝 10)
          🤖 Gemini: "정면에 문 보임"
          → 문을 향해 전진
```

## ⚙️ **조정 옵션**

### 더 자주 비전 사용 (3스텝마다)
```yaml
vision_check_interval: 3
```

### 특정 키워드에서만 비전 사용
```python
use_vision = (
    "찾" in goal or "탐색" in goal or "search" in goal.lower()
)
```

### 이미지 품질 조정
```python
# 더 높은 품질 (비용↑)
cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])

# 더 낮은 품질 (비용↓)
cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 40])
```

---

**이 패치를 적용하면 로봇이 주기적으로 카메라 이미지를 Gemini에게 전달하여 시각적 판단을 할 수 있습니다!** 🎥🤖

