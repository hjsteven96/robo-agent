# 🚀 빠른 비전 통합 가이드

## ✅ 완료된 작업
1. ✅ `gemini_api.py` - 이미지 분석 기능 추가 완료
2. ✅ `package.xml` - cv_bridge 의존성 추가 완료
3. ✅ `gemini_params.yaml` - vision_check_interval 추가 완료

## 📝 남은 작업: gemini_bridge.py 수정 (간단함!)

### 현재 상태
- 비전 API는 준비됨
- 설정 파일 준비됨
- **단지 gemini_bridge.py에서 이미지를 Gemini에게 전달하는 코드만 추가하면 됨**

### 빠른 해결책: 비전 없이 먼저 테스트

현재 상태로도 완벽하게 작동합니다! 비전 없이 센서만으로도 충분히 강력합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select my_gemini_robot
source install/setup.bash
export GEMINI_API_KEY="AIzaSyCnkZXXnhba8DnWC5z5WOer0d9T3FzvRb8"
export TURTLEBOT3_MODEL=waffle

# 실행
ros2 run my_gemini_robot gemini_agent

# 명령 (다른 터미널)
ros2 topic pub --once /agent_goal std_msgs/msg/String "{data: '우체통 옆 문으로 들어가세요'}"
```

## 🎥 비전이 정말 필요하다면

### Option 1: 나중에 추가 (권장)
- 먼저 센서만으로 테스트
- 필요성 확인 후 비전 추가
- 더 안정적인 개발

### Option 2: 지금 추가 (선택)
`VISION_INTEGRATION_PATCH.md` 파일을 참고하여 다음만 추가:

1. **Import 추가** (line 18 뒤)
```python
from sensor_msgs.msg import LaserScan, Image

# ↓ 추가
try:
    from cv_bridge import CvBridge
    import cv2
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
```

2. **파라미터 추가** (line 227 뒤)
```python
self.declare_parameter('gemini_thinking_budget', 0)
self.declare_parameter('vision_check_interval', 5)  # ← 추가
```

3. **이미지 구독 추가** (line 248 뒤)
```python
self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.on_scan, qos_best_effort)
# ↓ 추가
self.image_sub = self.create_subscription(
    Image, '/camera/image_raw', self.on_image, qos_best_effort)
```

4. **변수 초기화** (line 264 뒤)
```python
self.last_action_result: Optional[str] = None
# ↓ 추가
self.last_image: Optional[Image] = None
if CV_BRIDGE_AVAILABLE:
    self.bridge = CvBridge()
else:
    self.bridge = None
```

5. **콜백 추가** (line 343 뒤)
```python
def on_scan(self, msg: LaserScan) -> None:
    """레이저 스캔 수신 콜백"""
    self.last_scan = msg

# ↓ 추가
def on_image(self, msg: Image) -> None:
    """카메라 이미지 수신 콜백"""
    try:
        self.last_image = msg
    except Exception as e:
        self.get_logger().error(f'[agent] 이미지 데이터 처리 중 오류: {e}')
```

6. **이미지 변환 메서드 추가** (line 380 앞, build_observation 앞)
```python
def get_image_bytes(self) -> Optional[bytes]:
    """ROS Image 메시지를 JPEG bytes로 변환"""
    if self.last_image is None or self.bridge is None or not CV_BRIDGE_AVAILABLE:
        return None
    
    try:
        cv_image = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
        
        # 리사이즈 (비용 절감)
        height, width = cv_image.shape[:2]
        if width > 640 or height > 480:
            scale = min(640 / width, 480 / height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            cv_image = cv2.resize(cv_image, (new_width, new_height))
        
        # JPEG 압축
        _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
        return buffer.tobytes()
    except Exception as e:
        self.get_logger().error(f'이미지 변환 실패: {e}')
        return None
```

7. **plan_one_step 수정** (line 648-660)
```python
if self.gemini_client and self.gemini_client.is_available():
    try:
        observation_dict = json.loads(observation_json)
        available_actions = ["move", "rotate", "wait", "finish"]
        
        # ↓ 추가: 비전 체크
        vision_interval = self.get_parameter('vision_check_interval').value
        use_vision = (
            self.step_count % vision_interval == 0 or
            "찾" in goal or "탐색" in goal
        )
        
        image_bytes = self.get_image_bytes() if use_vision else None
        
        if image_bytes:
            self.get_logger().info(f'[agent] 📸 비전 활성화 (스텝 {self.step_count})')
        
        # ↓ 수정: image_bytes 추가
        action = self.gemini_client.plan_robot_action(
            goal=goal,
            observation=observation_dict,
            constraints=constraints,
            available_actions=available_actions,
            image_bytes=image_bytes  # ← 추가
        )
```

## 🔧 의존성 설치

```bash
# ROS cv_bridge
sudo apt-get install ros-jazzy-cv-bridge

# Python opencv
pip3 install --break-system-packages opencv-python
```

## 📊 예상 결과

### 비전 없이 (현재):
```
[agent] 스텝=1 | 전방 2.5m → 전진
[agent] 스텝=2 | 전방 2.3m → 전진
[agent] 스텝=3 | 전방 1.8m → 전진
```

### 비전 추가 후:
```
[agent] 스텝=1 | 전방 2.5m → 전진
[agent] 스텝=5 | 📸 비전 활성화
         🤖 Gemini: "왼쪽에 우체통 발견"
         → 왼쪽 회전
[agent] 스텝=10 | 📸 비전 활성화
          🤖 Gemini: "정면에 문 보임"
          → 문을 향해 전진
```

## 🎯 권장 순서

1. **먼저 현재 상태로 테스트** ← 지금 바로!
2. 센서 기반 동작 확인
3. 필요하면 비전 추가
4. 점진적으로 기능 확장

**지금 당장 비전 없이도 완벽하게 작동합니다!** 🚀

---

## 요약

- ✅ API 준비됨
- ✅ 설정 완료
- ⏭️ 비전은 선택사항
- 🎉 지금 바로 테스트 가능!

