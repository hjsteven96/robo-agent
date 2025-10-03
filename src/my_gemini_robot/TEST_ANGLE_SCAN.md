# 레이저 스캔 각도 기반 섹터링 개선

## 🔧 문제점 (이전)

```python
# 퍼센트 기반 인덱싱 (❌ 틀림)
front = sector_min(int(0.45 * n), int(0.55 * n))  # 45-55%
left  = sector_min(int(0.05 * n), int(0.25 * n))  # 5-25%
right = sector_min(int(0.75 * n), int(0.95 * n))  # 75-95%
```

### 왜 문제인가?

1. **배열 인덱스 ≠ 각도**: ROS LaserScan은 `angle_min`부터 `angle_increment`씩 증가하는 각도 기반
2. **센서 방향 가정**: "배열 가운데 = 정면"이라는 보장이 없음
3. **프레임 차이**: 센서가 `base_link`와 다른 방향을 향할 수 있음
4. **잘못된 거리 측정**: 측면/후방이 정면으로 잘못 분류 → 0.4~0.6m 같은 이상한 값

## ✅ 해결책 (개선)

```python
def sector_min_by_angle(scan: LaserScan, center_deg: float, half_width_deg: float) -> Optional[float]:
    """각도 기반으로 레이저 스캔에서 최소 거리 추출"""
    if scan is None or not scan.ranges:
        return None
    
    a0 = scan.angle_min         # 시작 각도 (rad)
    inc = scan.angle_increment  # 각도 증분 (rad)
    N = len(scan.ranges)
    rng_min, rng_max = scan.range_min, scan.range_max

    # 목표 각도 범위 (rad)
    cen = math.radians(center_deg)
    w = math.radians(half_width_deg)
    lo, hi = cen - w, cen + w

    vals = []
    for i in range(N):
        ang = a0 + i * inc  # 현재 각도 (rad), CCW+
        # -pi..pi로 정규화
        a = math.atan2(math.sin(ang), math.cos(ang))
        if lo <= a <= hi:
            r = scan.ranges[i]
            if (not math.isinf(r)) and (not math.isnan(r)) and (rng_min <= r <= rng_max):
                vals.append(r)
    
    return min(vals) if vals else None

# 사용
front = sector_min_by_angle(self.last_scan, 0.0, 10.0)    # 정면: 0°±10°
left  = sector_min_by_angle(self.last_scan, 90.0, 10.0)   # 좌: +90°±10°
right = sector_min_by_angle(self.last_scan, -90.0, 10.0)  # 우: -90°±10°
```

## 🎯 핵심 개선

| 항목 | 이전 | 개선 |
|------|------|------|
| **방식** | 배열 인덱스 퍼센트 | 실제 각도 (rad) |
| **정면** | 45-55% (추측) | 0°±10° (정확) |
| **왼쪽** | 5-25% (추측) | +90°±10° (정확) |
| **오른쪽** | 75-95% (추측) | -90°±10° (정확) |
| **각도 정규화** | 없음 | -π~π 정규화 |
| **센서 프레임** | 무시 | angle_min/increment 사용 |

## 📊 예상 효과

### 증상 해결:
- ✅ 정확한 정면 거리 측정 (더 이상 0.48m 같은 이상한 값 없음)
- ✅ 좌우 방향 정확도 향상
- ✅ 불필요한 회피 동작 감소
- ✅ Gemini가 올바른 센서 데이터로 판단

### 조정 가능한 파라미터:
```python
# 각도 폭 조정으로 민감도 변경 가능
front = sector_min_by_angle(scan, 0.0, 5.0)   # ±5° (좁음, 민감)
front = sector_min_by_angle(scan, 0.0, 15.0)  # ±15° (넓음, 안정)
```

## 🧪 테스트 방법

1. 빌드 후 실행
2. 로봇을 벽 정면에 배치
3. `/scan` 토픽 확인:
   ```bash
   ros2 topic echo /scan --once
   ```
4. Agent 로그에서 정면 거리 확인:
   ```
   [agent] 현재 관찰: {"front_range": 실제거리, ...}
   ```
5. 값이 실제 환경과 일치하는지 확인

## 🔍 추가 디버깅

### TF 프레임 확인 (필요시):
```bash
ros2 run tf2_tools view_frames
```

만약 LiDAR 프레임이 회전되어 있다면:
```python
# center_deg 값에 오프셋 추가
front = sector_min_by_angle(scan, 0.0 + offset, 10.0)
```

## 📝 참고

- ROS LaserScan: angle_min부터 CCW(반시계) 방향으로 증가
- 0 rad = 정면 (로봇 전방)
- +π/2 rad = 왼쪽
- -π/2 rad = 오른쪽
- angle_increment: 보통 0.01~0.02 rad (약 0.5~1도)

