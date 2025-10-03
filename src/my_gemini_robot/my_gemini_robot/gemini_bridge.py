#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan, Image

try:
    from cv_bridge import CvBridge
    import cv2
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

# Gemini API 통합
from my_gemini_robot.gemini_api import (
    GeminiRoboticsClient,
    GeminiConfig,
    get_gemini_client
)


# ---------- 상수 정의 ----------

class AgentStatus(Enum):
    IDLE = "idle"
    STARTED = "started"
    RUNNING = "running"
    FINISHED = "finished"
    TIMEOUT = "timeout"
    STEP_LIMIT = "step_limit"
    ERROR = "error"


class ActionType(Enum):
    MOVE = "move"
    ROTATE = "rotate"
    WAIT = "wait"
    FINISH = "finish"


# ---------- 공용 유틸 ----------

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class Observation:
    front_range: Optional[float] = None
    left_range: Optional[float] = None
    right_range: Optional[float] = None
    seen: Dict[str, Any] = field(default_factory=dict)
    last_action_result: Optional[str] = None
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> str:
        return json.dumps({
            "front_range": self.front_range,
            "left_range": self.left_range,
            "right_range": self.right_range,
            "seen": self.seen,
            "last_action_result": self.last_action_result,
            "timestamp": self.timestamp
        }, ensure_ascii=False)

    @classmethod
    def from_json(cls, json_str: str) -> 'Observation':
        try:
            data = json.loads(json_str)
            return cls(**data)
        except Exception:
            return cls()


# ---------- 에이전트 노드 (핵심) ----------

class GeminiAgentNode(Node):
    def __init__(self):
        super().__init__('gemini_agent')

        # 파라미터 선언
        self.declare_parameter('timer_period_sec', 0.8)
        self.declare_parameter('safety_stop_distance_m', 0.25)
        self.declare_parameter('max_steps', 300)
        self.declare_parameter('max_runtime_sec', 600)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.declare_parameter('use_gemini_api', True)
        self.declare_parameter('gemini_model_id', 'gemini-robotics-er-1.5-preview')
        self.declare_parameter('gemini_temperature', 0.5)
        self.declare_parameter('gemini_thinking_budget', 0)
        self.declare_parameter('vision_check_interval', 5)

        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # 구독자
        self.task_sub = self.create_subscription(
            String, '/agent_task', self.on_task, qos_reliable)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_best_effort)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_best_effort)

        # ================================================================= #
        # ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 수정된 핵심 부분 1 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ #
        # Gazebo bridge가 TwistStamped를 요구하므로 TwistStamped로 발행
        # QoS는 Gazebo bridge와 맞추기 위해 RELIABLE 사용
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_reliable)
        # ================================================================= #
        
        self.status_pub = self.create_publisher(String, '/agent_status', qos_reliable)

        # 상태 변수
        self.current_task: Optional[Dict[str, Any]] = None
        self.agent_status: AgentStatus = AgentStatus.IDLE
        self.step_count: int = 0
        self.started_at: Optional[float] = None

        # 관찰 데이터
        self.last_scan: Optional[LaserScan] = None
        self.last_action_result: Optional[str] = None
        self.last_image: Optional[Image] = None
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None

        # Gemini API 클라이언트 초기화
        use_api = self.get_parameter('use_gemini_api').value
        if use_api:
            api_key = os.getenv('GEMINI_API_KEY')
            config = GeminiConfig(
                model_id=self.get_parameter('gemini_model_id').value,
                api_key=api_key,
                temperature=self.get_parameter('gemini_temperature').value,
                thinking_budget=self.get_parameter('gemini_thinking_budget').value
            )
            self.gemini_client = get_gemini_client(config)
            
            if self.gemini_client.is_available():
                self.get_logger().info('Gemini API 연동 성공!')
            else:
                self.get_logger().warn('Gemini API 사용 불가. Fallback 사용.')
        else:
            self.gemini_client = None
            self.get_logger().info('Gemini API 비활성화. Fallback 사용.')

        timer_period = self.get_parameter('timer_period_sec').value
        self.timer = self.create_timer(timer_period, self.agent_loop)
        
        self.get_logger().info('GeminiAgentNode 시작됨.')
        self._publish_status(AgentStatus.IDLE)

    def on_task(self, msg: String) -> None:
        try:
            # JSON 파싱 시도
            try:
                self.current_task = json.loads(msg.data)
            except json.JSONDecodeError:
                # 단순 문자열인 경우 task로 래핑
                self.get_logger().warn(f'[agent] JSON 파싱 실패. 단순 문자열로 처리: {msg.data}')
                self.current_task = {
                    "task": msg.data,
                    "constraints": {"safety_distance_m": 0.35}
                }
            
            self._start_task()
            self.get_logger().info(f'[agent] 태스크 수신: {self.current_task.get("task")}')
        except Exception as e:
            self.get_logger().error(f'[agent] 태스크 처리 중 오류: {e}')

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def on_image(self, msg: Image) -> None:
        self.last_image = msg

    def _start_task(self) -> None:
        self.agent_status = AgentStatus.STARTED
        self.step_count = 0
        self.started_at = time.time()
        self.last_action_result = None
        self._publish_status(AgentStatus.STARTED)
        self.get_logger().info('[agent] 태스크 시작')

    def _publish_status(self, status: AgentStatus, extra: Optional[Dict[str, Any]] = None) -> None:
        payload = {
            "status": status.value,
            "goal": self.current_task.get("task") if self.current_task else None,
            "step": self.step_count,
            "timestamp": time.time()
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _is_running(self) -> bool:
        return self.agent_status in [AgentStatus.STARTED, AgentStatus.RUNNING]

    def get_image_bytes(self) -> Optional[bytes]:
        if self.last_image is None or self.bridge is None:
            return None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
            scale = min(640 / cv_image.shape[1], 480 / cv_image.shape[0])
            if scale < 1.0:
                new_size = (int(cv_image.shape[1] * scale), int(cv_image.shape[0] * scale))
                cv_image = cv2.resize(cv_image, new_size)
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return None

    def build_observation(self) -> Observation:
        front = left = right = None
        if self.last_scan:
            ranges = [r for r in self.last_scan.ranges if self.last_scan.range_min < r < self.last_scan.range_max]
            if ranges:
                num_ranges = len(self.last_scan.ranges)
                center_idx = num_ranges // 2
                side_offset = num_ranges // 4

                front_slice = self.last_scan.ranges[center_idx - 10 : center_idx + 10]
                # LaserScan은 반시계방향(CCW)이므로 인덱스 매핑 수정:
                # center_idx - side_offset = 왼쪽
                # center_idx + side_offset = 오른쪽
                left_slice = self.last_scan.ranges[center_idx - side_offset - 10 : center_idx - side_offset + 10]
                right_slice = self.last_scan.ranges[center_idx + side_offset - 10 : center_idx + side_offset + 10]

                front = min([r for r in front_slice if math.isfinite(r)], default=None)
                left = min([r for r in left_slice if math.isfinite(r)], default=None)
                right = min([r for r in right_slice if math.isfinite(r)], default=None)

        return Observation(
            front_range=front, left_range=left, right_range=right,
            last_action_result=self.last_action_result
        )

    def execute(self, action: Dict[str, Any]) -> str:
        try:
            fn = action.get("function")
            args = action.get("args", {}) or {}

            if fn == ActionType.MOVE.value:
                self._move(
                    str(args.get("direction", "forward")),
                    float(args.get("speed", 0.2)),
                    float(args.get("duration", 1.0))
                )
                return f"move({args.get('direction')},{args.get('speed')},{args.get('duration')})"
            elif fn == ActionType.ROTATE.value:
                self._rotate(
                    str(args.get("direction", "left")),
                    float(args.get("angular_velocity", 0.5)),
                    float(args.get("duration", 1.0))
                )
                return f"rotate({args.get('direction')},{args.get('angular_velocity')},{args.get('duration')})"
            elif fn == ActionType.WAIT.value:
                duration = float(args.get("duration", 0.5))
                self._stop()
                time.sleep(duration)
                return f"wait({duration})"
            elif fn == ActionType.FINISH.value:
                reason = str(args.get("reason", "Goal achieved"))
                self._stop()
                self.agent_status = AgentStatus.FINISHED
                self._publish_status(AgentStatus.FINISHED, {"reason": reason})
                self.get_logger().info(f'[agent] 완료: {reason}')
                return f"finish({reason})"
            else:
                self.get_logger().warn(f'[agent] 알 수 없는 행동: {action}')
                self._stop()
                return "unknown_action"
        except Exception as e:
            self.get_logger().error(f'[agent] 행동 실행 중 오류: {e}')
            self._stop()
            return f"error: {e}"

    # ================================================================= #
    # ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 수정된 핵심 함수 2, 3, 4 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ #
    # Gazebo bridge가 요구하는 TwistStamped 메시지를 사용합니다.
    
    def _move(self, direction: str, speed: float, duration: float) -> None:
        """로봇 이동 - TwistStamped 메시지와 time.sleep 사용"""
        max_speed = self.get_parameter('max_linear_speed').value
        lin_speed = clamp(speed, 0.0, max_speed)
        duration = clamp(duration, 0.1, 5.0)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if direction == 'forward':
            msg.twist.linear.x = lin_speed
        elif direction == 'backward':
            msg.twist.linear.x = -lin_speed

        self.get_logger().info(f"Executing: move '{direction}' for {duration:.2f}s.")
        self.cmd_pub.publish(msg)
        time.sleep(duration)
        self._stop()

    def _rotate(self, direction: str, angular_velocity: float, duration: float) -> None:
        """로봇 회전 - TwistStamped 메시지와 time.sleep 사용"""
        max_ang_speed = self.get_parameter('max_angular_speed').value
        ang_speed = clamp(angular_velocity, 0.0, max_ang_speed)
        duration = clamp(duration, 0.1, 5.0)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if direction == 'left':
            msg.twist.angular.z = ang_speed
        elif direction == 'right':
            msg.twist.angular.z = -ang_speed
        
        self.get_logger().info(f"Executing: rotate '{direction}' for {duration:.2f}s.")
        self.cmd_pub.publish(msg)
        time.sleep(duration)
        self._stop()

    def _stop(self) -> None:
        """로봇 정지 - TwistStamped 메시지 사용"""
        self.get_logger().info("Action finished. Sending stop command.")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # twist 필드는 자동으로 0.0으로 초기화됨
        self.cmd_pub.publish(msg)
    
    # ================================================================= #
    
    def agent_loop(self) -> None:
        if not self._is_running():
            return

        try:
            if self.agent_status == AgentStatus.STARTED:
                self.agent_status = AgentStatus.RUNNING

            max_runtime = self.get_parameter('max_runtime_sec').value
            max_steps = self.get_parameter('max_steps').value
            
            if (time.time() - self.started_at) > max_runtime:
                self.get_logger().warn(f'[agent] 최대 실행시간 초과.')
                self.agent_status = AgentStatus.TIMEOUT
                self._stop()
                self._publish_status(AgentStatus.TIMEOUT)
                return
                
            if self.step_count >= max_steps:
                self.get_logger().warn(f'[agent] 최대 스텝 수 초과.')
                self.agent_status = AgentStatus.STEP_LIMIT
                self._stop()
                self._publish_status(AgentStatus.STEP_LIMIT)
                return

            obs = self.build_observation()
            
            action = self.plan_one_step(
                goal=self.current_task.get("task", ""),
                observation=obs,
                constraints=self.current_task.get("constraints", {})
            )

            if not isinstance(action, dict) or "function" not in action:
                # str()로 감싸서 f-string 포맷팅 오류 방지
                self.get_logger().warn(f'[agent] 잘못된 행동 계획. 대기로 대체: {str(action)}')
                action = {"function": "wait", "args": {"duration": 0.5}}

            result = self.execute(action)
            self.last_action_result = result
            self.step_count += 1

            self.get_logger().info(
                f'[agent] 스텝={self.step_count} | 행동={action.get("function")} | 결과={result}'
            )
            self._publish_status(
                AgentStatus.RUNNING, 
                {"action": action, "result": result}
            )
        except Exception as e:
            self.get_logger().error(f'[agent] 루프 실행 중 오류: {e}')
            self.agent_status = AgentStatus.ERROR
            self._stop()
            self._publish_status(AgentStatus.ERROR, {"error": str(e)})

    def plan_one_step(self, goal: str, observation: Observation, constraints: Dict[str, Any]) -> Dict[str, Any]:
        if self.gemini_client and self.gemini_client.is_available():
            try:
                vision_interval = self.get_parameter('vision_check_interval').value
                use_vision = (self.step_count % vision_interval == 0)
                image_bytes = self.get_image_bytes() if use_vision else None

                if image_bytes:
                    self.get_logger().info(f'[agent] 📸 비전 활성화 (스텝 {self.step_count})')
                
                return self.gemini_client.plan_robot_action(
                    goal=goal,
                    observation=observation.__dict__,
                    constraints=constraints,
                    available_actions=[e.value for e in ActionType],
                    image_bytes=image_bytes
                )
            except Exception as e:
                self.get_logger().error(f'Gemini API 호출 실패: {e}. Fallback 사용.')
        
        return self._fallback_planner(observation, constraints)

    def _fallback_planner(self, observation: Observation, constraints: Dict[str, Any]) -> Dict[str, Any]:
        front = observation.front_range
        safety_dist = constraints.get("safety_distance_m", 0.35)
        if front is not None and front < safety_dist:
            return {"function": "rotate", "args": {"direction": "right", "angular_velocity": 0.8, "duration": 1.0}}
        return {"function": "move", "args": {"direction": "forward", "speed": 0.2, "duration": 1.5}}


def main_agent() -> None:
    rclpy.init()
    node = None
    try:
        node = GeminiAgentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.get_logger().info("Shutting down, sending final stop command.")
                node._stop()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main_agent()