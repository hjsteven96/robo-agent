#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gemini API 연동 모듈
"""

import os
import json
import re
from typing import Any, Dict, List, Optional
from dataclasses import dataclass

# Gemini API 임포트 (graceful fallback)
try:
    from google import genai
    from google.genai import types
    GENAI_AVAILABLE = True
except ImportError:
    genai = None
    types = None
    GENAI_AVAILABLE = False
    print("Warning: google-genai 패키지가 설치되지 않았습니다. 설치: pip install google-genai")


@dataclass
class GeminiConfig:
    """Gemini API 설정"""
    api_key: str
    model_id: str = "gemini-robotics-er-1.5-preview"
    temperature: float = 0.7
    thinking_budget: int = 0  # 추론 예산 (0=빠름, 높은값=정확)


class GeminiRoboticsClient:
    """
    Gemini Robotics-ER 1.5 API 클라이언트
    
    Function Calling을 통해 로봇 행동을 계획하고 실행합니다.
    """
    
    def __init__(self, config: GeminiConfig):
        """
        Args:
            config: Gemini API 설정
        """
        self.config = config
        self.client = None
        
        if not GENAI_AVAILABLE:
            print("경고: Gemini API를 사용할 수 없습니다. 데모 모드로 작동합니다.")
            return
        
        try:
            # Client 초기화
            self.client = genai.Client(api_key=config.api_key)
            print(f"✅ Gemini API 초기화 완료 (모델: {config.model_id})")
        except Exception as e:
            print(f"경고: Gemini API 초기화 실패: {e}")
            self.client = None
    
    def is_available(self) -> bool:
        """API 사용 가능 여부 확인"""
        return self.client is not None and GENAI_AVAILABLE
    
    def plan_robot_action(
        self,
        goal: str,
        observation: Dict[str, Any],
        constraints: Dict[str, Any],
        available_actions: List[str],
        image_bytes: Optional[bytes] = None
    ) -> Dict[str, Any]:
        """
        로봇의 다음 행동을 계획합니다.
        
        Args:
            goal: 목표 설명
            observation: 현재 관찰 데이터 (센서 정보 등)
            constraints: 제약 조건 (안전 거리, 최대 속도 등)
            available_actions: 사용 가능한 행동 목록
            image_bytes: 카메라 이미지 (선택, JPEG bytes)
            
        Returns:
            행동 딕셔너리: {"function": "action_name", "args": {...}}
        """
        if not self.is_available():
            return self._fallback_planner(goal, observation, constraints)
        
        try:
            # 행동 함수 정의
            function_declarations = self._create_action_functions(available_actions)
            
            # 비전 정보 추가
            vision_info = ""
            if image_bytes:
                vision_info = "\n🎥 CAMERA IMAGE PROVIDED - Analyze visual information to make better decisions!"
            
            # 프롬프트 생성
            prompt = f"""
          You are an intelligent and persistent autonomous robot. Your primary mission is to achieve the goal: "{goal}". Use your advanced visual understanding and planning capabilities to succeed.

### 1. Current Situation Analysis

**On First Turn Only:**
- If this is the first step (`last_action_result` is 'none'), perform a detailed analysis of the initial image. Describe the environment (e.g., hallway, room), identify key objects, obstacles, and potential paths toward the goal.

**During Execution (Subsequent Turns):**
- **Last Action's Result:** {observation.get('last_action_result', 'none')}
- **Current Sensor Data:**
  - Front Range: {observation.get('front_range', 'unknown')} m
  - Left Range: {observation.get('left_range', 'unknown')} m
  - Right Range: {observation.get('right_range', 'unknown')} m
- **Visual Context:**{vision_info}
  - Briefly describe what you see and how it relates to your current strategy.

### 2. Strategic Planning

**Core Directive:** Make steady, logical progress toward the goal. Do not get stuck or panic.

**Goal Decomposition:**
- Break down the final goal, "{goal}", into logical sub-tasks.
- State the *single most critical sub-task* you are trying to accomplish right now.

**Strategic Context & Memory:**  # <---- 개선점 1: 전략적 끈기 강조
- **Crucial Rule:** If you have determined that following a specific path (e.g., "follow the left wall") is the correct strategy, **DO NOT abandon this strategy** due to expected sensor readings.
- Proximity to a wall you are intentionally following is NOT a critical failure. Only change your core strategy if the path is truly blocked by an unexpected obstacle or a much better path becomes visible.

### 3. Action Rationale

Think step-by-step to decide your next action:

1.  **Reflect:** What was the outcome of my last action? Did it bring me closer to the goal as planned?
2.  **Analyze:** Synthesize all sensor and visual data, but **weigh it against your current strategy.**  # <---- 개선점 2: 센서 데이터의 맥락적 해석
   - Is a close sensor reading an unexpected obstacle, or is it an **expected and necessary part of navigating a narrow space** or following a wall to the goal?
   - Differentiate between a *threat* (an obstacle blocking your path) and a *guide* (a wall you are following).
3.  **Strategize:** What is the single best action to execute *right now* to advance your current sub-task? Be specific (e.g., 'Continue moving forward along the left wall to reach the door at the end').
4.  **Explain:** Clearly justify your chosen action, explaining why it's better than alternatives (like turning away).

### 4. Final Output Format (MANDATORY)

You MUST conclude with a final, clean JSON code block containing your chosen action. This JSON block must be the VERY LAST part of your response.

**Available actions:** {', '.join(available_actions)}

**Environment Context:**
- Robot: TurtleBot3 Waffle (Width: 0.28m)
- Environment: TurtleBot3 House (Corridor width: ~1.0m, Doorway width: ~0.8m) # <---- 개선점 3: 환경 정보 구체화
- **Rule for Narrow Spaces:** It is NORMAL and EXPECTED for side sensors to report close distances (e.g., 0.2m - 0.4m) when navigating corridors or approaching doorways. Do not treat this as a reason to panic and turn away from your goal. Prioritize forward movement along the strategic path.

Now, achieve your goal: "{goal}"
            """
            
            # Contents 구성 (텍스트 + 선택적 이미지)
            parts = []
            parts.append(types.Part(text=prompt))
            
            if image_bytes:
                try:
                    image_part = types.Part(
                        inline_data=types.Blob(
                            mime_type='image/jpeg',
                            data=image_bytes
                        )
                    )
                    parts.append(image_part)
                    print('📸 이미지를 Gemini에게 전달 중...')
                except Exception as e:
                    print(f'⚠️  이미지 데이터 추가 실패: {e}')
            
            # 최종적으로 parts 리스트를 포함하는 Content 객체를 생성
            contents = [types.Content(role='user', parts=parts)]
            
            # GenerateContentConfig 설정 (thinking_config는 제외)
            config_params = {
                'temperature': self.config.temperature,
                'tools': [types.Tool(function_declarations=function_declarations)]
            }
            
            # thinking_config는 일부 모델에서만 지원되므로 조건부로 추가
            # gemini-robotics-er 모델은 thinking을 지원하지 않음
            if self.config.thinking_budget > 0 and 'robotics' not in self.config.model_id.lower():
                config_params['thinking_config'] = types.ThinkingConfig(
                    thinking_budget=self.config.thinking_budget,
                    include_thoughts=True
                )
            
            response = self.client.models.generate_content(
                model=self.config.model_id,
                contents=contents,
                config=types.GenerateContentConfig(**config_params)
            )
            
            # 응답 출력 (디버깅용)
            print(f"\n{'='*80}")
            print(f"🤖 Gemini AI 추론 결과:")
            print(f"{'='*80}")
            
            # 전체 응답 구조 확인
            has_thought = False
            has_text = False
            has_function = False
            
            # 함수 호출 추출
            for part in response.candidates[0].content.parts:
                # Thought (추론 과정)
                if hasattr(part, 'thought') and part.thought:
                    has_thought = True
                    print(f"🧠 추론 과정 (Thought):")
                    print(f"   {part.thought}")
                    print()
                
                # Text (설명)
                if part.text:
                    has_text = True
                    print(f"💭 설명:")
                    print(f"   {part.text}")
                    print()
                
                # Function Call (행동)
                if part.function_call:
                    has_function = True
                    print(f"🎯 선택한 행동: {part.function_call.name}")
                    print(f"📋 파라미터:")
                    for key, value in dict(part.function_call.args).items():
                        print(f"   - {key}: {value}")
            
            print(f"{'='*80}\n")
            
            # 디버깅: 어떤 부분이 있는지 표시
            if not (has_thought or has_text or has_function):
                print("⚠️  응답에 thought, text, function_call이 없습니다.")
                print(f"Response parts: {[type(p).__name__ for p in response.candidates[0].content.parts]}")
            
            # 함수 호출 반환
            for part in response.candidates[0].content.parts:
                if part.function_call:
                    return {
                        "function": part.function_call.name,
                        "args": dict(part.function_call.args)
                    }
            
            # 함수 호출이 없으면 텍스트 응답 파싱
            result_text = response.text.strip()
            if result_text:
                try:
                    # JSON 추출 시도 (마크다운 코드 블록 또는 일반 JSON)
                    json_str = self._extract_json_from_response(result_text)
                    
                    if not json_str:
                        print(f"⚠️  응답에서 JSON을 찾을 수 없습니다.")
                        print(f"원본 응답: {result_text[:300]}...")
                        return {"function": "wait", "args": {"duration": 0.5}}
                    
                    # JSON 파싱
                    parsed = json.loads(json_str)
                    
                    # 형식 변환: {'action': 'X', 'param1': ..., 'param2': ...} 
                    # → {'function': 'X', 'args': {'param1': ..., 'param2': ...}}
                    if 'action' in parsed and 'function' not in parsed:
                        action_name = parsed.pop('action')
                        print(f"🔄 형식 변환: action='{action_name}' → function")
                        return {
                            "function": action_name,
                            "args": parsed  # 나머지 파라미터를 args로
                        }
                    
                    # 이미 올바른 형식이면 그대로 반환
                    if 'function' in parsed:
                        # args가 없으면 빈 딕셔너리로
                        if 'args' not in parsed:
                            parsed['args'] = {}
                        return parsed
                    
                    # function도 action도 없으면 전체를 args로
                    print(f"⚠️  'function' 또는 'action' 키가 없음. 기본 wait 사용.")
                    print(f"파싱된 데이터: {parsed}")
                    
                except json.JSONDecodeError as e:
                    print(f"⚠️  JSON 파싱 실패: {e}")
                    print(f"파싱 시도한 문자열: {json_str[:200]}...")
                except Exception as e:
                    print(f"⚠️  예외 발생: {e}")
                    print(f"원본 텍스트: {result_text[:200]}...")
            
            # 기본 행동
            print("⚠️  유효한 함수 호출을 찾지 못함. 기본 대기 행동 반환.")
            return {"function": "wait", "args": {"duration": 0.5}}
            
        except Exception as e:
            print(f"❌ 행동 계획 오류: {e}")
            import traceback
            traceback.print_exc()
            return self._fallback_planner(goal, observation, constraints)
    
    def analyze_scene(
        self,
        image_bytes: bytes,
        query: str = "What do you see in this image?"
    ) -> str:
        """
        이미지 장면 분석 (독립 메서드)
        
        Args:
            image_bytes: JPEG 이미지 데이터
            query: 질문
        
        Returns:
            분석 결과 텍스트
        """
        if not self.is_available():
            return "Vision API not available"
        
        try:
            # 텍스트와 이미지를 별도의 Part로 구성
            parts = [
                types.Part(text=query),
                types.Part(
                    inline_data=types.Blob(
                        mime_type='image/jpeg',
                        data=image_bytes
                    )
                )
            ]
            
            contents = [types.Content(role='user', parts=parts)]
            
            response = self.client.models.generate_content(
                model=self.config.model_id,
                contents=contents
            )
            
            return response.text
        except Exception as e:
            print(f"❌ 이미지 분석 실패: {e}")
            return f"Error: {e}"
    
    def _extract_json_from_response(self, text: str) -> Optional[str]:
        """
        응답 텍스트에서 JSON 코드 블록 또는 JSON 객체를 추출합니다.
        
        Args:
            text: Gemini API 응답 텍스트
            
        Returns:
            추출된 JSON 문자열 또는 None
        """
        # 1. 마크다운 JSON 코드 블록 찾기 (```json ... ```)
        match = re.search(r"```json\s*([\s\S]+?)\s*```", text, re.IGNORECASE)
        if match:
            return match.group(1).strip()
        
        # 2. 일반 코드 블록 찾기 (``` ... ```)
        match = re.search(r"```\s*([\s\S]+?)\s*```", text)
        if match:
            content = match.group(1).strip()
            # JSON인지 확인
            if content.startswith('{') and content.endswith('}'):
                return content
        
        # 3. 텍스트에서 { ... } 블록 찾기
        if '{' in text and '}' in text:
            json_start = text.find('{')
            json_end = text.rfind('}') + 1
            if json_start < json_end:
                return text[json_start:json_end]
        
        return None
    
    def _create_action_functions(
        self, 
        available_actions: List[str]
    ) -> List[types.FunctionDeclaration]:
        """
        사용 가능한 행동들을 Function Declaration으로 변환
        
        Args:
            available_actions: 행동 이름 리스트
            
        Returns:
            Function Declaration 리스트
        """
        function_defs = []
        
        # move 함수
        if "move" in available_actions:
            function_defs.append(
                types.FunctionDeclaration(
                    name="move",
                    description="Move the robot in a specified direction at a given speed for a duration",
                    parameters={
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["forward", "backward"],
                                "description": "Direction to move"
                            },
                            "speed": {
                                "type": "number",
                                "description": "Linear speed in m/s (0.1 to 1.0)"
                            },
                            "duration": {
                                "type": "number",
                                "description": "Duration in seconds (0.1 to 5.0)"
                            }
                        },
                        "required": ["direction", "speed", "duration"]
                    }
                )
            )
        
        # rotate 함수
        if "rotate" in available_actions:
            function_defs.append(
                types.FunctionDeclaration(
                    name="rotate",
                    description="Rotate the robot in place",
                    parameters={
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["left", "right"],
                                "description": "Direction to rotate"
                            },
                            "angular_velocity": {
                                "type": "number",
                                "description": "Angular velocity in rad/s (0.1 to 2.0)"
                            },
                            "duration": {
                                "type": "number",
                                "description": "Duration in seconds (0.1 to 5.0)"
                            }
                        },
                        "required": ["direction", "angular_velocity", "duration"]
                    }
                )
            )
        
        # wait 함수
        if "wait" in available_actions:
            function_defs.append(
                types.FunctionDeclaration(
                    name="wait",
                    description="Wait in place for a specified duration",
                    parameters={
                        "type": "object",
                        "properties": {
                            "duration": {
                                "type": "number",
                                "description": "Duration to wait in seconds (0.1 to 3.0)"
                            }
                        },
                        "required": ["duration"]
                    }
                )
            )
        
        # finish 함수
        if "finish" in available_actions:
            function_defs.append(
                types.FunctionDeclaration(
                    name="finish",
                    description="Mark the task as complete and stop",
                    parameters={
                        "type": "object",
                        "properties": {
                            "reason": {
                                "type": "string",
                                "description": "Reason for completion"
                            }
                        },
                        "required": []
                    }
                )
            )
        
        return function_defs
    
    def _fallback_planner(
        self,
        goal: str,
        observation: Dict[str, Any],
        constraints: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Gemini API 사용 불가 시 사용하는 간단한 규칙 기반 플래너
        """
        front = observation.get('front_range')
        safety = constraints.get('safety_distance_m', 0.5)
        
        # 안전 체크
        if isinstance(front, (int, float)) and front < safety:
            return {
                "function": "rotate",
                "args": {"direction": "right", "angular_velocity": 0.3, "duration": 1.0}
            }
        
        # 기본 행동: 전진
        return {
            "function": "move",
            "args": {"direction": "forward", "speed": 0.3, "duration": 2.0}
        }


# 전역 클라이언트 인스턴스
_global_client: Optional[GeminiRoboticsClient] = None


def get_gemini_client(
    config: Optional[GeminiConfig] = None
) -> Optional[GeminiRoboticsClient]:
    """
    전역 Gemini 클라이언트 가져오기 (싱글톤 패턴)
    
    Args:
        config: 설정 (처음 호출 시에만 필요)
        
    Returns:
        GeminiRoboticsClient 인스턴스 또는 None
    """
    global _global_client
    
    if _global_client is None and config is not None:
        _global_client = GeminiRoboticsClient(config)
    
    return _global_client