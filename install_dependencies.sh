#!/bin/bash

# Gemini Robot 의존성 설치 스크립트

echo "=== Gemini Robot 의존성 설치 시작 ==="

# Python 패키지 설치
echo "1. Python 패키지 설치 중..."
pip install google-genai Pillow python-dotenv

# 설치 확인
echo ""
echo "2. 설치 확인..."
python3 -c "
try:
    from google import genai
    from google.genai import types
    print('✅ google-genai 설치 완료')
except ImportError as e:
    print('❌ google-genai 설치 실패:', e)

try:
    from PIL import Image
    print('✅ Pillow 설치 완료')
except ImportError:
    print('❌ Pillow 설치 실패')

try:
    import dotenv
    print('✅ python-dotenv 설치 완료')
except ImportError:
    print('❌ python-dotenv 설치 실패')
"

echo ""
echo "=== 설치 완료! ==="
echo ""
echo "다음 단계:"
echo "1. API 키 설정: export GEMINI_API_KEY='your_key'"
echo "2. 빌드: cd ~/ros2_ws && colcon build --packages-select my_gemini_robot"
echo "3. 실행: source install/setup.bash && ros2 run my_gemini_robot gemini_agent"

