# -*- coding: utf-8 -*-
"""
Test script for Volcano Engine Ark API
"""
import os
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from llm import VolcanoArkClient, MultimodalRecognizerArk, Config


def test_ark_client():
    """Test Ark API client"""
    print("=== Testing Volcano Engine Ark API ===\n")

    Config.set_key_and_endpoint(
        api_key="d5155a8e-50ea-43b3-9eaf-3abc6eba852a",
        endpoint="https://ark.cn-beijing.volces.com/api/v3/chat/completions",
    )
    Config.set_models(default_model="doubao-seed-2-0-lite-260215")

    # Initialize client with your credentials
    client = VolcanoArkClient(
        api_key=Config.VOLCANO_API_KEY,
        endpoint=Config.VOLCANO_ENDPOINT,
    )

    # Test 1: Simple chat
    print("1. Testing simple chat...")
    messages = [{"role": "user", "content": "你好！请介绍一下火山引擎方舟大模型。"}]

    try:
        response = client.chat_completions(
            model=Config.DEFAULT_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=500,
        )

        if "choices" in response and response["choices"]:
            content = response["choices"][0]["message"]["content"]
            print(f"   Response: {content[:100]}...")
            print("   ✓ Chat test successful")
        else:
            print(f"   Response: {response}")

    except Exception as e:
        print(f"   ✗ Chat test failed: {e}")

    # Test 2: Multimodal recognizer
    print("\n2. Testing multimodal recognizer...")
    recognizer = MultimodalRecognizerArk(
        api_key=Config.VOLCANO_API_KEY,
        endpoint=Config.VOLCANO_ENDPOINT,
    )

    # Test with a simple description
    test_image = "/Users/bytedance/Downloads/IMG_5664.png"
    if os.path.exists(test_image):
        print(f"   Testing with image: {test_image}")

        try:
            # Simple description
            result = recognizer.detect_chess_pieces(
                test_image, model=Config.DEFAULT_MODEL
            )
            if result:
                print(f"   Pieces: {result}")
                print("   ✓ pieces detection successful")
            else:
                print(f"   Result: {result}")

        except Exception as e:
            print(f"   ✗ Image description failed: {e}")
    else:
        print(f"   Image not found: {test_image}")
    # {'pieces': [[5, 1, 'black', 'heijiang'], [5, 5, 'black', 'heibing'], [5, 6, 'red', 'hongbing'], [5, 9, 'red', 'hongshuai']], 'analysis': {'advantage': 'black', 'key_threats': ['红兵位于(5,6)，若黑方未吃掉红兵，下一步红方将吃黑兵形成将帅对面，黑方直接输棋'], 'suggested_moves': [{'piece': 'heibing', 'from': [5, 5], 'to': [5, 6], 'reason': '吃掉红兵后形成将帅对面，黑方直接获胜，是当前必胜走法', 'priority': 1}, {'piece': 'heijiang', 'from': [5, 1], 'to': [5, 2], 'reason': '进将躲开，会让红方吃黑兵获胜，属于劣着', 'priority': 2}, {'piece': 'heijiang', 'from': [5, 1], 'to': [6, 1], 'reason': '移将到六路，仍会让红方吃黑兵获胜，属于劣着', 'priority': 3}, {'piece': 'heijiang', 'from': [5, 1], 'to': [4, 1], 'reason': '移将到四路，仍会让红方吃黑兵获胜，属于劣着', 'priority': 4}]}}
    print("\n=== Test completed ===")


def main():
    """Main function"""
    test_ark_client()


if __name__ == "__main__":
    main()
