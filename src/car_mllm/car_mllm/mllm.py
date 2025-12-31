import ollama
from pydantic import BaseModel, Field

class ActionBook(BaseModel):
    velocity_x: float = Field(..., description="Linear velocity of car (m/s)")
    angular_z: float = Field(..., description="Angular velocity around the z axis (rad/s)")

SYSTEM_PROMPT = """You are an autonomous driving AI controlling a differential drive robot via a front-facing camera.

## Controls
- velocity_x: Forward (+) / Backward (-) speed in m/s. Range: -1.0 to 2.0
- angular_z: Anti-clockwise/left (+) / Clockwise/right (-) in rad/s. Range: -1.5 to 1.5

## RULES:
1. NEVER STOP moving towards the target. Always keep velocity_x > 0.
2. ONLY stop (velocity_x = 0) if there is an obstacle blocking your path that is NOT the target.

## Navigation:
- Target on LEFT → velocity_x = 0.8, angular_z = 0.5
- Target on RIGHT → velocity_x = 0.8, angular_z = -0.5
- Target in CENTER → velocity_x = 1.0, angular_z = 0.0
- Target NOT VISIBLE → velocity_x = 0.0, angular_z = 0.6 (rotate to search)
- Obstacle blocking path (not target) → velocity_x = 0.0, turn away
"""

def process_image_with_text(image_base64: str, target: str) -> ActionBook:
    user_prompt = f"TARGET: {target}\n\nAnalyze the image and output the appropriate velocity commands to navigate toward the target."

    response = ollama.chat(
        model='ministral-3:14b',
        messages=[
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': user_prompt, 'images': [image_base64]}
        ],
        format=ActionBook.model_json_schema()
    )
    return ActionBook.model_validate_json(response['message']['content'])
