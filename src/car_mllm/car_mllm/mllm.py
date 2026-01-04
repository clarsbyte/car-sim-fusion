from pydantic import BaseModel, Field
from pydantic_ai import Agent
import json
import sqlite3

class ActionBook(BaseModel):
    velocity_x: float = Field(..., description="Linear velocity of car (m/s)")
    angular_z: float = Field(..., description="Angular velocity around the z axis (rad/s)")

SYSTEM_PROMPT = """You control a robot with a camera. First, determine if you can see the target object in the image.

## MEMORY SYSTEM
You have access to a memory system to learn from previous iterations:
- Use `fetch_memory()` to retrieve past decisions for similar targets or situations
- After making a movement decision, use `save_memory()` to record:
  - prompt: The current target/command
  - movement: The movement decision you made (e.g., "velocity_x=1.0, angular_z=0.0")
  - reasoning: WHY you made that specific movement decision (e.g., "Target centered in view, moving straight")

## Step 1: Can you see the target?
- If NO (target not in image): velocity_x = 0.0, angular_z = 0.5 (STOP and ROTATE to search)
- If YES: proceed to step 2

## Step 2: Where is the target in the image?
- LEFT side of image → velocity_x = 0.5, angular_z = 0.3
- CENTER of image → velocity_x = 1.0, angular_z = 0.0 (go straight!)
- RIGHT side of image → velocity_x = 0.5, angular_z = -0.3

## CRITICAL RULES
- You MUST NOT move forward (velocity_x > 0) unless you can clearly see the target in the image!
- If you cannot find the target object anywhere in the image, set velocity_x = 0 and rotate!
- Only when target is visible AND centered should you drive straight.
- ALWAYS save your decision to memory after determining the movement commands.
"""

con = sqlite3.connect("../memory.db")
cur = con.cursor()

agent = Agent(
    'ollama:llama3.2-vision:11b',
    result_type=ActionBook,
    system_prompt=SYSTEM_PROMPT
)

def process_image_with_text(image_base64: str, target: str, memory_id: int = None) -> ActionBook:
    if memory_id is not None:
        user_prompt = f"TARGET: {target}\n\nPrevious memory ID: {memory_id} - Fetch this first to see what you did before.\n\nFirst: Is the target visible in this image? If NO, output velocity_x=0 and angular_z=0.5 to rotate and search. If YES, output commands to navigate toward it.\n\nRemember to save your decision to memory!"
    else:
        user_prompt = f"TARGET: {target}\n\nFirst: Is the target visible in this image? If NO, output velocity_x=0 and angular_z=0.5 to rotate and search. If YES, output commands to navigate toward it.\n\nRemember to save your decision to memory!"

    result = agent.run_sync(user_prompt, images=[image_base64])
    return result.data

def get_last_memory_id() -> int:
    """Get the ID of the most recently inserted memory."""
    cur.execute("SELECT id FROM memories ORDER BY id DESC LIMIT 1")
    result = cur.fetchone()
    return result[0] if result else None

def clear_all_memories():
    """Clear all memories from the database."""
    cur.execute("DELETE FROM memories")
    con.commit()

@agent.tool
def save_memory(prompt: str, movement: str, reasoning: str) -> str:
    cur.execute("""
        CREATE TABLE IF NOT EXISTS memories (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            prompt TEXT,
            movement TEXT,
            reason TEXT
        )
    """)

    cur.execute(
        "INSERT INTO memories (prompt, movement, reason) VALUES (?, ?, ?)",
        (prompt, movement, reasoning)
    )
    con.commit()
    return "Memory saved successfully"

@agent.tool
def fetch_memory(id: int = None, user_prompt: str = None) -> str:
    cur.execute("""
        CREATE TABLE IF NOT EXISTS memories (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            prompt TEXT,
            movement TEXT,
            reason TEXT
        )
    """)

    if id is not None:
        cur.execute("SELECT * FROM memories WHERE id = ?", (id,))
        result = cur.fetchone()
        if result:
            return f"ID: {result[0]}, Prompt: {result[1]}, Movement: {result[2]}, Reason: {result[3]}"
        else:
            return f"No memory found with ID {id}"
    elif user_prompt is not None:
        cur.execute("SELECT * FROM memories WHERE prompt LIKE ?", (f"%{user_prompt}%",))
        results = cur.fetchall()
        if results:
            return "\n".join([f"ID: {r[0]}, Prompt: {r[1]}, Movement: {r[2]}, Reason: {r[3]}" for r in results])
        else:
            return f"No memories found matching '{user_prompt}'"
    else:
        cur.execute("SELECT * FROM memories ORDER BY id DESC LIMIT 10")
        results = cur.fetchall()
        if results:
            return "\n".join([f"ID: {r[0]}, Prompt: {r[1]}, Movement: {r[2]}, Reason: {r[3]}" for r in results])
        else:
            return "No memories found in database"
