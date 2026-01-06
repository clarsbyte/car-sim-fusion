import ollama
from pydantic import BaseModel, Field
import sqlite3

class ActionBook(BaseModel):
    velocity_x: float = Field(..., description="Linear velocity of car (m/s)")
    angular_z: float = Field(..., description="Angular velocity around the z axis (rad/s)")
    reasoning: str = Field(..., description="What you see in the image and why you chose this action. Example: 'Orange chair visible on left side of image, turning left to center it'")

SYSTEM_PROMPT = """You control a robot with a camera. Your goal is to navigate toward the target object.

## MANDATORY: ALWAYS describe what you see in the current image!
Your reasoning MUST start by describing visible objects in the current frame.

## Navigation Logic

### If target is VISIBLE in current image:
1. **TARGET CENTERED (middle 20% of image)**:
   - velocity_x = 1.0, angular_z = 0.0
   - Reasoning: "I see [target] centered in view, moving straight toward it"

2. **TARGET on LEFT side**:
   - velocity_x = 0.7, angular_z = 0.4
   - Reasoning: "I see [target] on the left side, turning left to center it"

3. **TARGET on RIGHT side**:
   - velocity_x = 0.7, angular_z = -0.4
   - Reasoning: "I see [target] on the right side, turning right to center it"

4. **TARGET VERY CLOSE (fills >50% of view)**:
   - velocity_x = 0.3, angular_z = 0.0
   - Reasoning: "I see [target] very close/large in view, slowing down"

### If target is NOT VISIBLE in current image:
FIRST: Describe what you DO see (walls, furniture, floor, etc.)
THEN: Use recent history ONLY if it shows target was recently visible with clear direction

**Strategy when target not visible:**
1. **If recent memory shows target on LEFT (angular_z > 0)**: Continue turning LEFT
   - Reasoning: "I see [wall/objects], target was recently on left so continuing left turn"

2. **If recent memory shows target on RIGHT (angular_z < 0)**: Continue turning RIGHT
   - Reasoning: "I see [wall/objects], target was recently on right so continuing right turn"

3. **If no useful recent history OR just been spinning**: Pick direction based on what's visible
   - Reasoning: "I see [describe scene], target not visible, rotating to search"
   - velocity_x = 0.0, angular_z = 0.5

## CRITICAL RULES
1. ALWAYS describe current image first in reasoning
2. NEVER say "use recent history" without describing what you see NOW
3. Trust your eyes - current image is ground truth
4. Move forward (velocity_x > 0) when target is visible
5. History is only a hint for direction when target disappears
"""

con = sqlite3.connect("../memory.db")
cur = con.cursor()

def process_image_with_text(image_base64: str, target: str, memory_id: int = None) -> ActionBook:
    memory_context = ""
    if memory_id is not None:
        # Fetch last 3 memories to provide context without noise
        memory_context = fetch_memory(recent_n=3)

    user_prompt = f"TARGET: {target}\n\n"
    user_prompt += "STEP 1: Describe what you see in the current image (objects, walls, furniture, etc.)\n"
    user_prompt += "STEP 2: Is the target visible? If YES, navigate toward it. If NO, use recent history as a hint.\n"

    if memory_context:
        user_prompt += f"\nRecent history (reference only):\n{memory_context}\n"

    user_prompt += "\nYour reasoning MUST describe the current scene. Never just say 'use history'!"

    response = ollama.chat(
        model='qwen2.5vl:7b',
        messages=[
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': user_prompt, 'images': [image_base64]}
        ],
        format=ActionBook.model_json_schema()
    )

    action = ActionBook.model_validate_json(response['message']['content'])

    movement = f"velocity_x={action.velocity_x}, angular_z={action.angular_z}"
    save_memory(prompt=target, movement=movement, reasoning=action.reasoning)

    return action

def get_last_memory_id() -> int:
    """Get the ID of the most recently inserted memory."""
    cur.execute("SELECT id FROM memories ORDER BY id DESC LIMIT 1")
    result = cur.fetchone()
    return result[0] if result else None

def clear_all_memories():
    """Clear all memories from the database and reset the ID counter."""
    cur.execute("""
        CREATE TABLE IF NOT EXISTS memories (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            prompt TEXT,
            movement TEXT,
            reason TEXT
        )
    """)
    cur.execute("DELETE FROM memories")
    # Reset the auto-increment counter
    cur.execute("DELETE FROM sqlite_sequence WHERE name='memories'")
    con.commit()
    print("[MEMORY CLEARED] All memories deleted and ID counter reset")

def save_memory(prompt: str, movement: str, reasoning: str) -> str:
    """Save a memory of a robot movement decision to the database."""
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

    print(f"[MEMORY SAVED] Prompt: {prompt} | Movement: {movement} | Reasoning: {reasoning}")

    return "Memory saved successfully"

def fetch_memory(id: int = None, user_prompt: str = None, recent_n: int = None) -> str:
    """Fetch memories from the database by ID, prompt search, or recent N entries."""
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
    elif recent_n is not None:
        cur.execute("SELECT * FROM memories ORDER BY id DESC LIMIT ?", (recent_n,))
        results = cur.fetchall()
        if results:
            # Reverse to show oldest first (chronological order)
            results = list(reversed(results))
            return "\n".join([f"ID: {r[0]}, Movement: {r[2]}, Reason: {r[3]}" for r in results])
        else:
            return "No recent memories found"
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
