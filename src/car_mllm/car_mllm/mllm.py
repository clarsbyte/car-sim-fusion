import ollama

prompt = """
"""

def process_image_with_text(image_path, prompt):
    response = ollama.chat(
        model='gemma3',  #mistral3:14b
        messages=[{
            'role': 'user',
            'content': prompt,
            'images': [image_path]  
        }]
    )
    return response['message']['content']
