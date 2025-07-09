import os
import logging
from dotenv import load_dotenv
import openai
import subprocess
import os
import sys

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")  # OpenAI APIキーの取得
MODEL = os.getenv("OPENAI_MODEL", "o4-mini")  # Model select

# 事前promptをprompt.txtから読み込む
with open("prompt.txt", "r") as f:
    pre_prompt = f.read()

# ChatGPT APIでpromptを入力して返信を受け取る
def get_chat_response(prompt):
    response = openai.chat.completions.create(
        model=MODEL,
        messages=[{"role": "user", "content": prompt}],
    )
    return response.choices[0].message.content.strip()

#返信の内容を.pyファイルに書いて保存する
def generate_python_script(res):
    python_code = res
    with open("generated_script.py", "w") as f:
        f.write(python_code)

# subprocessでpythonスクリプトを実行する
def run_python_script():
    result = subprocess.run(["python3", "generated_script.py"], capture_output=True, text=True)
    #print("stdout:")
    #print(result.stdout)
    #print("stderr:")
    #print(result.stderr)
    #print("Return code:", result.returncode)

def main(prompt):
    res = get_chat_response(prompt)
    print("Generated code:")
    print(res)
    generate_python_script(res)
    run_python_script()

if __name__ == "__main__":
    #prompt を入力する
    print("Please enter additional prompt text:")
    prompt = pre_prompt + input("")
    main(prompt)
