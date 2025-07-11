import os
import streamlit as st
import threading

# 既存コードから機能をインポート
from generate_and_run_python_script import (
    get_chat_response,
    generate_python_script,
    run_python_script,
    PRE_PROMPT,
)

if 'generated_code' not in st.session_state:
    st.session_state['generated_code'] = ''
if 'token_usage' not in st.session_state:
    st.session_state['token_usage'] = None
if 'run_thread' not in st.session_state:
    st.session_state['run_thread'] = None

st.set_page_config(page_title="LLM→ROS2 Code Generator", layout="wide")
st.title("🤖 LLM → ROS2 Code Generator")

# プロンプトの前置き読み込み
prompt_file = os.getenv("PRE_PROMPT", PRE_PROMPT)
try:
    with open(prompt_file, "r") as f:
        pre_prompt = f.read()
except FileNotFoundError:
    st.error(f"PRE_PROMPTファイルが見つかりません: {prompt_file}")
    st.stop()

# 入力エリア
user_input = st.text_area(
    "▶︎ ここにロボットへの指示を日本語で入力してください",
    placeholder="例：「0.2m/sで前に進む」「3秒間右に旋回する」など",
    height=200,
)

# 「コード生成」ボタン
if st.button("🔄 コード生成"):
    full_prompt = pre_prompt + user_input
    with st.spinner("生成中…"):
        try:
            code, usage = get_chat_response(full_prompt)
            st.session_state['generated_code'] = code
            st.session_state['token_usage'] = usage
        except Exception as e:
            st.error(f"コード生成に失敗しました: {e}")
        else:
            # 生成コードをファイルに保存
            generate_python_script(code)

if st.session_state['generated_code']:
    st.subheader("▶︎ 生成された Python コード")
    st.code(st.session_state['generated_code'], language='python')
    usage = st.session_state['token_usage']
    if usage:
        st.markdown(f"**Token Usage:** Prompt: {usage.prompt_tokens}, Completion: {usage.completion_tokens}, Total: {usage.total_tokens}")
    
    # 実行ボタンでバックグラウンド実行
    if st.button("▶ 実行する"):
        st.info("Gazebo 上で動作を確認してください。")
        def target():
            try:
                run_python_script()
            except Exception as e:
                st.error(f"コード実行中にエラーが発生しました: {e}")
            else:
                st.success("コードの実行が完了しました！")
        th = threading.Thread(target=target, daemon=True)
        th.start()
        st.session_state['run_thread'] = th

    # スレッド実行中はスピナーを回す
    if st.session_state['run_thread'] and st.session_state['run_thread'].is_alive():
        with st.spinner("コード実行中…"):
            pass

# アプリ停止ボタン
#if st.button("■ アプリ停止"):
#    st.warning("アプリケーションを停止します")
#    #os._exit(0)
#    st.stop()