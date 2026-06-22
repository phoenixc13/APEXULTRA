import streamlit as st
import requests
import time
try:
    import openai
except Exception:
    openai = None
try:
    import anthropic
except Exception:
    anthropic = None


def ensure_messages():
    if "messages" not in st.session_state:
        st.session_state.messages = [
            {"role": "system", "content": "Você é um assistente para robótica APEX ULTRA."}
        ]


def add_message(role, content):
    ensure_messages()
    st.session_state.messages.append({"role": role, "content": content})


def call_openai(api_key, model, messages):
    if openai is None:
        return "Erro: biblioteca openai não instalada"
    openai.api_key = api_key
    try:
        resp = openai.ChatCompletion.create(model=model, messages=messages)
        return resp["choices"][0]["message"]["content"]
    except Exception as e:
        return f"Erro OpenAI: {e}"


def call_anthropic(api_key, model, messages):
    if anthropic is None:
        return "Erro: biblioteca anthropic não instalada"
    client = anthropic.Client(api_key)
    prompt = "".join([f"{m['role']}: {m['content']}\n" for m in messages])
    try:
        resp = client.completions.create(model=model, prompt=prompt, max_tokens=512)
        return resp.get("completion")
    except Exception as e:
        return f"Erro Anthropic: {e}"


def call_nvidia_nim(api_key, model, messages):
    base = "https://integrate.api.nvidia.com/v1"
    url = f"{base}/models/{model}/completions"
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    prompt = "\n".join([f"{m['role']}: {m['content']}" for m in messages])
    payload = {"input": prompt}
    try:
        r = requests.post(url, headers=headers, json=payload, timeout=30)
        r.raise_for_status()
        j = r.json()
        # try common fields
        return j.get("output", j.get("choices", [{}])[0].get("text", str(j)))
    except Exception as e:
        return f"Erro NVIDIA NIM: {e}"


def send_message(use_apex_ai: bool, provider: str, api_key: str, model: str):
    ensure_messages()
    user_msg = st.session_state.get("chat_input", "")
    if not user_msg:
        return None
    add_message("user", user_msg)

    messages = st.session_state.messages
    # if there's a selected device, add context
    device = st.session_state.get("selected_device")
    if device:
        messages.insert(1, {"role": "system", "content": f"Você controla o robô {device}. Responda comandos de movimento."})

    if use_apex_ai:
        # default NVIDIA NIM model
        key = api_key or "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"
        model_name = model or "meta/llama-3.1-8b-instruct"
        resp = call_nvidia_nim(key, model_name, messages)
    else:
        if provider == "openai":
            resp = call_openai(api_key, model or "gpt-4o-mini", messages)
        else:
            resp = call_anthropic(api_key, model or "claude-2.1", messages)

    add_message("assistant", resp)
    st.session_state.chat_input = ""
    return resp
