import streamlit as st
import requests
try:
    from supabase import create_client
    _has_supabase = True
except Exception:
    _has_supabase = False


def supabase_client():
    if not _has_supabase:
        return None
    url = st.secrets.get("supabase_url")
    key = st.secrets.get("supabase_key")
    if not url or not key:
        return None
    return create_client(url, key)


def login(email: str, password: str) -> dict:
    """Attempt to authenticate via Supabase, else demo fallback."""
    client = supabase_client()
    if client:
        try:
            res = client.auth.sign_in_with_password({"email": email, "password": password})
            if res and res.get("user"):
                return {"ok": True, "email": email}
        except Exception:
            return {"ok": False, "error": "Supabase error"}

    # Demo fallback
    if email == "demo@apex.ai" and password == "apex2024":
        return {"ok": True, "email": email}

    return {"ok": False, "error": "Credenciais inválidas"}


def logout():
    st.session_state["logged_in"] = False
    st.session_state["user_email"] = None
