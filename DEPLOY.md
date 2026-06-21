# APEX v6 — Deploy no Streamlit Cloud (grátis, 5 minutos)

URL público gerado será tipo: `https://apex-v6-ultra.streamlit.app`

## Pré-requisitos
- Conta no GitHub (grátis) — https://github.com/signup
- Conta no Streamlit Cloud (grátis, login com GitHub) — https://share.streamlit.io

## Passo a passo

### 1. Criar repositório no GitHub
1. Abre https://github.com/new
2. Nome: `apex-v6-ultra`
3. Visibilidade: **Public** (necessário para plano free)
4. **NÃO** inicializar com README (já temos)
5. Clica "Create repository"

### 2. Subir os ficheiros do projeto

No terminal (PowerShell ou Git Bash) na pasta `C:\Users\User\apex_v6`:

```bash
cd C:\Users\User\apex_v6

git init
git add apex_middleware_v6.py apex_engine.py streamlit_app.py requirements.txt packages.txt .streamlit
git commit -m "APEX v6 ULTRA - middleware de tempo real"
git branch -M main
git remote add origin https://github.com/SEU-USER/apex-v6-ultra.git
git push -u origin main
```

(substitui `SEU-USER` pelo teu username do GitHub; vais precisar de autenticar)

**Alternativa sem git:** instala o GitHub Desktop (https://desktop.github.com) e arrasta os ficheiros.

### 3. Deploy no Streamlit Cloud
1. Abre https://share.streamlit.io
2. Login com GitHub
3. Clica "New app"
4. Preenche:
   - **Repository:** `SEU-USER/apex-v6-ultra`
   - **Branch:** `main`
   - **Main file path:** `streamlit_app.py`
   - **App URL:** `apex-v6-ultra` (escolhe o nome — vai virar o subdomínio)
5. Clica "Deploy!"

### 4. Esperar o build (2-5 minutos)
O Streamlit vai:
- Clonar o teu repo
- Instalar dependências do `requirements.txt`
- Compilar a app
- Atribuir URL pública

Vai aparecer no log:
```
Your app is live at: https://apex-v6-ultra.streamlit.app
```

## Limites do plano free Streamlit Cloud

- 1 GB RAM por app (suficiente)
- CPU compartilhada (suficiente para 100Hz simulado)
- Repositório público no GitHub (obrigatório)
- Cold start após inactividade (~30s para reativar)
- Apps ilimitadas

## Partilhar o link

Quando estiver pronto, o link é público — qualquer pessoa pode aceder sem login. Podes:
- Enviar por WhatsApp/email
- Postar no LinkedIn/Twitter
- Incorporar num site com iframe

## Testar localmente antes de deployar

```bash
cd C:\Users\User\apex_v6
py -m pip install -r requirements.txt
py -m streamlit run streamlit_app.py
```

Abre em http://localhost:8501

## Troubleshooting

**Erro "ModuleNotFoundError: No module named 'apex_engine'":**
- Confirma que `apex_engine.py` está na raiz do repo
- Confirma que `streamlit_app.py` tem `import apex_engine` (sem caminho relativo)

**Erro "Port 8501 already in use":**
- Mata o processo ou muda a porta no `.streamlit/config.toml`

**App não atualiza dados em tempo real:**
- O `st.rerun()` força refresh a cada 100ms
- No Streamlit Cloud há throttling agressivo — pode ver latência maior
- Para tempo real puro, usa o frontend desktop PySide6 local

## Próximos passos (opcional)

- Adicionar autenticação com `streamlit-authenticator`
- Migrar para Plotly Dash para mais controlo
- Deploy em Hugging Face Spaces (alternativa com 16GB RAM)
