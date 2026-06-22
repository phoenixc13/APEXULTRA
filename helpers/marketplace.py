import streamlit as st

FRAMEWORKS = [
    {"id": "nav2", "name": "nav2", "desc": "Navegação autônoma ROS2", "compat": "ROS2", "version": "1.0.0"},
    {"id": "moveit2", "name": "moveit2", "desc": "Planejamento de movimento ROS2", "compat": "ROS2", "version": "2.2.0"},
    {"id": "micro-ros", "name": "micro-ROS", "desc": "ROS2 para microcontroladores", "compat": "ROS2", "version": "0.6.0"},
    {"id": "opencv-bridge", "name": "OpenCV Bridge", "desc": "Visão computacional ROS", "compat": "ROS", "version": "0.1.0"},
    {"id": "slam-toolbox", "name": "SLAM Toolbox", "desc": "Mapeamento simultâneo ROS2", "compat": "ROS2", "version": "0.8.0"},
    {"id": "ros2_control", "name": "ros2_control", "desc": "Controle de hardware ROS2", "compat": "ROS2", "version": "1.1.0"},
    {"id": "behaviortree", "name": "Behaviour Trees", "desc": "BehaviorTree.CPP para lógica de comportamento", "compat": "ROS2", "version": "3.0.0"},
    {"id": "apex-vision", "name": "APEX Vision", "desc": "Visão própria (em breve)", "compat": "ROS", "version": "0.0.1"},
]


def list_frameworks(filter_by="Todos"):
    if filter_by == "Todos":
        return FRAMEWORKS
    return [f for f in FRAMEWORKS if f["compat"] == filter_by]


def render_marketplace():
    st.write("Explore frameworks e instale rapidamente.")
    filter_by = st.selectbox("Filtrar por", ["Todos", "ROS", "ROS2", "Arduino"])
    fw = list_frameworks(filter_by)
    cols = st.columns(2)
    for i, item in enumerate(fw):
        c = cols[i % 2]
        with c:
            st.markdown(f"**{item['name']}** — {item['desc']}")
            st.caption(f"Compatibilidade: {item['compat']} — Versão: {item['version']}")
            if st.button(f"Instalar {item['name']}", key=f"install_{item['id']}"):
                cmd = generate_install_command(item)
                st.code(cmd, language="bash")


def generate_install_command(item):
    # show a realistic command depending on compatibility
    if item["compat"] in ("ROS", "ROS2"):
        return f"# ROS/ROS2 package install\nsudo apt install ros-{item['compat'].lower()}-{item['id']} || pip install {item['id']}"
    if item["compat"] == "Arduino":
        return f"# Arduino library install\n# Use Arduino IDE / PlatformIO to install {item['name']}"
    return f"pip install {item['id']}"
