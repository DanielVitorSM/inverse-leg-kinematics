import matplotlib.pyplot as plt

# Paleta de cores
THEME = {
    'bg': '#1e1e1e',
    'panel': '#252526',
    'plot': '#1e1e1e',
    'text': '#e0e0e0',
    'subtext': '#888888',
    'accent': '#00bcd4',
    'warn': '#ff5252',
    'servo1': '#00e676',
    'servo2': '#ff4081',
    'widget_bg': '#333333'
}

def configure_matplotlib():
    """Aplica as configurações globais do Matplotlib"""
    plt.style.use('dark_background')
    plt.rcParams['font.family'] = 'monospace'
    plt.rcParams['axes.grid'] = False
    # plt.rcParams['toolbar'] = 'None'