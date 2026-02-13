import matplotlib.pyplot as plt
from settings import configure_matplotlib
from dashboard import RobotDashboard

if __name__ == "__main__":
    # Aplica configurações globais (cores e fontes)
    configure_matplotlib()
    
    # Inicia o dashboard
    app = RobotDashboard()
    
    # Mantém a janela aberta
    plt.show()