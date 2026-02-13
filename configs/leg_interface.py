import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Tuple, List, Optional, Any
from utils import calculate_concave_hull

class LegModel(ABC):
    """
    Classe base abstrata para todos os modelos de pernas robóticas.
    Define a interface obrigatória para cinemática, parâmetros e renderização.
    """
    start_s1_angle = 130.0
    start_s2_angle = 90.0

    def __init__(self, name: str):
        self.name = name
        self.parameters: Dict[str, Any] = {}
        
        # Offsets angulares para calibração (em radianos)
        self.offset_t1 = 0.0
        self.offset_t2 = 0.0
        
        # Margem de segurança para detecção de colisão (mm)
        self.collision_threshold = 5.0

    @abstractmethod
    def forward_kinematics(self, theta1: float, theta2: float) -> Tuple[Optional[np.ndarray], Optional[Dict]]:
        """
        Calcula a cinemática direta (posição do pé) baseada nos ângulos dos servos.

        Args:
            theta1 (float): Ângulo do Servo 1 em graus.
            theta2 (float): Ângulo do Servo 2 em graus.

        Returns:
            Tuple: (array[x, y] do efetuador, dict com posições das juntas) ou (None, None) se inválido.
        """
        pass

    @abstractmethod
    def update_params(self, params: Dict[str, float]) -> None:
        """
        Atualiza as dimensões físicas (links) da perna.
        
        Args:
            params (Dict): Dicionário contendo os novos comprimentos.
        """
        pass
    
    @abstractmethod
    def check_collisions(self, joints: Dict) -> bool:
        """
        Verifica se a configuração atual das juntas resulta em colisão física.

        Returns:
            bool: True se houver colisão, False caso contrário.
        """
        pass

    @abstractmethod
    def draw(self, ax, joints: Dict, theta1: float, theta2: float, colors: Dict) -> None:
        """
        Renderiza a estrutura da perna no eixo matplotlib fornecido.
        """
        pass

    def get_workspace(self, resolution: int = 30) -> Tuple[np.ndarray, np.ndarray, float, List]:
        """
        Gera a nuvem de pontos do espaço de trabalho, calcula a área útil e o contorno.

        Args:
            resolution (int): Resolução da varredura angular (pontos por servo).

        Returns:
            Tuple: (arrays X, arrays Y, área calculada, linhas do contorno)
        """
        points = []
        scan_range = np.linspace(0, 180, resolution)
        
        # Varredura de força bruta no espaço de juntas
        for theta1 in scan_range:
            for theta2 in scan_range:
                end_effector, joints = self.forward_kinematics(theta1, theta2)
                
                if end_effector is not None:
                    if not self.check_collisions(joints):
                        points.append(end_effector)
        
        if not points:
            return np.array([]), np.array([]), 0.0, []

        points_array = np.array(points)
        xs, ys = points_array[:, 0], points_array[:, 1]
        
        # Utiliza Alpha Shape (Hull Côncavo) para estimar a área real, ignorando vazios internos
        area, hull_lines = calculate_concave_hull(points_array, alpha_threshold=0.04)

        return xs, ys, area, hull_lines