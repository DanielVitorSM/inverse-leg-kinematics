import numpy as np
from typing import Dict, Tuple, Optional, Any
from configs.leg_interface import LegModel
from utils import pol2cart, draw_link, draw_servo_patch

class SerialLeg(LegModel):
    """
    Modelo de Perna Serial (2 Graus de Liberdade).
    
    Topologia:
    - Base -> Coxa (L1) -> Joelho -> Canela (L2) -> Pé.
    - O ângulo do joelho é calculado relativamente à coxa (Theta1 - Theta2).
    """

    def __init__(self):
        super().__init__("Serial")
        
        # Definição dos parâmetros padrão [valor_atual, min, max]
        self.parameters = {
            "L1 (S1->Knee)": [50.0, 10.0, 105.0], 
            "L2 (Knee->Foot)": [95.0, 10.0, 105.0]
        }
        
        # Inicializa comprimentos internos
        self.update_params({k: v[0] for k, v in self.parameters.items()})
        
        # Posição da base
        self.base_pos = np.array([0.0, 0.0])
        
        # Distância mínima segura para evitar colisão pé-base
        self.collision_threshold = 10.0

    def update_params(self, params: Dict[str, float]):
        self.l1 = params["L1 (S1->Knee)"]
        self.l2 = params["L2 (Knee->Foot)"]

    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> Tuple[Optional[np.ndarray], Optional[Dict]]:
        theta1 = np.radians(theta1_deg) + self.offset_t1
        theta2 = np.radians(theta2_deg) + self.offset_t2
        
        # Posição do joelho
        point_knee = self.base_pos + pol2cart(self.l1, theta1)
        
        # Posição do pé
        point_foot = point_knee + pol2cart(self.l2, theta1 - theta2) 
        
        return point_foot, {
            "Base": self.base_pos,
            "Knee": point_knee,
            "Foot": point_foot
        }

    def check_collisions(self, joints: Dict) -> bool:
        point_foot = joints["Foot"]
        point_base = joints["Base"]
        
        # Verifica distância euclidiana
        if np.linalg.norm(point_foot - point_base) < self.collision_threshold:
            return True
            
        return False

    def draw(self, ax, joints: Dict, theta1: float, theta2: float, colors: Dict):
        # Desenha servo da base
        draw_servo_patch(
            ax, joints["Base"], theta1, self.offset_t1, "Base", 
            colors['servo1']
        )
        
        # Desenha segmentos
        draw_link(ax, joints["Base"], joints["Knee"], color=colors['servo1'])
        draw_link(ax, joints["Knee"], joints["Foot"], color='white')