import numpy as np
from typing import Dict, Tuple, Optional, Any
from configs.leg_interface import LegModel
from utils import pol2cart, draw_link, draw_servo_patch, dist_point_segment

class PantographLeg(LegModel):
    """
    Modelo de Perna Pantográfica Coaxial.
    
    Topologia:
    - O mecanismo utiliza um arranjo coaxial virtual onde tanto a coxa quanto
      a manivela giram em torno do ponto pivô do Servo 2.
    - O Servo 1 impulsiona a coxa (através de um link visual).
    - O Servo 2 impulsiona a manivela inferior.
    - A canela é formada pelo fechamento do paralelogramo.
    """

    def __init__(self):
        super().__init__("Pantógrafo (Coaxial)")
        
        # Definição dos parâmetros padrão [valor_atual, min, max]
        self.parameters = {
            "L_Thigh (S2->Knee)": [95.0, 10.0, 105.0], 
            "L_Crank (S2->Link)": [10.0, 10.0, 55.0],
            "L_Shin (Knee->Foot)": [95.0, 10.0, 95.0]
        }
        
        # Inicializa comprimentos internos
        self.update_params({k: v[0] for k, v in self.parameters.items()})
        
        # Posicionamento dos servos para visualização
        self.servo1_pos = np.array([-15.0, -10.0])  
        self.servo2_pos = np.array([0.0, 0.0])      
        
        self.collision_threshold = 10.0

    def update_params(self, params: Dict[str, float]):
        self.l_thigh = params["L_Thigh (S2->Knee)"]
        self.l_crank = params["L_Crank (S2->Link)"]
        self.l_shin = params["L_Shin (Knee->Foot)"]

    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> Tuple[Optional[np.ndarray], Optional[Dict]]:
        theta1 = np.radians(theta1_deg) + self.offset_t1
        theta2 = np.radians(theta2_deg) + self.offset_t2
        
        # Vetor da coxa (controlado por S1, mas pivotando em S2)
        vec_thigh = pol2cart(self.l_thigh, theta1)
        point_knee = self.servo2_pos + vec_thigh
        
        # Ponto visual de conexão da biela do S1 (apenas estético)
        point_thigh_connect = self.servo2_pos + pol2cart(35.0, theta1)

        # Vetor da manivela (controlado por S2)
        vec_crank = pol2cart(self.l_crank, theta2)
        point_crank = self.servo2_pos + vec_crank
        
        # Ponto de montagem superior (fechamento do paralelogramo)
        # O ponto superior da canela é calculado somando o vetor da coxa à ponta da manivela
        point_mount = point_crank + vec_thigh
        
        # Cálculo do pé
        # Direção da canela é definida pela linha entre o joelho e o ponto de montagem
        vec_shin_dir = point_mount - point_knee
        norm = np.linalg.norm(vec_shin_dir)
        
        if norm == 0: 
            return None, {}
            
        unit_shin = vec_shin_dir / norm
        point_foot = point_knee + (unit_shin * self.l_shin)
        
        return point_foot, {
            "S1": self.servo1_pos, 
            "S2": self.servo2_pos, 
            "Knee": point_knee, 
            "CrankTip": point_crank, 
            "Mount": point_mount, 
            "Foot": point_foot,
            "ThighConnect": point_thigh_connect
        }

    def _hits_servo(self, segment: Tuple[np.ndarray, np.ndarray], servo_pos: np.ndarray) -> bool:
        """Verifica colisão simples com servo."""
        p1, p2 = segment
        if np.linalg.norm(p1 - servo_pos) < 1e-3 or np.linalg.norm(p2 - servo_pos) < 1e-3: 
            return False
        return dist_point_segment(servo_pos, p1, p2) < self.collision_threshold

    def check_collisions(self, joints: Dict) -> bool:
        """
        Verifica colisões e restrições angulares específicas do pantógrafo.
        """
        # Recupera vetores relativos ao pivô central (S2)
        vec_thigh = joints["Knee"] - joints["S2"]      
        vec_crank = joints["CrankTip"] - joints["S2"]  

        # Converte para graus [0, 360] ou [-180, 180]
        ang_s1 = np.degrees(np.arctan2(vec_thigh[1], vec_thigh[0]))
        ang_s2 = np.degrees(np.arctan2(vec_crank[1], vec_crank[0]))

        # Limite inferior da coxa (evita bater no chão ou na estrutura)
        if ang_s1 < 80: 
            return True
            
        # Abertura máxima (evita que a perna se estique demais e trave)
        if ang_s1 - ang_s2 > 170: 
            return True
            
        # Inversão de joelho (a manivela S2 nunca pode ultrapassar a coxa S1)
        # Adiciona uma margem de 10 graus
        if ang_s2 > ang_s1 - 10: 
            return True

        return False

    def draw(self, ax, joints: Dict, theta1: float, theta2: float, colors: Dict):
        # Desenha Servos
        draw_servo_patch(
            ax, joints["S2"], theta2, self.offset_t2, "S2", 
            colors['servo2'], visual_offset_x=-26.5
        )
        draw_servo_patch(
            ax, joints["S1"], theta1, self.offset_t1, "S1", 
            colors['servo1'], visual_offset_x=-11.5
        )
        
        # Link rígido S1 -> coxa
        draw_link(ax, joints["S1"], joints["ThighConnect"], color=colors['servo1'], lw=4, style='-')
        
        # Coxa principal
        draw_link(ax, joints["S2"], joints["Knee"], color=colors['servo1'], lw=4)
        
        # Manivela inferior
        draw_link(ax, joints["S2"], joints["CrankTip"], color=colors['servo2'], lw=4)
        
        # Acoplador traseiro
        draw_link(ax, joints["CrankTip"], joints["Mount"], color='#ddd', style='-', lw=2)
        
        # Canela - Parte inferior (joelho -> pé)
        ax.plot([joints["Knee"][0], joints["Foot"][0]], 
                [joints["Knee"][1], joints["Foot"][1]], 
                color='white', lw=4, solid_capstyle='round', zorder=10)
        
        # Canela - Parte superior (joelho -> mount)
        ax.plot([joints["Knee"][0], joints["Mount"][0]], 
                [joints["Knee"][1], joints["Mount"][1]], 
                color='white', lw=2, zorder=10)
        
        # Juntas (pontos)
        for key in ["Knee", "CrankTip", "Mount", "ThighConnect"]:
            pt = joints[key]
            ax.plot(pt[0], pt[1], 'o', color='#111', markeredgecolor='#666', markersize=5, zorder=12)