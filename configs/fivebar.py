import numpy as np
from typing import Dict, Tuple, Optional, Any
from configs.leg_interface import LegModel
from utils import (
    pol2cart,
    get_circle_intersection,
    draw_link,
    draw_servo_patch,
    min_dist_segments,
    dist_point_segment,
)

class FiveBarRearLeg(LegModel):
    """
    Modelo de Perna Five-Bar (Configuração traseira).
    
    Topologia:
    - Cadeia 1: Servo 1 -> A (L1) -> B (L2)
    - Cadeia 2: Servo 2 -> C (L3) -> B (L4)
    - Pé: Extensão a partir de C, alinhado com o segmento C->B (ou vetor calculado).
    """
    start_s1_angle = 90.0
    start_s2_angle = 130.0

    def __init__(self):
        super().__init__("Five-Bar (Tração Traseira)")
        
        # Definição dos parâmetros padrão [valor_atual, min, max]
        self.parameters = {
            "L1 (S1->A)": [45.0, 10.0, 105.0],
            "L2 (A->B)": [45.0, 10.0, 105.0],
            "L3 (S2->C)": [50.0, 10.0, 105.0],
            "L4 (B->C)": [45.0, 10.0, 105.0],
            "L5 (Ext. C)": [95.0, 10.0, 105.0],
        }
        
        # Inicializa as variáveis internas de comprimento
        self.update_params({k: v[0] for k, v in self.parameters.items()})
        
        # Posições relativas dos servos (mm)
        self.servo1_pos = np.array([-15.0, -10.0])
        self.servo2_pos = np.array([0.0, 0.0])

        # Limite de colisão: Raio do Servo (6mm) + Margem (4mm)
        self.collision_threshold = 10.0

    def update_params(self, params: Dict[str, float]):
        self.l1 = params["L1 (S1->A)"]
        self.l2 = params["L2 (A->B)"]
        self.l3 = params["L3 (S2->C)"]
        self.l4 = params["L4 (B->C)"]
        self.l5 = params["L5 (Ext. C)"]

    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> Tuple[Optional[np.ndarray], Optional[Dict]]:
        # Aplica offsets de calibração
        theta1 = np.radians(theta1_deg) + self.offset_t1
        theta2 = np.radians(theta2_deg) + self.offset_t2

        # Calcula posições dos cotovelos (A e C)
        # S1 controla A, S2 controla C nesta configuração
        point_a = self.servo1_pos + pol2cart(self.l1, theta1)
        point_c = self.servo2_pos + pol2cart(self.l3, theta2)

        # Calcula a interseção (ponto B)
        # B é a interseção do círculo centrado em A (raio L2) e C (raio L4)
        point_b = get_circle_intersection(point_a, self.l2, point_c, self.l4, chirality=-1)

        if point_b is None:
            return None, {}

        # Vetor para o pé (extensão a partir de C na direção C-B invertida ou definida pela geometria)
        vec_cb = point_c - point_b
        norm = np.linalg.norm(vec_cb)
        
        if norm < 1e-6:
            return None, {}

        unit_vector = vec_cb / norm
        point_foot = point_c + unit_vector * self.l5

        return point_foot, {
            "S1": self.servo1_pos,
            "S2": self.servo2_pos,
            "A": point_a,
            "B": point_b,
            "C": point_c,
            "Foot": point_foot,
        }

    def _hits_servo(self, segment: Tuple[np.ndarray, np.ndarray], servo_pos: np.ndarray) -> bool:
        """Verifica se um segmento colide com um servo, ignorando se o segmento nasce nele."""
        p1, p2 = segment
        
        # Ignora se uma das pontas do segmento é o próprio servo
        if np.linalg.norm(p1 - servo_pos) < 1e-3 or np.linalg.norm(p2 - servo_pos) < 1e-3:
            return False
            
        return dist_point_segment(servo_pos, p1, p2) < self.collision_threshold

    def check_collisions(self, joints: Dict) -> bool:
        # Definição dos segmentos
        seg_s1_a = (joints["S1"], joints["A"])
        seg_s2_c = (joints["S2"], joints["C"])
        seg_a_b = (joints["A"], joints["B"])
        seg_b_c = (joints["B"], joints["C"])
        seg_c_foot = (joints["C"], joints["Foot"])

        segments = [seg_s1_a, seg_s2_c, seg_a_b, seg_b_c, seg_c_foot]
        servos = [joints["S1"], joints["S2"]]

        # Colisão entre segmentos e servo
        for seg in segments:
            for servo in servos:
                if self._hits_servo(seg, servo):
                    return True

        # Colisão entre segmentos
        # Verifica cruzamentos críticos que não deveriam acontecer
        if min_dist_segments(seg_s1_a, seg_s2_c) < self.collision_threshold:
            return True
        if min_dist_segments(seg_s1_a, seg_b_c) < self.collision_threshold:
            return True
        if min_dist_segments(seg_s2_c, seg_a_b) < self.collision_threshold:
            return True

        return False

    def draw(self, ax, joints: Dict, theta1: float, theta2: float, colors: Dict):
        # Desenha servos
        draw_servo_patch(
            ax, joints["S2"], theta2, self.offset_t2, "S2",
            colors["servo2"], visual_offset_x=-26.5
        )
        draw_servo_patch(
            ax, joints["S1"], theta1, self.offset_t1, "S1",
            colors["servo1"], visual_offset_x=-11.5
        )

        # Desenha hastes
        draw_link(ax, joints["S1"], joints["A"], color=colors["servo1"])
        draw_link(ax, joints["S2"], joints["C"], color=colors["servo2"])
        draw_link(ax, joints["A"], joints["B"], color="#ccc")
        draw_link(ax, joints["B"], joints["C"], color="#ccc")
        draw_link(ax, joints["C"], joints["Foot"], color="white", lw=4)


class FiveBarFrontLeg(LegModel):
    """
    Modelo de Perna Five-Bar (Configuração dianteira).
    
    Topologia:
    - Cadeia 1: Servo 1 -> A (L1) -> C (L4)
    - Cadeia 2: Servo 2 -> B (L2) -> C (L3)
    - Pé: Extensão a partir da união C.
    """

    def __init__(self):
        super().__init__("Five-Bar (Tração dianteira)")
        
        # Definição dos parâmetros padrão [valor_atual, min, max]
        self.parameters = {
            "L1 (S1->A)": [45.0, 10.0, 105.0],
            "L2 (S2->B)": [45.0, 10.0, 105.0],
            "L3 (B->C)": [50.0, 10.0, 105.0],
            "L4 (A->C)": [45.0, 10.0, 105.0],
            "L5 (Ext. C)": [95.0, 10.0, 105.0],
        }
        
        # Inicializa as variáveis internas de comprimento
        self.update_params({k: v[0] for k, v in self.parameters.items()})
        
        # Posições relativas dos servos (mm)
        self.servo1_pos = np.array([-15.0, -10.0])
        self.servo2_pos = np.array([0.0, 0.0])

        # Limite de colisão: Raio do Servo (6mm) + Margem (4mm)
        self.collision_threshold = 10.0

    def update_params(self, params: Dict[str, float]):
        self.l1 = params["L1 (S1->A)"]
        self.l2 = params["L2 (S2->B)"]
        self.l3 = params["L3 (B->C)"]
        self.l4 = params["L4 (A->C)"]
        self.l5 = params["L5 (Ext. C)"]

    def forward_kinematics(self, theta1_deg: float, theta2_deg: float) -> Tuple[Optional[np.ndarray], Optional[Dict]]:
        theta1 = np.radians(theta1_deg) + self.offset_t1
        theta2 = np.radians(theta2_deg) + self.offset_t2

        # Calcula cotovelos A e B
        point_a = self.servo1_pos + pol2cart(self.l1, theta1)
        point_b = self.servo2_pos + pol2cart(self.l2, theta2)

        # Calcula interseção C
        # C é a interseção do círculo em A (raio L4) e B (raio L3)
        point_c = get_circle_intersection(point_a, self.l4, point_b, self.l3, chirality=-1)

        if point_c is None:
            return None, {}

        # Calcula vetor do pé
        # Direção baseada na geometria C -> A (Haste L4) para extensão
        vec_ca = point_c - point_a
        norm = np.linalg.norm(vec_ca)

        if norm < 1e-6:
            return None, {}

        unit_vector = vec_ca / norm
        point_foot = point_c + unit_vector * self.l5

        return point_foot, {
            "S1": self.servo1_pos,
            "S2": self.servo2_pos,
            "A": point_a,
            "B": point_b,
            "C": point_c,
            "Foot": point_foot,
        }

    def _hits_servo(self, segment: Tuple[np.ndarray, np.ndarray], servo_pos: np.ndarray) -> bool:
        """Verifica colisão com servo (código duplicado propositalmente para manter isolamento)."""
        p1, p2 = segment

        # Ignora se uma das pontas do segmento é o próprio servo
        if np.linalg.norm(p1 - servo_pos) < 1e-3 or np.linalg.norm(p2 - servo_pos) < 1e-3:
            return False

        return dist_point_segment(servo_pos, p1, p2) < self.collision_threshold

    def check_collisions(self, joints: Dict) -> bool:
        # Definição dos segmentos
        seg_s1_a = (joints["S1"], joints["A"])
        seg_s2_b = (joints["S2"], joints["B"])
        seg_a_c = (joints["A"], joints["C"])
        seg_b_c = (joints["B"], joints["C"])
        seg_c_foot = (joints["C"], joints["Foot"])

        segments = [seg_s1_a, seg_s2_b, seg_a_c, seg_b_c, seg_c_foot]
        servos = [joints["S1"], joints["S2"]]

        # Colisão entre segmentos e servo
        for seg in segments:
            for servo in servos:
                if self._hits_servo(seg, servo):
                    return True

        # Colisões entre hastes
        if min_dist_segments(seg_s1_a, seg_s2_b) < self.collision_threshold:
            return True
        if min_dist_segments(seg_s1_a, seg_b_c) < self.collision_threshold:
            return True
        if min_dist_segments(seg_s2_b, seg_a_c) < self.collision_threshold:
            return True

        return False

    def draw(self, ax, joints: Dict, theta1: float, theta2: float, colors: Dict):
        # Desenha servos
        draw_servo_patch(
            ax, joints["S1"], theta1, self.offset_t1, "S1",
            colors["servo1"], visual_offset_x=-11.5
        )
        draw_servo_patch(
            ax, joints["S2"], theta2, self.offset_t2, "S2",
            colors["servo2"], visual_offset_x=-26.5
        )
        
        # Desenha hastes
        draw_link(ax, joints["S1"], joints["A"], color=colors["servo1"])
        draw_link(ax, joints["S2"], joints["B"], color=colors["servo2"])
        draw_link(ax, joints["A"], joints["C"], color="#ccc")
        draw_link(ax, joints["B"], joints["C"], color="#ccc")
        draw_link(ax, joints["C"], joints["Foot"], color="white", lw=4)