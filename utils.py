import numpy as np
from matplotlib.patches import Rectangle, Wedge, Circle
from scipy.spatial import Delaunay
from typing import Tuple, List, Optional, Set


def pol2cart(rho: float, phi: float) -> np.ndarray:
    """Converte coordenadas polares (raio, ângulo) para cartesianas (x, y)."""
    return np.array([rho * np.cos(phi), rho * np.sin(phi)])


def get_circle_intersection(
    p1: np.ndarray, r1: float, p2: np.ndarray, r2: float, chirality: int = 1
) -> Optional[np.ndarray]:
    """
    Calcula os pontos de interseção entre dois círculos.

    Args:
        p1 (np.ndarray): Centro do círculo 1 [x, y].
        r1 (float): Raio do círculo 1.
        p2 (np.ndarray): Centro do círculo 2 [x, y].
        r2 (float): Raio do círculo 2.
        chirality (int): Escolhe entre as duas soluções possíveis (1 ou -1).

    Returns:
        np.ndarray ou None: Coordenadas do ponto de interseção escolhido.
    """
    d2 = np.sum((p1 - p2) ** 2)
    d = np.sqrt(d2)

    # Verifica se os círculos estão muito distantes, um dentro do outro ou concêntricos
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        return None

    a = (r1**2 - r2**2 + d2) / (2 * d)
    h = np.sqrt(max(0, r1**2 - a**2))

    # Ponto médio da corda de interseção
    x2 = p1[0] + a * (p2[0] - p1[0]) / d
    y2 = p1[1] + a * (p2[1] - p1[1]) / d

    # Diferença das coordenadas dos centros
    dx = p2[1] - p1[1]
    dy = p2[0] - p1[0]

    # Duas soluções possíveis
    x3_1 = x2 + h * dx / d
    y3_1 = y2 - h * dy / d
    x3_2 = x2 - h * dx / d
    y3_2 = y2 + h * dy / d

    return np.array([x3_1, y3_1]) if chirality == 1 else np.array([x3_2, y3_2])


def calculate_concave_hull(
    points: np.ndarray, alpha_threshold: float = 0.04
) -> Tuple[float, List]:
    """
    Calcula a área e o contorno côncavo de uma nuvem de pontos usando Alpha Shapes.

    Args:
        points (np.ndarray): Array Nx2 de pontos.
        alpha_threshold (float): Parâmetro que define o nível de detalhe da concavidade.

    Returns:
        Tuple[float, List]: (área total, lista de segmentos de linha do contorno).
    """
    if len(points) < 4:
        return 0.0, []

    tri = Delaunay(points)
    total_area = 0.0
    boundary_edges: Set[Tuple[int, int]] = set()

    for simplex in tri.simplices:
        pa, pb, pc = points[simplex]

        # Comprimento dos lados do triângulo
        a = np.linalg.norm(pa - pb)
        b = np.linalg.norm(pb - pc)
        c = np.linalg.norm(pc - pa)

        # Área do triângulo
        s = (a + b + c) / 2.0
        tri_area = np.sqrt(max(0, s * (s - a) * (s - b) * (s - c)))

        if tri_area < 1e-5:
            continue

        # Raio do círculo circunscrito
        circum_r = (a * b * c) / (4.0 * tri_area)

        # Filtro alpha: Se o triângulo for pequeno o suficiente, ele faz parte do hull
        if circum_r < (1.0 / alpha_threshold):
            total_area += tri_area

            # Adiciona arestas ao conjunto de borda
            edges = [
                tuple(sorted((simplex[i], simplex[(i + 1) % 3]))) for i in range(3)
            ]

            for edge in edges:
                if edge in boundary_edges:
                    boundary_edges.remove(edge)
                else:
                    boundary_edges.add(edge)

    lines = [[points[i], points[j]] for i, j in boundary_edges]
    return total_area, lines


def draw_servo_patch(
    ax,
    center: np.ndarray,
    current_angle_deg: float,
    offset_rad: float,
    label: str = "",
    color: str = "#444",
    visual_offset_x: Optional[float] = None,
):
    """
    Desenha uma representação visual estilizada de um servo motor.
    """
    offset_deg = np.degrees(offset_rad)
    w_body, h_body = 32.5, 12.0
    r_axis = 2.5

    # Define posição do corpo do servo relativo ao eixo
    if visual_offset_x is not None:
        rect_x = center[0] + visual_offset_x
    else:
        rect_x = center[0] - (w_body - 6.0)

    rect_y = center[1] - (h_body / 2)

    # Corpo do servo
    ax.add_patch(
        Rectangle((rect_x, rect_y), w_body, h_body, color="#222", alpha=0.9, zorder=5)
    )

    ax.add_patch(
        Rectangle(
            (rect_x, rect_y),
            w_body,
            h_body,
            fill=False,
            edgecolor="#666",
            lw=1,
            zorder=6,
        )
    )

    # Eixo central
    ax.add_patch(Circle(center, r_axis, color="white", zorder=8))
    ax.add_patch(Circle(center, r_axis / 2, color="#333", zorder=9))

    # Indicador de range
    ax.add_patch(
        Wedge(
            center, 22, offset_deg, offset_deg + 180, color=color, alpha=0.15, zorder=4
        )
    )

    # Label
    ax.text(
        center[0],
        rect_y - 5,
        label,
        fontsize=8,
        fontweight="bold",
        color="white",
        ha="center",
        va="top",
    )


def draw_link(
    ax,
    p1: np.ndarray,
    p2: np.ndarray,
    color: str = "white",
    lw: float = 3,
    style: str = "-",
    alpha: float = 1.0,
):
    """
    Desenha uma haste (link) conectando dois pontos.
    """
    # Linha do link
    ax.plot(
        [p1[0], p2[0]],
        [p1[1], p2[1]],
        color=color,
        lw=lw,
        linestyle=style,
        solid_capstyle="round",
        zorder=10,
        alpha=alpha,
    )

    # Juntas (círculos nas pontas)
    for p in [p1, p2]:
        ax.plot(
            p[0],
            p[1],
            "o",
            color="#111",
            markeredgecolor=color,
            markersize=6,
            zorder=12,
            alpha=alpha,
        )


def dist_point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """
    Calcula a distância mínima de um ponto P até um segmento de reta AB.
    """
    ab = b - a
    ap = p - a
    len_sq = np.sum(ab**2)

    if len_sq == 0:
        return np.linalg.norm(ap)

    # Projeção escalar normalizada (t)
    t = max(0, min(1, np.dot(ap, ab) / len_sq))

    projection = a + t * ab
    return np.linalg.norm(p - projection)


def ccw(A: np.ndarray, B: np.ndarray, C: np.ndarray) -> bool:
    """Verifica orientação de três pontos (Counter-Clockwise)."""
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def intersect(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray) -> bool:
    """
    Verifica se dois segmentos de reta AB e CD se interceptam.
    """
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def min_dist_segments(
    seg1: Tuple[np.ndarray, np.ndarray], seg2: Tuple[np.ndarray, np.ndarray]
) -> float:
    """
    Calcula a distância mínima entre dois segmentos de reta (seg1 e seg2).
    Se eles se cruzam, a distância é 0.
    Caso contrário, é a menor distância entre os extremos e o outro segmento.
    """
    p1, p2 = seg1
    p3, p4 = seg2

    # Verifica interseção direta
    if intersect(p1, p2, p3, p4):
        return 0.0

    # Verifica distância dos extremos projetados
    d1 = dist_point_segment(p3, p1, p2)
    d2 = dist_point_segment(p4, p1, p2)
    d3 = dist_point_segment(p1, p3, p4)
    d4 = dist_point_segment(p2, p3, p4)

    return min(d1, d2, d3, d4)


def get_scan_limits_at_y(y: float, hull_lines: List) -> Tuple[float, float]:
    """
    Retorna os limites [min_x, max_x] da interseção em Y.
    Retorna (0,0) se X=0 não estiver contido ou acessível.
    """
    intersections = []
    
    # Coleta todas as interseções na altura Y
    for (p1, p2) in hull_lines:
        if (p1[1] <= y <= p2[1]) or (p2[1] <= y <= p1[1]):
            if abs(p2[1] - p1[1]) < 1e-5: continue 
            t = (y - p1[1]) / (p2[1] - p1[1])
            x_inter = p1[0] + t * (p2[0] - p1[0])
            intersections.append(x_inter)
            
    if not intersections: return 0.0, 0.0
    
    # Separa paredes à esquerda e direita
    neg_x = [x for x in intersections if x < -0.1]
    pos_x = [x for x in intersections if x > 0.1]
    
    # Se não houver parede de um dos lados, X=0 está exposto ou fora
    if not neg_x or not pos_x:
        return 0.0, 0.0
        
    # Pega as paredes mais próximas do centro (o "gargalo")
    limit_left = max(neg_x) # Ex: max(-50, -100) = -50
    limit_right = min(pos_x)
    
    return limit_left, limit_right

def calculate_max_centered_ellipse(hull_lines: list, aspect_ratio: float = 0.5) -> tuple:
    """
    Encontra a maior elipse de marcha centralizada em X=0 usando varredura de perfil.
    Garante que a elipse não colida em NENHUM ponto de sua borda.
    """
    if not hull_lines: return 0.0, 0.0, 0.0

    # 1. Cria o Perfil de Largura (Discretização)
    ys = [p[1] for line in hull_lines for p in line]
    if not ys: return 0.0, 0.0, 0.0
    
    min_y, max_y = min(ys), max(ys)
    resolution = 1.0 # Precisão de 1mm
    
    # Array de alturas para escanear
    scan_ys = np.arange(min_y, max_y, resolution)
    n_points = len(scan_ys)
    
    # Para cada Y, qual a largura máxima disponível à esquerda e direita?
    # Armazenamos como raio disponível (distância do centro até a parede mais próxima)
    profile_radius = np.zeros(n_points)
    
    for i, y in enumerate(scan_ys):
        lim_l, lim_r = get_scan_limits_at_y(y, hull_lines)
        # O raio seguro é o menor lado, para garantir simetria em X=0
        if lim_l == 0 and lim_r == 0:
            profile_radius[i] = 0.0
        else:
            profile_radius[i] = min(abs(lim_l), abs(lim_r))

    # 2. Otimização (Brute-force inteligente no Grid 1D)
    best_width = 0.0
    best_y = 0.0
    
    # Iteramos por cada Y possível para ser o CENTRO da elipse
    for i, center_y in enumerate(scan_ys):
        max_possible_r_at_center = profile_radius[i]
        
        # Se o próprio centro está fechado, pula
        if max_possible_r_at_center < 1.0: continue
        
        # Agora, expandimos a elipse a partir deste centro e vemos até onde ela vai
        # A equação da elipse é: (x/W)^2 + (y/H)^2 = 1
        # Largura necessária w(dy) = W_max * sqrt(1 - (dy / (H/2))^2)
        # Onde H = W_max * aspect_ratio
        
        # Tentamos encontrar o maior W_max possível para este centro.
        # Começamos com um chute baseado no espaço do centro e vamos diminuindo se bater nas bordas
        current_w = max_possible_r_at_center * 2.0
        valid_w = 0.0
        
        # Otimização: Busca binária ou iterativa para achar o tamanho
        # Vamos testar tamanhos decrescentes a partir do máximo permitido pelo centro
        # Passo de teste: 2mm
        
        test_widths = np.arange(current_w, 0, -2.0)
        
        for w_cand in test_widths:
            if w_cand <= best_width: break # Se já achamos um melhor antes, pare
            
            h_cand = w_cand * aspect_ratio
            half_h = h_cand / 2.0
            
            # Índices do array que a elipse cobre
            idx_start = int((center_y - half_h - min_y) / resolution)
            idx_end = int((center_y + half_h - min_y) / resolution)
            
            # Verifica limites do array
            if idx_start < 0 or idx_end >= n_points:
                continue # Bateu no teto ou chão global
                
            collision = False
            
            # Verifica colisão em cada fatia horizontal da elipse
            for j in range(idx_start, idx_end + 1):
                y_curr = scan_ys[j]
                dy = abs(y_curr - center_y)
                
                # Largura necessária da elipse nessa altura dy
                # x = (w/2) * sqrt(1 - (dy / (h/2))^2)
                term = 1 - (dy / half_h)**2
                if term < 0: term = 0
                req_half_width = (w_cand / 2.0) * np.sqrt(term)
                
                # Verifica contra o perfil pré-calculado
                if req_half_width > profile_radius[j]:
                    collision = True
                    break
            
            if not collision:
                valid_w = w_cand
                break # Achamos o maior possível para este centro!
        
        if valid_w > best_width:
            best_width = valid_w
            best_y = center_y

    return best_y, best_width, best_width * aspect_ratio