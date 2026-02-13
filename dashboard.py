import os
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from typing import Dict, Any, Optional

from settings import THEME
from ui_manager import StackLayout
from utils import calculate_max_centered_ellipse

from configs.serial import SerialLeg
from configs.pantograph import PantographLeg
from configs.fivebar import FiveBarRearLeg, FiveBarFrontLeg


class RobotDashboard:
    """
    Controlador principal da interface gráfica do simulador.
    Gerencia a interação entre modelos matemáticos, sliders e a visualização Matplotlib.
    """

    def __init__(self):
        # Inicialização dos modelos disponíveis
        self.models = {
            "Serial": SerialLeg(),
            "Pantógrafo": PantographLeg(),
            "Five-Bar Traseira": FiveBarRearLeg(),
            "Five-Bar Dianteira": FiveBarFrontLeg(),
        }
        self.model_keys = list(self.models.keys())
        self.current_model = self.models["Serial"]

        # Carrega configurações salvas
        self.optimized_configs = self.load_config_file()

        # Estado inicial
        self.theta1 = 150
        self.theta2 = 45
        self.sim_resolution = 45

        # Gerenciamento de UI dinâmica
        self.dynamic_sliders: Dict[str, Any] = {}
        self.dynamic_elements = []
        self.offset_group = []
        self.button_workspace = None

        # Dados do dorkspace
        self.workspace_x = []
        self.workspace_y = []
        self.workspace_area = 0.0
        self.workspace_poly = None

        # Configuração da janela do matplotlib
        self.fig = plt.figure(figsize=(16, 9), facecolor=THEME["bg"])
        self.fig.canvas.manager.set_window_title("Simulador cinemático")

        self.ax_sim = self.fig.add_axes([0.02, 0.05, 0.65, 0.90])
        self.ax_sim.set_facecolor(THEME["plot"])

        self.layout = StackLayout(self.fig, [0.70, 0.02, 0.28, 0.96], padding=0.005)

        # Construção da interface
        self.build_static_ui()
        self.toggle_offset_controls(False)
        self.update_dynamic_sliders()
        self.redraw()

        # Configuração inicial do modelo
        self.change_model(self.model_keys[0])

    def load_config_file(self) -> Dict:
        """Carrega o arquivo JSON com parâmetros otimizados."""
        filename = "best_legs_config.json"
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    data = json.load(f)
                print(f"[INFO] Configurações carregadas de '{filename}'")
                return data
            except Exception as e:
                print(f"[ERRO] Falha ao ler JSON: {e}")
                return {}
        return {}

    def build_static_ui(self):
        """Cria os elementos fixos da interface (cabeçalhos, atuadores, simulação)."""
        self.layout.add_header("CONTROLE CINEMÁTICO")
        self.layout.add_separator()

        self.radio_model = self.layout.add_radio(self.model_keys, active_index=0)
        self.radio_model.on_clicked(self.change_model)
        self.layout.add_separator()

        self.layout.add_header("ATUADORES")
        self.slider_theta1, _, _, _ = self.layout.add_slider(
            "Servo 1", 0, 180, 90, THEME["servo1"]
        )
        self.slider_theta2, _, _, _ = self.layout.add_slider(
            "Servo 2", 0, 180, 90, THEME["servo2"]
        )

        self.slider_theta1.on_changed(self.on_slider_change)
        self.slider_theta2.on_changed(self.on_slider_change)
        self.layout.add_separator()

        # Controles de Calibração (ocultos inicialmente para modelos simples)
        header_off = self.layout.add_header("CALIBRAÇÃO (Offsets)")
        self.slider_offset1, ax1, l1, v1 = self.layout.add_slider(
            "Offset S1", -90, 90, 0, "#666"
        )
        self.slider_offset2, ax2, l2, v2 = self.layout.add_slider(
            "Offset S2", -90, 90, 0, "#666"
        )

        self.slider_offset1.on_changed(self.on_offset_change)
        self.slider_offset2.on_changed(self.on_offset_change)

        sep_off = self.layout.add_separator()
        self.offset_group = [header_off, ax1, l1, v1, ax2, l2, v2, sep_off]

        self.layout.add_header("SIMULAÇÃO")
        self.slider_res, _, _, _ = self.layout.add_slider(
            "Resolução", 20, 180, self.sim_resolution, "#9b59b6"
        )
        self.slider_res.on_changed(self.on_res_change)
        self.layout.add_separator()

        # Marca ponto de inserção para elementos dinâmicos
        self.dynamic_start_y = self.layout.get_cursor()

    def toggle_offset_controls(self, visible: bool):
        """Exibe ou oculta os sliders de calibração."""
        for elem in self.offset_group:
            if elem:
                elem.set_visible(visible)

        self.slider_offset1.active = visible
        self.slider_offset2.active = visible

    def update_dynamic_sliders(self):
        """Reconstrói os sliders de parâmetros baseados no modelo atual."""
        # Limpeza segura de widgets antigos para evitar vazamento de memória/eventos
        if self.dynamic_sliders:
            for _, slider in self.dynamic_sliders.items():
                if hasattr(slider, "disconnect_events"):
                    slider.disconnect_events()

        if self.button_workspace:
            if hasattr(self.button_workspace, "disconnect_events"):
                self.button_workspace.disconnect_events()
            # Libera o mouse se estiver preso no widget antigo
            if (
                self.button_workspace.ax.figure.canvas.mouse_grabber
                == self.button_workspace.ax
            ):
                self.button_workspace.ax.figure.canvas.release_mouse(
                    self.button_workspace.ax
                )
            self.button_workspace = None

        for elem in self.dynamic_elements:
            try:
                elem.remove()
            except:
                pass

        self.dynamic_elements.clear()
        self.dynamic_sliders.clear()

        # Construção dos novos sliders
        self.layout.set_cursor(self.dynamic_start_y)

        header_param = self.layout.add_header("PARÂMETROS (mm)")
        self.dynamic_elements.append(header_param)

        params = list(self.current_model.parameters.items())
        for i, (label, (val, v_min, v_max)) in enumerate(params):
            col_idx = i % 2

            # Cria slider usando o valor atual do modelo
            slider, ax, lbl, val_txt = self.layout.add_slider(
                label, v_min, v_max, val, color="#888", cols=2, col_idx=col_idx
            )

            if slider:
                slider.on_changed(self.on_param_change)
                self.dynamic_sliders[label] = slider
                self.dynamic_elements.extend([ax, lbl, val_txt])

        self.layout.add_space(0.02)

        self.button_workspace, ax_btn = self.layout.add_button(
            "GERAR WORKSPACE",
            self.calc_workspace,
            height=0.06,
            color=THEME["widget_bg"],
        )

        if ax_btn:
            self.dynamic_elements.append(ax_btn)

        self.fig.canvas.draw_idle()

    def change_model(self, label: str):
        """Callback ao trocar o modelo no Radio Button."""
        self.current_model = self.models[label]

        # Reseta controles padrão
        self.slider_offset1.reset()
        self.slider_offset2.reset()
        self.slider_theta1.set_val(self.current_model.start_s1_angle)
        self.slider_theta2.set_val(self.current_model.start_s2_angle)

        # Limpa workspace
        self.workspace_x, self.workspace_y, self.workspace_poly = [], [], None
        self.workspace_area = 0.0

        # Aplica configurações otimizadas se disponíveis
        class_name = self.current_model.__class__.__name__

        if class_name in self.optimized_configs:
            print(f"--> Carregando otimização para {label}...")
            config = self.optimized_configs[class_name]
            best_params = config["parameters"].copy()

            # Aplica Offset S1 (específico para Five-Bar)
            if "Offset_S1_Deg" in best_params:
                offset_deg = best_params.pop("Offset_S1_Deg")
                self.slider_offset1.set_val(offset_deg)
                self.current_model.offset_t1 = np.radians(offset_deg)

            # Aplica Offset S2 (específico para Five-Bar)
            if "Offset_S2_Deg" in best_params:
                offset_deg = best_params.pop("Offset_S2_Deg")
                self.slider_offset2.set_val(offset_deg)
                self.current_model.offset_t2 = np.radians(offset_deg)

            # Atualiza modelo físico
            self.current_model.update_params(best_params)

            # Atualiza valores padrão para os sliders dinâmicos
            for key, val in best_params.items():
                if key in self.current_model.parameters:
                    self.current_model.parameters[key][0] = val

        # Atualiza interface
        is_fivebar = "Five-Bar" in label
        self.toggle_offset_controls(is_fivebar)
        self.update_dynamic_sliders()
        self.calc_workspace()
        self.redraw()

    def on_slider_change(self, val):
        self.theta1 = self.slider_theta1.val
        self.theta2 = self.slider_theta2.val
        self.redraw()

    def on_offset_change(self, val):
        if self.slider_offset1.active:
            self.current_model.offset_t1 = np.radians(self.slider_offset1.val)
            self.current_model.offset_t2 = np.radians(self.slider_offset2.val)
            self.redraw()

    def on_param_change(self, val):
        new_params = {k: s.val for k, s in self.dynamic_sliders.items()}
        self.current_model.update_params(new_params)
        self.calc_workspace()
        self.redraw()

    def on_res_change(self, val):
        self.sim_resolution = int(val)

    def calc_workspace(self, event=None):
        """Calcula e exibe a nuvem de pontos do espaço de trabalho."""
        results = self.current_model.get_workspace(resolution=self.sim_resolution)
        self.workspace_x, self.workspace_y, self.workspace_area, self.workspace_poly = (
            results
        )
        self.redraw(draw_ws=True)

    def redraw(self, draw_ws=False):
        """Loop principal de desenho."""

        ellipse_text = ""

        # Garante offsets zerados se os controles estiverem ocultos
        if not self.slider_offset1.active:
            self.current_model.offset_t1 = 0
            self.current_model.offset_t2 = 0

        # Cálculo cinemático
        end_effector, joints = self.current_model.forward_kinematics(
            self.theta1, self.theta2
        )

        # Verificação de estado
        status_msg = "OK"
        status_color = THEME["accent"]

        if end_effector is None:
            status_msg = "GEOMETRIA INVÁLIDA"
            status_color = THEME["warn"]
        elif self.current_model.check_collisions(joints):
            status_msg = "COLISÃO FÍSICA"
            status_color = THEME["warn"]

        # Limpeza e configuração do plot
        ax = self.ax_sim
        ax.clear()
        ax.set_aspect("equal")
        ax.set_xlim(-150, 150)
        ax.set_ylim(200, -100)

        # Grid
        ax.grid(True, color="#333", linestyle="-", linewidth=1, alpha=0.5)
        ax.axhline(0, color="#555", lw=1)
        ax.axvline(0, color="#555", lw=1)

        # Desenho do workspace
        if len(self.workspace_x) > 0:
            from matplotlib.collections import LineCollection

            # Pontos
            pt_size = 30 if len(self.workspace_x) < 1000 else 10
            ax.scatter(
                self.workspace_x,
                self.workspace_y,
                s=pt_size,
                c=THEME["accent"],
                alpha=0.15,
                edgecolors="none",
                zorder=1,
            )

            # Contorno (concave hull)
            if self.workspace_poly and len(self.workspace_poly) > 0:
                lc = LineCollection(
                    self.workspace_poly,
                    colors=THEME["accent"],
                    linewidths=1.5,
                    linestyles="-",
                    alpha=0.8,
                    zorder=2,
                )
                ax.add_collection(lc)

                cy, w, h = calculate_max_centered_ellipse(self.workspace_poly)

                if w > 0:
                    # Desenha a elipse
                    ellipse = Ellipse(
                        (0, cy),
                        w,
                        h,
                        edgecolor="#00ff00",
                        facecolor="#00ff00",
                        alpha=0.3,
                        lw=2,
                        zorder=3,
                        linestyle="--",
                    )
                    ax.add_patch(ellipse)

                    # Marca o centro
                    ax.plot(0, cy, "+", color="#00ff00", markersize=10, zorder=3)

                    ellipse_text = f"ELIPSE (X=0):\nLargura: {w:.1f} mm\nAltura:  {h:.1f} mm"

        # Desenho do robô
        if end_effector is not None:
            self.current_model.draw(ax, joints, self.theta1, self.theta2, THEME)

            # Marcador do efetuador final
            ax.plot(
                end_effector[0],
                end_effector[1],
                "o",
                color=status_color,
                markersize=8,
                markeredgecolor="white",
                zorder=20,
            )

            coord_text = f"X: {end_effector[0]:6.1f}\nY: {end_effector[1]:6.1f}"
        else:
            coord_text = "X: ---\nY: ---"

        # HUD
        area_cm2 = self.workspace_area / 100.0
        info_text = (
            f"STATUS: {status_msg}\n"
            f"------------------\n"
            f"POSIÇÃO PÉ (mm)\n"
            f"{coord_text}\n"
            f"------------------\n"
            f"ÁREA ÚTIL\n"
            f"{area_cm2:6.1f} cm²"
        )

        if ellipse_text:
            info_text += f"\n------------------\n{ellipse_text}"

        # Caixa de informações
        ax.text(
            0.02,
            0.98,
            info_text,
            transform=ax.transAxes,
            va="top",
            color=status_color,
            fontsize=10,
            family="monospace",
            weight="bold",
            bbox=dict(
                facecolor="black",
                alpha=0.8,
                edgecolor=status_color,
                boxstyle="round,pad=0.5",
            ),
        )

        self.fig.canvas.draw_idle()
