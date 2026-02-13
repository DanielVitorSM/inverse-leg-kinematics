import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from settings import THEME

class StackLayout:
    def __init__(self, fig, rect, padding=0.005):
        # Inicializa o layout vertical
        self.fig = fig
        self.x = rect[0]
        self.bottom_limit = rect[1]
        self.width = rect[2]
        self.top_y = rect[1] + rect[3]
        self.current_y = self.top_y
        self.padding = padding
        
        # Fundo do painel
        self.ax_bg = fig.add_axes(rect)
        self.ax_bg.set_facecolor(THEME['panel'])
        self.ax_bg.set_zorder(0)
        self.ax_bg.axis('off')

    def add_space(self, size=None):
        if size is None: 
            size = self.padding
        self.current_y -= size

    def check_space(self, height):
        if (self.current_y - height) < self.bottom_limit:
            return False
        return True

    def add_header(self, text, size=0.025):
        """Adiciona um texto de cabeçalho."""
        if not self.check_space(size): 
            return None
            
        self.current_y -= size
        text_obj = self.fig.text(self.x, self.current_y, text, color=THEME['text'], 
                                 fontsize=10, weight='bold', va='center')
        self.add_space(0.005)
        return text_obj

    def add_separator(self):
        """Desenha uma linha horizontal."""
        if not self.check_space(0.01): 
            return None
            
        self.add_space(0.005)
        line = plt.Line2D([self.x, self.x + self.width], 
                          [self.current_y, self.current_y], 
                          transform=self.fig.transFigure, color='#444', linewidth=1)
        self.fig.lines.append(line)
        self.add_space(0.01)
        return line

    def add_radio(self, options, active_index=0, height=0.10):
        """Adiciona botões de rádio."""
        if not self.check_space(height): 
            return None
            
        self.current_y -= height
        rect = [self.x, self.current_y, self.width, height]
        ax = self.fig.add_axes(rect, facecolor=THEME['panel'])
        
        radio = RadioButtons(ax, options, active=active_index, 
                             activecolor=THEME['accent'], radio_props={'s': [50]*len(options)})
        
        for label in radio.labels: 
            label.set_color(THEME['text'])
            label.set_fontsize(9)
        
        self.add_space()
        return radio

    def add_slider(self, label, val_min, val_max, val_init, color=THEME['subtext'], cols=1, col_idx=0):
        """Adiciona um slider, suportando layout em colunas."""
        # Lógica de grid
        col_width = (self.width - (self.padding * (cols - 1))) / cols
        col_x = self.x + (col_idx * (col_width + self.padding))
        
        height_text = 0.02
        height_bar = 0.025
        total_height = height_text + height_bar

        # Só desce o cursor se for a primeira coluna
        if col_idx == 0:
            if not self.check_space(total_height): 
                return None, None, None, None
            self.current_y -= height_text
        
        y_text = self.current_y + 0.005
        y_bar = self.current_y - height_bar

        # Elementos de texto
        label_obj = self.fig.text(col_x, y_text, label, color=THEME['text'], fontsize=8)
        val_obj = self.fig.text(col_x + col_width, y_text, f"{val_init:.1f}", 
                                color=color, fontsize=8, ha='right')

        # Eixo do slider
        rect = [col_x, y_bar, col_width, height_bar]
        ax = self.fig.add_axes(rect, facecolor=THEME['widget_bg'])
        
        slider = Slider(ax, '', val_min, val_max, valinit=val_init, 
                        valstep=1 if val_max == 180 else 5, color=color)
        slider.label.set_visible(False)
        slider.valtext.set_visible(False)
        
        # Callback para atualizar texto
        orig_on_changed = slider.on_changed
        def new_on_changed(func):
            def wrapper(val):
                val_obj.set_text(f"{val:.1f}")
                func(val)
            return orig_on_changed(wrapper)
        slider.on_changed = new_on_changed
        
        # Atualiza cursor apenas na última coluna
        if col_idx == cols - 1:
            self.current_y -= height_bar
            self.add_space(0.015)
            
        return slider, ax, label_obj, val_obj

    def add_button(self, label, callback, height=0.06, color='#333'):
        """Adiciona um botão simples."""
        if not self.check_space(height): 
            return None, None
            
        self.current_y -= height
        rect = [self.x, self.current_y, self.width, height]
        ax = self.fig.add_axes(rect)
        
        btn = Button(ax, label, color=color, hovercolor=THEME['accent'])
        btn.label.set_color('white')
        btn.label.set_weight('bold')
        btn.on_clicked(callback)
        
        self.add_space()
        return btn, ax

    def get_cursor(self):
        return self.current_y

    def set_cursor(self, y):
        self.current_y = y