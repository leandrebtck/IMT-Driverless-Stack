#!/usr/bin/env python3
"""
eval_dashboard.py — Dashboard graphique d'évaluation SLAM en temps réel.

Affiche :
  - Jauge circulaire : précision globale (0 % = 1.5 m d'erreur, 100 % = 0 m)
  - Métriques détaillées : erreur moy, RMSE, min, max, cônes détectés, fantômes
  - Historique graphique de l'erreur moyenne sur les N dernières secondes
  - Code couleur : vert > 70 %, jaune 40-70 %, rouge < 40 %

Souscrit à /evaluation/stats (std_msgs/String publié par cone_evaluator.py).
"""

import math
import threading
import time
import re
import argparse
import tkinter as tk
from tkinter import font as tkfont
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Paramètres ────────────────────────────────────────────────────────────────
MAX_ERROR_M   = 1.5    # erreur max de référence (0 % de précision)
HISTORY_LEN   = 120    # nombre de points dans l'historique
BG            = '#0d1117'
FG            = '#e6edf3'
ACCENT        = '#58a6ff'
CARD_BG       = '#161b22'
BORDER        = '#30363d'
GREEN         = '#3fb950'
YELLOW        = '#d29922'
RED           = '#f85149'
GRAY          = '#6e7681'


def accuracy_color(pct: float) -> str:
    if pct >= 70:
        return GREEN
    elif pct >= 40:
        return YELLOW
    return RED


def parse_stats(text: str) -> dict:
    """Extrait les métriques du message /evaluation/stats."""
    d = {'detected': 0, 'total': 0, 'mean': None, 'rmse': None,
         'min': None, 'max': None, 'ghosts': 0}
    m = re.search(r'(\d+)/(\d+)', text)
    if m:
        d['detected'] = int(m.group(1))
        d['total']    = int(m.group(2))
    for key, pat in [('mean', r'moy=([\d.]+)m'),
                     ('rmse', r'RMSE=([\d.]+)m'),
                     ('min',  r'min=([\d.]+)m'),
                     ('max',  r'max=([\d.]+)m'),
                     ('ghosts', r'fantômes=(\d+)')]:
        m2 = re.search(pat, text)
        if m2:
            d[key] = float(m2.group(1))
    return d


# ── Nœud ROS2 (tourne dans un thread séparé) ──────────────────────────────────
class StatsSubscriber(Node):
    def __init__(self, topic: str, callback):
        super().__init__('eval_dashboard_node')
        self.create_subscription(String, topic, lambda msg: callback(msg.data), 10)


# ── Dashboard Tkinter ──────────────────────────────────────────────────────────
class Dashboard:

    GAUGE_R   = 110   # rayon de la jauge
    GAUGE_CX  = 160   # centre X
    GAUGE_CY  = 155   # centre Y

    def __init__(self, root: tk.Tk, topic: str):
        self.root    = root
        self.topic   = topic
        self.data    = {}
        self.history = deque(maxlen=HISTORY_LEN)
        self.last_update = 0.0

        root.title("SLAM Evaluation Dashboard")
        root.configure(bg=BG)
        root.resizable(False, False)

        self._build_ui()
        self._update_loop()

    # ── Construction UI ───────────────────────────────────────────────────────
    def _build_ui(self):
        root = self.root
        title_f = tkfont.Font(family='Helvetica', size=13, weight='bold')
        big_f   = tkfont.Font(family='Helvetica', size=32, weight='bold')
        med_f   = tkfont.Font(family='Helvetica', size=11)
        small_f = tkfont.Font(family='Helvetica', size=9)
        mono_f  = tkfont.Font(family='Courier',   size=9)

        # ── Titre ─────────────────────────────────────────────────────────────
        tk.Label(root, text="SLAM — Évaluation en temps réel",
                 bg=BG, fg=ACCENT, font=title_f).grid(
                     row=0, column=0, columnspan=2, pady=(14, 4), padx=16, sticky='w')
        tk.Label(root, text=f"topic : {self.topic}",
                 bg=BG, fg=GRAY, font=small_f).grid(
                     row=1, column=0, columnspan=2, padx=16, sticky='w')

        # ── Jauge circulaire ──────────────────────────────────────────────────
        gauge_frame = tk.Frame(root, bg=CARD_BG, relief='flat',
                               highlightbackground=BORDER, highlightthickness=1)
        gauge_frame.grid(row=2, column=0, padx=16, pady=10, sticky='nsew')

        self.canvas = tk.Canvas(gauge_frame, width=320, height=220,
                                bg=CARD_BG, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        # ── Métriques ─────────────────────────────────────────────────────────
        metrics_frame = tk.Frame(root, bg=CARD_BG,
                                 highlightbackground=BORDER, highlightthickness=1)
        metrics_frame.grid(row=2, column=1, padx=(0, 16), pady=10, sticky='nsew')

        labels = [
            ("Erreur moyenne",  'mean_lbl'),
            ("RMSE",            'rmse_lbl'),
            ("Min / Max",       'minmax_lbl'),
            ("Cônes détectés",  'det_lbl'),
            ("Cônes fantômes",  'ghost_lbl'),
            ("Dernière MàJ",    'time_lbl'),
        ]
        self._metric_vars = {}
        for i, (txt, key) in enumerate(labels):
            tk.Label(metrics_frame, text=txt, bg=CARD_BG, fg=GRAY,
                     font=small_f, anchor='w').grid(
                         row=i, column=0, padx=14, pady=5, sticky='w')
            var = tk.StringVar(value='—')
            self._metric_vars[key] = var
            tk.Label(metrics_frame, textvariable=var, bg=CARD_BG, fg=FG,
                     font=med_f, anchor='e').grid(
                         row=i, column=1, padx=14, pady=5, sticky='e')
        metrics_frame.columnconfigure(1, weight=1)

        # ── Historique ────────────────────────────────────────────────────────
        hist_frame = tk.Frame(root, bg=CARD_BG,
                              highlightbackground=BORDER, highlightthickness=1)
        hist_frame.grid(row=3, column=0, columnspan=2, padx=16, pady=(0, 16),
                        sticky='nsew')
        tk.Label(hist_frame, text="Historique erreur moyenne (m)",
                 bg=CARD_BG, fg=GRAY, font=small_f).pack(anchor='w', padx=10, pady=(6, 0))
        self.hist_canvas = tk.Canvas(hist_frame, width=588, height=80,
                                     bg=CARD_BG, highlightthickness=0)
        self.hist_canvas.pack(padx=10, pady=(2, 8))

        # Légende historique
        leg = tk.Frame(hist_frame, bg=CARD_BG)
        leg.pack(anchor='w', padx=10, pady=(0, 6))
        for color, label in [(GREEN, '>0.75 m ok'), (YELLOW, '0.75–1.2 m moyen'),
                              (RED, '>1.2 m mauvais')]:
            tk.Canvas(leg, width=14, height=14, bg=CARD_BG,
                      highlightthickness=0).pack(side='left')
            c = tk.Canvas(leg, width=12, height=12, bg=color,
                          highlightthickness=0)
            c.pack(side='left', padx=(0, 2))
            tk.Label(leg, text=label, bg=CARD_BG, fg=GRAY,
                     font=small_f).pack(side='left', padx=(0, 12))

        self._draw_gauge(None)
        self._draw_history()

    # ── Jauge circulaire ──────────────────────────────────────────────────────
    def _draw_gauge(self, pct):
        c  = self.canvas
        cx, cy, r = self.GAUGE_CX, self.GAUGE_CY, self.GAUGE_R
        c.delete('all')

        # Arc de fond (gris)
        c.create_arc(cx-r, cy-r, cx+r, cy+r,
                     start=220, extent=-260,
                     style='arc', outline=BORDER, width=18)

        if pct is not None:
            color  = accuracy_color(pct)
            extent = -260 * (pct / 100.0)
            c.create_arc(cx-r, cy-r, cx+r, cy+r,
                         start=220, extent=extent,
                         style='arc', outline=color, width=18)

            # Valeur centrale
            c.create_text(cx, cy - 10, text=f"{pct:.0f}%",
                          fill=color, font=('Helvetica', 36, 'bold'))
            c.create_text(cx, cy + 30, text="précision",
                          fill=GRAY, font=('Helvetica', 10))

            # Aiguille
            angle_deg = 220 - 260 * (pct / 100.0)
            angle_rad = math.radians(angle_deg)
            nx = cx + (r - 20) * math.cos(angle_rad)
            ny = cy - (r - 20) * math.sin(angle_rad)
            c.create_line(cx, cy, nx, ny, fill=color, width=3,
                          capstyle='round')
            c.create_oval(cx-5, cy-5, cx+5, cy+5, fill=BORDER, outline='')
        else:
            c.create_text(cx, cy, text="En attente…",
                          fill=GRAY, font=('Helvetica', 13))

        # Graduations 0 % et 100 %
        for pct_g, label in [(0, '0%'), (100, '100%')]:
            a = math.radians(220 - 260 * (pct_g / 100.0))
            lx = cx + (r + 14) * math.cos(a)
            ly = cy - (r + 14) * math.sin(a)
            c.create_text(lx, ly, text=label, fill=GRAY,
                          font=('Helvetica', 8))

    # ── Historique ────────────────────────────────────────────────────────────
    def _draw_history(self):
        c  = self.hist_canvas
        W, H = 588, 80
        c.delete('all')

        # Axes
        c.create_line(30, 5, 30, H-15, fill=BORDER)
        c.create_line(30, H-15, W-10, H-15, fill=BORDER)

        if not self.history:
            c.create_text(W//2, H//2, text="Pas encore de données",
                          fill=GRAY, font=('Helvetica', 9))
            return

        max_e = max(max(self.history), MAX_ERROR_M)
        pts   = list(self.history)
        n     = len(pts)
        x_step = (W - 45) / max(HISTORY_LEN - 1, 1)

        coords = []
        for i, e in enumerate(pts):
            x = 30 + (HISTORY_LEN - n + i) * x_step
            y = (H - 15) - (e / max_e) * (H - 25)
            coords.append((x, y))

        # Ligne colorée segment par segment
        for i in range(1, len(coords)):
            e = pts[i]
            if e < 0.75:
                col = GREEN
            elif e < 1.2:
                col = YELLOW
            else:
                col = RED
            x0, y0 = coords[i-1]
            x1, y1 = coords[i]
            c.create_line(x0, y0, x1, y1, fill=col, width=2)

        # Ligne de référence MAX_ERROR_M
        ref_y = (H - 15) - (MAX_ERROR_M / max_e) * (H - 25)
        c.create_line(30, ref_y, W-10, ref_y, fill=RED, dash=(4, 4), width=1)
        c.create_text(W-8, ref_y - 6, text=f"{MAX_ERROR_M:.1f}m",
                      fill=RED, font=('Helvetica', 7), anchor='e')

        # Labels axe Y
        for frac, label in [(0, '0m'), (0.5, f'{MAX_ERROR_M*0.5:.2f}m'),
                             (1.0, f'{MAX_ERROR_M:.1f}m')]:
            yy = (H - 15) - frac * (H - 25)
            c.create_text(28, yy, text=label, fill=GRAY,
                          font=('Helvetica', 7), anchor='e')

    # ── Boucle de mise à jour UI ──────────────────────────────────────────────
    def on_stats(self, text: str):
        """Appelé depuis le thread ROS2 quand un nouveau message arrive."""
        self.data      = parse_stats(text)
        self.last_update = time.time()
        if self.data.get('mean') is not None:
            self.history.append(self.data['mean'])

    def _update_loop(self):
        d = self.data
        if d:
            mean  = d.get('mean')
            pct   = max(0.0, (1 - mean / MAX_ERROR_M) * 100) if mean is not None else None
            col   = accuracy_color(pct) if pct is not None else GRAY

            self._draw_gauge(pct)
            self._draw_history()

            fmt = lambda v, unit='m': f"{v:.3f} {unit}" if v is not None else '—'
            self._metric_vars['mean_lbl'].set(
                f"{fmt(d.get('mean'))}  " + (f"({pct:.1f}%)" if pct else ''))
            self._metric_vars['rmse_lbl'].set(fmt(d.get('rmse')))
            self._metric_vars['minmax_lbl'].set(
                f"{fmt(d.get('min'))} / {fmt(d.get('max'))}")
            det = d.get('detected', 0); tot = d.get('total', 0)
            self._metric_vars['det_lbl'].set(
                f"{det} / {tot}  ({int(det/tot*100) if tot else 0}%)")
            g = int(d.get('ghosts', 0))
            self._metric_vars['ghost_lbl'].set(
                f"{g}" + ("  ⚠" if g > 2 else ""))
            age = time.time() - self.last_update
            self._metric_vars['time_lbl'].set(
                f"il y a {age:.1f}s" + ("  ⏸" if age > 5 else ""))

        self.root.after(500, self._update_loop)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/evaluation/stats',
                        help='Topic std_msgs/String des statistiques')
    parsed, _ = parser.parse_known_args()

    # Tkinter dans le thread principal
    root = tk.Tk()
    dash = Dashboard(root, parsed.topic)

    # ROS2 dans un thread daemon
    def ros_thread():
        rclpy.init()
        node = StatsSubscriber(parsed.topic, dash.on_stats)
        try:
            rclpy.spin(node)
        except Exception:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()

    root.mainloop()


if __name__ == '__main__':
    main()
