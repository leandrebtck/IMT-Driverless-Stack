#!/usr/bin/env python3
"""
launcher.py — Interface graphique de lancement des scripts IMT Driverless.
Lance ce fichier depuis la racine du repo :  python3 launcher.py
"""

import os
import sys
import subprocess
import tkinter as tk
from tkinter import messagebox

# Import du collecteur de métriques système depuis system_monitor.py
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))
try:
    from system_monitor import DataCollector, pct_color, bytes_to_str, GPU_OK
    MONITOR_OK = True
except Exception:
    MONITOR_OK = False

REPO_ROOT    = os.path.dirname(os.path.abspath(__file__))
EXCLUDE_DIRS = {'ros_workspace', '.git', 'site', 'docs', '__pycache__'}

# ── Palette ──────────────────────────────────────────────────────────────────
BG      = "#1e1e2e"
BG2     = "#313244"
BG3     = "#181825"
FG      = "#cdd6f4"
FG_DIM  = "#a6adc8"
ACCENT  = "#89b4fa"
ACCENT2 = "#b4befe"
GREEN   = "#a6e3a1"
RED     = "#f38ba8"
YELLOW  = "#f9e2af"
GRAY    = "#6c7086"
BORDER  = "#45475a"


# ── Utilitaires scripts ───────────────────────────────────────────────────────
def find_scripts():
    scripts = []
    for dirpath, dirnames, filenames in os.walk(REPO_ROOT):
        dirnames[:] = sorted(d for d in dirnames if d not in EXCLUDE_DIRS)
        for f in sorted(filenames):
            if f.endswith('.sh'):
                full = os.path.join(dirpath, f)
                rel  = os.path.relpath(full, REPO_ROOT)
                scripts.append((rel, full))
    return scripts


def get_description(path):
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if line.startswith('#') and not line.startswith('#!'):
                    desc = line.lstrip('#').strip(' =–-')
                    if len(desc) > 4:
                        return desc
    except Exception:
        pass
    return "—"


def friendly_name(rel_path):
    return os.path.basename(rel_path).replace('.sh', '').replace('_', ' ').title()


def category(rel_path):
    folder = os.path.dirname(rel_path)
    return folder if folder else "racine"


# ── Widget barre de progression ───────────────────────────────────────────────
class MiniBar:
    def __init__(self, parent, width=180, height=10):
        self.c  = tk.Canvas(parent, width=width, height=height,
                            bg=BG3, highlightthickness=0)
        self._w = width
        self._h = height

    def grid(self, **kw):
        self.c.grid(**kw)

    def set(self, pct, color):
        self.c.delete('all')
        self.c.create_rectangle(0, 0, self._w, self._h, fill=BORDER, outline='')
        w = max(1, int(self._w * min(pct, 100) / 100))
        self.c.create_rectangle(0, 0, w, self._h, fill=color, outline='')


# ── Panneau monitoring intégré ────────────────────────────────────────────────
class MonitorPanel(tk.Frame):

    PROC_KW = ['yolo_lidar', 'yolo_stereo', 'cone_mapper', 'odom_tf',
               'global_drive', 'cone_evaluator', 'fsds_ros2_bridge', 'FSDS']

    def __init__(self, parent, collector):
        super().__init__(parent, bg=BG3,
                         highlightbackground=BORDER, highlightthickness=1)
        self.collector = collector
        self._build()
        self._update()

    def _lbl(self, parent, text, fg=FG_DIM, font=("Helvetica", 8), **kw):
        l = tk.Label(parent, text=text, bg=BG3, fg=fg, font=font, **kw)
        return l

    def _section(self, text):
        f = tk.Frame(self, bg=BORDER, height=1)
        f.pack(fill=tk.X, pady=(6, 0))
        tk.Label(self, text=text, bg=BG3, fg=ACCENT,
                 font=("Helvetica", 8, "bold")).pack(anchor='w', padx=8)

    def _build(self):
        tk.Label(self, text="Ressources système", bg=BG3, fg=ACCENT2,
                 font=("Helvetica", 9, "bold")).pack(anchor='w', padx=8, pady=(8, 2))
        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X)

        # ── CPU ───────────────────────────────────────────────────────────────
        self._section("RAM (scripts Python)")
        row = tk.Frame(self, bg=BG3); row.pack(fill=tk.X, padx=8, pady=2)
        self._cpu_val = tk.Label(row, text="— Go", bg=BG3, fg=GREEN,
                                 font=("Helvetica", 10, "bold"), anchor='w')
        self._cpu_val.pack(side=tk.LEFT)
        self._cpu_freq = tk.Label(row, text="", bg=BG3, fg=GRAY,
                                  font=("Helvetica", 7), anchor='e')
        self._cpu_freq.pack(side=tk.RIGHT)
        self._cpu_bar = MiniBar(self, width=190)
        self._cpu_bar.c.pack(padx=8, pady=(0, 2))

        # ── CPU ───────────────────────────────────────────────────────────────
        self._section("CPU (scripts Python)")
        row2 = tk.Frame(self, bg=BG3); row2.pack(fill=tk.X, padx=8, pady=2)
        self._ram_val = tk.Label(row2, text="— cœurs", bg=BG3, fg=GREEN,
                                 font=("Helvetica", 10, "bold"), anchor='w')
        self._ram_val.pack(side=tk.LEFT)
        self._ram_info = tk.Label(row2, text="", bg=BG3, fg=GRAY,
                                  font=("Helvetica", 7), anchor='e')
        self._ram_info.pack(side=tk.RIGHT)
        self._ram_bar = MiniBar(self, width=190)
        self._ram_bar.c.pack(padx=8, pady=(0, 2))

        # ── GPU ───────────────────────────────────────────────────────────────
        self._section("VRAM (scripts Python)")
        row3 = tk.Frame(self, bg=BG3); row3.pack(fill=tk.X, padx=8, pady=2)
        self._gpu_val = tk.Label(row3, text="N/A" if not GPU_OK else "— Go",
                                 bg=BG3, fg=GRAY if not GPU_OK else GREEN,
                                 font=("Helvetica", 12, "bold"), width=6, anchor='w')
        self._gpu_val.pack(side=tk.LEFT)
        self._gpu_info = tk.Label(row3, text="", bg=BG3, fg=GRAY,
                                  font=("Helvetica", 7), anchor='e')
        self._gpu_info.pack(side=tk.RIGHT)
        self._gpu_bar = MiniBar(self, width=190)
        self._gpu_bar.c.pack(padx=8, pady=(0, 2))
        self._gpu_mem_bar = MiniBar(self, width=190, height=4)
        self._gpu_mem_bar.c.pack(padx=8, pady=(0, 2))
        self._gpu_mem_lbl = tk.Label(self, text="", bg=BG3, fg=GRAY,
                                     font=("Helvetica", 7))
        self._gpu_mem_lbl.pack(anchor='w', padx=8)

        # ── Processus (RAM Go + CPU cœurs) ────────────────────────────────────
        self._section("Par script  RAM·cœurs")
        self._proc_frame = tk.Frame(self, bg=BG3)
        self._proc_frame.pack(fill=tk.X, padx=8, pady=(2, 6))
        self._proc_labels = []

        # ── Bouton détacher ───────────────────────────────────────────────────
        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X, pady=(4, 0))
        tk.Button(self, text="Ouvrir dashboard complet",
                  bg=BG2, fg=ACCENT, font=("Helvetica", 8),
                  relief="flat", cursor="hand2",
                  command=self._open_full).pack(pady=6)

    def _color(self, pct):
        if pct < 60:   return GREEN
        if pct < 85:   return YELLOW
        return RED

    def _col(self, v, max_v):
        if max_v == 0: return GREEN
        r = v / max_v
        if r < 0.5:  return GREEN
        if r < 0.75: return YELLOW
        return RED

    def _update(self):
        import psutil as _ps
        d = self.collector.data if self.collector else {}
        if d:
            vm     = _ps.virtual_memory()
            ram_t  = vm.total / (1 << 30)
            ncores = d.get('n_cores', 1)
            gtotal = d.get('gpu_mem_total', 8.0) or 8.0

            # RAM en Go
            ram = d.get('total_ram_gb', 0)
            col = self._col(ram, ram_t)
            self._cpu_val.config(text=f"{ram:.2f} Go", fg=col)
            self._cpu_bar.set(int(ram / ram_t * 100), col)
            self._cpu_freq.config(text=f"/{ram_t:.1f} Go")

            # CPU en cœurs
            cpu = d.get('total_cpu_cores', 0)
            col = self._col(cpu, ncores)
            self._ram_val.config(text=f"{cpu:.2f} cœurs", fg=col)
            self._ram_bar.set(int(cpu / ncores * 100), col)
            self._ram_info.config(text=f"/{ncores} cœurs")

            # GPU VRAM en Go
            vram = d.get('total_vram_gb', 0)
            if GPU_OK:
                col = self._col(vram, gtotal)
                self._gpu_val.config(text=f"{vram:.2f} Go", fg=col)
                self._gpu_bar.set(int(vram / gtotal * 100), col)
                self._gpu_mem_bar.set(0, GRAY)
                temp = d.get('gpu_temp', '?')
                self._gpu_info.config(text=f"{temp}°C")
                self._gpu_mem_lbl.config(text=f"/{gtotal:.1f} Go VRAM")

            # Processus (RAM Go + CPU cœurs)
            procs = d.get('procs', [])[:6]
            while len(self._proc_labels) < len(procs):
                r = tk.Frame(self._proc_frame, bg=BG3)
                r.pack(fill=tk.X, pady=1)
                n = tk.Label(r, text='', bg=BG3, fg=FG,
                             font=("Courier", 8), width=14, anchor='w')
                n.pack(side=tk.LEFT)
                c = tk.Label(r, text='', bg=BG3, fg=GRAY,
                             font=("Courier", 8), width=12, anchor='e')
                c.pack(side=tk.RIGHT)
                self._proc_labels.append((r, n, c))

            for i, (row, nlbl, clbl) in enumerate(self._proc_labels):
                if i < len(procs):
                    p   = procs[i]
                    col = self._col(p['ram_gb'], ram_t)
                    nlbl.config(text=p['name'][:14], fg=FG)
                    clbl.config(text=f"{p['ram_gb']:.2f}Go {p['cpu_cores']:.2f}c",
                                fg=col)
                    row.pack(fill=tk.X, pady=1)
                else:
                    row.pack_forget()

        self.after(1000, self._update)

    def _open_full(self):
        script = os.path.join(REPO_ROOT, 'tools', 'system_monitor.py')
        subprocess.Popen([sys.executable, script])


# ── Application principale ────────────────────────────────────────────────────
class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IMT Driverless — Launcher")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(800, 480)
        self._scripts = find_scripts()

        # Collecteur de métriques (thread daemon)
        self._collector = None
        if MONITOR_OK:
            self._collector = DataCollector()
            self._collector.start()

        self._build_ui()
        self._populate()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Header ────────────────────────────────────────────────────────────
        hdr = tk.Frame(self, bg=BG, pady=12)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="IMT Driverless Stack",
                 font=("Helvetica", 16, "bold"), bg=BG, fg=ACCENT).pack()
        tk.Label(hdr, text="Double-clic ou bouton Lancer pour exécuter un script",
                 font=("Helvetica", 9), bg=BG, fg=FG_DIM).pack()
        tk.Frame(self, bg=BG2, height=1).pack(fill=tk.X)

        # ── Contenu principal (liste gauche + monitor droit) ──────────────────
        main = tk.Frame(self, bg=BG)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=0)
        main.rowconfigure(0, weight=1)

        # Liste scripts
        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 8))

        sb = tk.Scrollbar(left, bg=BG2, troughcolor=BG)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox = tk.Listbox(
            left, font=("Monospace", 11),
            bg=BG2, fg=FG,
            selectbackground=ACCENT, selectforeground=BG,
            activestyle="none", bd=0, highlightthickness=0,
            yscrollcommand=sb.set
        )
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.config(command=self.listbox.yview)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)
        self.listbox.bind("<Double-Button-1>", lambda e: self._launch())

        # Panneau monitor
        if MONITOR_OK:
            self._monitor_panel = MonitorPanel(main, self._collector)
            self._monitor_panel.grid(row=0, column=1, sticky='ns')
        else:
            tk.Label(main, text="psutil\nnon\ninstallé",
                     bg=BG3, fg=GRAY, font=("Helvetica", 8)).grid(
                         row=0, column=1, sticky='ns', padx=4)

        # ── Description ───────────────────────────────────────────────────────
        desc_frame = tk.Frame(self, bg=BG, padx=10)
        desc_frame.pack(fill=tk.X)
        tk.Label(desc_frame, text="Description :", font=("Helvetica", 9, "bold"),
                 bg=BG, fg=FG_DIM).pack(anchor="w")
        self.desc_var = tk.StringVar(value="Sélectionne un script…")
        tk.Label(desc_frame, textvariable=self.desc_var,
                 font=("Helvetica", 9, "italic"), bg=BG, fg=FG_DIM,
                 wraplength=560, justify="left").pack(anchor="w")

        # ── Boutons ───────────────────────────────────────────────────────────
        btn_frame = tk.Frame(self, bg=BG, pady=10)
        btn_frame.pack()
        self.btn_launch = tk.Button(
            btn_frame, text="▶   Lancer",
            font=("Helvetica", 12, "bold"),
            bg=GREEN, fg=BG, activebackground=ACCENT2, activeforeground=BG,
            relief="flat", padx=24, pady=8, cursor="hand2",
            command=self._launch
        )
        self.btn_launch.pack(side=tk.LEFT, padx=8)
        tk.Button(
            btn_frame, text="↻   Actualiser",
            font=("Helvetica", 10),
            bg=BG2, fg=FG, activebackground=ACCENT, activeforeground=BG,
            relief="flat", padx=14, pady=8, cursor="hand2",
            command=self._refresh
        ).pack(side=tk.LEFT, padx=8)

    # ── Données ───────────────────────────────────────────────────────────────
    def _populate(self):
        self.listbox.delete(0, tk.END)
        current_cat = None
        for rel, full in self._scripts:
            cat = category(rel)
            if cat != current_cat:
                self.listbox.insert(tk.END, f"── {cat} ──")
                self.listbox.itemconfig(tk.END, fg=ACCENT2,
                                        selectbackground=BG2, selectforeground=ACCENT2)
                current_cat = cat
            self.listbox.insert(tk.END, f"   {friendly_name(rel)}")

    def _refresh(self):
        self._scripts = find_scripts()
        self._populate()
        self.desc_var.set("Sélectionne un script…")

    # ── Sélection ─────────────────────────────────────────────────────────────
    def _selected_script(self):
        sel = self.listbox.curselection()
        if not sel:
            return None
        item = self.listbox.get(sel[0]).strip()
        if item.startswith("──"):
            return None
        name = item.strip()
        for rel, full in self._scripts:
            if friendly_name(rel) == name:
                return rel, full
        return None

    def _on_select(self, _event):
        script = self._selected_script()
        if script:
            _, full = script
            self.desc_var.set(get_description(full))
        else:
            self.desc_var.set("")

    # ── Lancement ─────────────────────────────────────────────────────────────
    def _launch(self):
        script = self._selected_script()
        if not script:
            messagebox.showwarning("Aucun script", "Sélectionne un script dans la liste.")
            return
        rel, full = script
        if not os.access(full, os.X_OK):
            os.chmod(full, 0o755)
        subprocess.Popen(
            ["gnome-terminal", "--title", friendly_name(rel), "--", "bash", full],
            cwd=os.path.dirname(full)
        )


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    app = Launcher()
    app.mainloop()
