#!/usr/bin/env python3
"""
system_monitor.py — Dashboard ressources utilisées par la stack de perception IMT.

Affiche uniquement les ressources consommées par les scripts Python du projet
(YOLO, cone_mapper, odom_tf, etc.) — le simulateur FSDS est exclu.

Valeurs affichées en unités absolues :
  RAM   → Go (sum RSS de tous les scripts Python)
  CPU   → cœurs équivalents (sum cpu_percent / 100)
  GPU   → Go VRAM utilisés par les processus Python (via nvidia-smi)
"""

import tkinter as tk
from tkinter import font as tkfont
import threading
import time
import subprocess
from collections import deque

import psutil

try:
    import pynvml
    pynvml.nvmlInit()
    N_GPU = pynvml.nvmlDeviceGetCount()
    GPU_OK = N_GPU > 0
except Exception:
    GPU_OK = False

# ── Thème ─────────────────────────────────────────────────────────────────────
BG      = '#0d1117'
CARD_BG = '#161b22'
BORDER  = '#30363d'
FG      = '#e6edf3'
GRAY    = '#6e7681'
GREEN   = '#3fb950'
YELLOW  = '#d29922'
RED     = '#f85149'
BLUE    = '#58a6ff'
PURPLE  = '#bc8cff'
ORANGE  = '#f0883e'

HISTORY = 60   # secondes d'historique

# Scripts Python de perception pure — ordre important : plus long en premier
# pour éviter qu'un sous-mot masque un mot plus long
PERCEPTION_PROCS = [
    'cone_mapper_lidar',   # avant cone_mapper !
    'yolo_lidar',
    'yolo_stereo',
    'cone_mapper',
    'odom_tf_publisher',
    'cone_evaluator',
]

# Middleware ROS2 (présent sur véhicule réel, overhead non négligeable)
ROS2_MIDDLEWARE = [
    'ros2',
    'fastrtps',
    'fastdds',
    'cyclonedds',
]

# Drivers capteurs réels (adapter selon matériel embarqué)
SENSOR_DRIVERS = [
    'velodyne',
    'ouster',
    'realsense',
    'usb_cam',
    'v4l2',
    'zed',
]

N_CORES = psutil.cpu_count(logical=True) or 1


def bytes_to_gb(b: int) -> float:
    return b / (1 << 30)


def pct_color(v: float) -> str:
    if v < 60:  return GREEN
    if v < 85:  return YELLOW
    return RED


def bytes_to_str(b: int) -> str:
    if b >= 1 << 30: return f"{b/(1<<30):.2f} Go"
    if b >= 1 << 20: return f"{b/(1<<20):.0f} Mo"
    return f"{b/(1<<10):.0f} Ko"


# ── GPU par processus (nvidia-smi) ─────────────────────────────────────────────
def _gpu_per_process():
    """Retourne {pid: vram_bytes} pour tous les processus utilisant le GPU."""
    result = {}
    try:
        out = subprocess.check_output(
            ['nvidia-smi', '--query-compute-apps=pid,used_memory',
             '--format=csv,noheader,nounits'],
            stderr=subprocess.DEVNULL, timeout=2
        ).decode()
        for line in out.strip().splitlines():
            parts = line.strip().split(',')
            if len(parts) == 2:
                pid  = int(parts[0].strip())
                mb   = int(parts[1].strip())
                result[pid] = mb * (1 << 20)   # Mo → octets
    except Exception:
        pass
    return result


# ── Collecteur de données ──────────────────────────────────────────────────────
class DataCollector(threading.Thread):

    def __init__(self):
        super().__init__(daemon=True)
        self.lock  = threading.Lock()
        self._data = {}
        self._hist_cpu     = deque(maxlen=HISTORY)
        self._hist_ram     = deque(maxlen=HISTORY)
        self._hist_gpu_mem = deque(maxlen=HISTORY)

    @property
    def data(self):
        with self.lock:
            return dict(self._data)

    def _scan_procs(self, gpu_pids, keywords, category):
        """Scanne les processus correspondant aux keywords et retourne la liste + totaux."""
        procs = []
        total_ram = 0; total_cpu = 0.0; total_vram = 0
        for proc in psutil.process_iter(
                ['pid', 'name', 'cmdline', 'cpu_percent', 'memory_info', 'status']):
            try:
                cmd = ' '.join(proc.info['cmdline'] or [])
                matched = None
                for kw in keywords:
                    if kw.lower() in cmd.lower():
                        matched = kw
                        break
                if not matched:
                    continue
                ram_b  = proc.info['memory_info'].rss
                cpu_p  = proc.info['cpu_percent']
                vram_b = gpu_pids.get(proc.info['pid'], 0)
                total_ram  += ram_b
                total_cpu  += cpu_p
                total_vram += vram_b
                procs.append({
                    'pid':       proc.info['pid'],
                    'name':      matched,
                    'category':  category,
                    'cpu_cores': cpu_p / 100.0,
                    'ram_gb':    bytes_to_gb(ram_b),
                    'vram_gb':   bytes_to_gb(vram_b),
                    'status':    proc.info['status'],
                })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return procs, total_ram, total_cpu, total_vram

    def run(self):
        psutil.cpu_percent(interval=None)
        time.sleep(0.5)

        while True:
            d = {}
            gpu_pids = _gpu_per_process()

            # ── Perception Python ─────────────────────────────────────────────
            procs_py, ram_py, cpu_py, vram_py = self._scan_procs(
                gpu_pids, PERCEPTION_PROCS, 'perception')

            # ── Middleware ROS2 ───────────────────────────────────────────────
            procs_ros, ram_ros, cpu_ros, vram_ros = self._scan_procs(
                gpu_pids, ROS2_MIDDLEWARE, 'middleware')

            # ── Drivers capteurs ──────────────────────────────────────────────
            procs_drv, ram_drv, cpu_drv, vram_drv = self._scan_procs(
                gpu_pids, SENSOR_DRIVERS, 'driver')

            # Tous les processus triés par RAM décroissante
            all_procs = sorted(procs_py + procs_ros + procs_drv,
                               key=lambda p: -p['ram_gb'])

            # Totaux par catégorie
            d['procs']               = all_procs
            d['procs_perception']    = procs_py

            d['ram_perception_gb']   = bytes_to_gb(ram_py)
            d['cpu_perception_cores']= cpu_py / 100.0
            d['vram_perception_gb']  = bytes_to_gb(vram_py)

            d['ram_middleware_gb']   = bytes_to_gb(ram_ros)
            d['cpu_middleware_cores']= cpu_ros / 100.0

            d['ram_drivers_gb']      = bytes_to_gb(ram_drv)
            d['cpu_drivers_cores']   = cpu_drv / 100.0

            # Total embarqué = perception + middleware + drivers
            d['total_ram_gb']        = bytes_to_gb(ram_py + ram_ros + ram_drv)
            d['total_cpu_cores']     = (cpu_py + cpu_ros + cpu_drv) / 100.0
            d['total_vram_gb']       = bytes_to_gb(vram_py + vram_ros + vram_drv)
            d['n_cores']             = N_CORES

            # GPU info global
            if GPU_OK:
                try:
                    h    = pynvml.nvmlDeviceGetHandleByIndex(0)
                    mem  = pynvml.nvmlDeviceGetMemoryInfo(h)
                    temp = pynvml.nvmlDeviceGetTemperature(h, pynvml.NVML_TEMPERATURE_GPU)
                    name = pynvml.nvmlDeviceGetName(h)
                    if isinstance(name, bytes): name = name.decode()
                    d['gpu_name']     = name
                    d['gpu_temp']     = temp
                    d['gpu_mem_total']= bytes_to_gb(mem.total)
                except Exception:
                    pass

            self._hist_cpu.append(d['total_cpu_cores'])
            self._hist_ram.append(d['total_ram_gb'])
            self._hist_gpu_mem.append(d['total_vram_gb'])

            d['hist_cpu']     = list(self._hist_cpu)
            d['hist_ram']     = list(self._hist_ram)
            d['hist_gpu_mem'] = list(self._hist_gpu_mem)

            with self.lock:
                self._data = d

            time.sleep(1.0)


# ── Widget barre ───────────────────────────────────────────────────────────────
class BarWidget:
    def __init__(self, parent, width=220, height=12):
        self.c = tk.Canvas(parent, width=width, height=height,
                           bg=CARD_BG, highlightthickness=0)
        self._w = width; self._h = height

    def pack(self, **kw): self.c.pack(**kw)

    def set(self, ratio, color):
        self.c.delete('all')
        self.c.create_rectangle(0, 0, self._w, self._h, fill=BORDER, outline='')
        w = max(1, int(self._w * min(ratio, 1.0)))
        self.c.create_rectangle(0, 0, w, self._h, fill=color, outline='')


class SparklineWidget:
    def __init__(self, parent, width=300, height=45, color=BLUE):
        self.c    = tk.Canvas(parent, width=width, height=height,
                              bg=CARD_BG, highlightthickness=0)
        self._w   = width; self._h = height; self._col = color

    def pack(self, **kw): self.c.pack(**kw)

    def set(self, values, max_val=1.0):
        self.c.delete('all')
        if len(values) < 2 or max_val <= 0:
            return
        n    = len(values)
        step = self._w / (HISTORY - 1)
        pts  = []
        for i, v in enumerate(values):
            x = (HISTORY - n + i) * step
            y = self._h - max(1, int(v / max_val * (self._h - 4)))
            pts.extend([x, y])
        self.c.create_line(*pts, fill=self._col, width=2, smooth=True)


# ── Dashboard ──────────────────────────────────────────────────────────────────
class Dashboard:

    def __init__(self, root, collector):
        self.root      = root
        self.collector = collector

        root.title("Ressources — Stack Perception IMT")
        root.configure(bg=BG)
        root.resizable(False, False)

        self._tf  = tkfont.Font(family='Helvetica', size=12, weight='bold')
        self._big = tkfont.Font(family='Helvetica', size=28, weight='bold')
        self._med = tkfont.Font(family='Helvetica', size=10)
        self._sm  = tkfont.Font(family='Helvetica', size=8)
        self._mono= tkfont.Font(family='Courier',   size=9)

        self._max_ram  = 0.0
        self._max_cpu  = 0.0
        self._max_vram = 0.0

        self._build()
        self._update()

    def _card(self, parent, title, row, col, colspan=1):
        f = tk.Frame(parent, bg=CARD_BG,
                     highlightbackground=BORDER, highlightthickness=1)
        f.grid(row=row, column=col, columnspan=colspan,
               padx=6, pady=6, sticky='nsew')
        tk.Label(f, text=title, bg=CARD_BG, fg=BLUE,
                 font=self._tf).pack(anchor='w', padx=10, pady=(8, 2))
        tk.Frame(f, bg=BORDER, height=1).pack(fill='x', padx=10, pady=(0, 6))
        return f

    def _build(self):
        root = self.root
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.columnconfigure(2, weight=1)

        # Sous-titre
        tk.Label(root,
                 text="Consommation totale du système embarqué de perception (FSDS exclu)",
                 bg=BG, fg=GRAY, font=self._sm).grid(
                     row=0, column=0, columnspan=3, pady=(10, 0))

        # ── RAM ───────────────────────────────────────────────────────────────
        cf = self._card(root, "RAM  (Go)  — total embarqué", 1, 0)
        self._ram_val = tk.Label(cf, text="—", bg=CARD_BG, fg=GREEN, font=self._big)
        self._ram_val.pack()
        tk.Label(cf, text="Go utilisés (perception + ROS2 + drivers)",
                 bg=CARD_BG, fg=GRAY, font=self._sm).pack()
        self._ram_bar = BarWidget(cf)
        self._ram_bar.pack(padx=10, pady=4)
        self._ram_sub = tk.Label(cf, text="", bg=CARD_BG, fg=GRAY, font=self._sm)
        self._ram_sub.pack(pady=(0, 2))
        self._ram_max = tk.Label(cf, text="max : —", bg=CARD_BG, fg=YELLOW, font=self._sm)
        self._ram_max.pack(pady=(0, 4))
        self._ram_spark = SparklineWidget(cf, color=PURPLE)
        self._ram_spark.pack(padx=10, pady=(0, 8))

        # ── CPU ───────────────────────────────────────────────────────────────
        cf2 = self._card(root, "CPU  (cœurs)  — total embarqué", 1, 1)
        self._cpu_val = tk.Label(cf2, text="—", bg=CARD_BG, fg=GREEN, font=self._big)
        self._cpu_val.pack()
        tk.Label(cf2, text="cœurs équivalents (perception + ROS2 + drivers)",
                 bg=CARD_BG, fg=GRAY, font=self._sm).pack()
        self._cpu_bar = BarWidget(cf2)
        self._cpu_bar.pack(padx=10, pady=4)
        self._cpu_sub = tk.Label(cf2, text="", bg=CARD_BG, fg=GRAY, font=self._sm)
        self._cpu_sub.pack(pady=(0, 2))
        self._cpu_max = tk.Label(cf2, text="max : —", bg=CARD_BG, fg=YELLOW, font=self._sm)
        self._cpu_max.pack(pady=(0, 4))
        self._cpu_spark = SparklineWidget(cf2, color=BLUE)
        self._cpu_spark.pack(padx=10, pady=(0, 8))

        # ── GPU ───────────────────────────────────────────────────────────────
        cf3 = self._card(root, "GPU VRAM  (Go)  — total embarqué", 1, 2)
        self._gpu_val = tk.Label(cf3,
                                 text="N/A" if not GPU_OK else "—",
                                 bg=CARD_BG, fg=GRAY if not GPU_OK else GREEN,
                                 font=self._big)
        self._gpu_val.pack()
        tk.Label(cf3, text="Go VRAM (perception + ROS2 + drivers)",
                 bg=CARD_BG, fg=GRAY, font=self._sm).pack()
        self._gpu_bar = BarWidget(cf3)
        self._gpu_bar.pack(padx=10, pady=4)
        self._gpu_sub = tk.Label(cf3, text="", bg=CARD_BG, fg=GRAY, font=self._sm)
        self._gpu_sub.pack(pady=(0, 2))
        self._gpu_max = tk.Label(cf3, text="max : —", bg=CARD_BG, fg=YELLOW, font=self._sm)
        self._gpu_max.pack(pady=(0, 4))
        self._gpu_spark = SparklineWidget(cf3, color=ORANGE)
        self._gpu_spark.pack(padx=10, pady=(0, 8))

        # ── Tableau par catégorie ─────────────────────────────────────────────
        bf = self._card(root, "Répartition par catégorie", 2, 0, colspan=3)
        bhdr = tk.Frame(bf, bg=CARD_BG); bhdr.pack(fill='x', padx=10)
        for txt, w in [("Catégorie", 22), ("RAM (Go)", 12), ("CPU (cœurs)", 14),
                       ("VRAM (Go)", 12), ("Processus actifs", 18)]:
            tk.Label(bhdr, text=txt, width=w, bg=CARD_BG, fg=GRAY,
                     font=self._sm, anchor='w').pack(side='left')
        tk.Frame(bf, bg=BORDER, height=1).pack(fill='x', padx=10, pady=2)

        self._cat_rows = {}
        cat_defs = [
            ('perception', 'Scripts perception Python', PURPLE),
            ('middleware', 'Middleware ROS2',            BLUE),
            ('drivers',   'Drivers capteurs',           ORANGE),
            ('total',     '▶  TOTAL embarqué',          GREEN),
        ]
        for key, label, color in cat_defs:
            row = tk.Frame(bf, bg=CARD_BG); row.pack(fill='x', padx=10, pady=1)
            cols = {}
            lbl = tk.Label(row, text=label, width=22, bg=CARD_BG, fg=color,
                           font=self._mono, anchor='w')
            lbl.pack(side='left')
            for col_key, w in [('ram', 12), ('cpu', 14), ('vram', 12), ('procs', 18)]:
                l = tk.Label(row, text='—', width=w, bg=CARD_BG, fg=FG,
                             font=self._mono, anchor='w')
                l.pack(side='left')
                cols[col_key] = l
            self._cat_rows[key] = cols
        tk.Frame(bf, bg=BORDER, height=1).pack(fill='x', padx=10, pady=(4, 2))

        # ── Tableau processus ─────────────────────────────────────────────────
        pf = self._card(root, "Détail par script", 3, 0, colspan=3)
        hdr = tk.Frame(pf, bg=CARD_BG); hdr.pack(fill='x', padx=10)
        for txt, w in [("Script", 18), ("Catégorie", 12), ("PID", 7), ("RAM (Go)", 10),
                       ("CPU (cœurs)", 12), ("VRAM (Go)", 10), ("Statut", 9)]:
            tk.Label(hdr, text=txt, width=w, bg=CARD_BG, fg=GRAY,
                     font=self._sm, anchor='w').pack(side='left')
        self._proc_frame = tk.Frame(pf, bg=CARD_BG)
        self._proc_frame.pack(fill='x', padx=10, pady=(2, 10))
        self._proc_rows = []

    def _ensure_rows(self, n):
        while len(self._proc_rows) < n:
            r = tk.Frame(self._proc_frame, bg=CARD_BG)
            r.pack(fill='x', pady=1)
            cols = {}
            for key, w in [('name', 18), ('cat', 12), ('pid', 7), ('ram', 10),
                            ('cpu', 12), ('vram', 10), ('status', 9)]:
                lbl = tk.Label(r, text='', width=w, bg=CARD_BG, fg=FG,
                               font=self._mono, anchor='w')
                lbl.pack(side='left')
                cols[key] = lbl
            self._proc_rows.append(cols)

    def _col(self, v, max_v):
        if max_v == 0: return GREEN
        r = v / max_v
        if r < 0.5:  return GREEN
        if r < 0.75: return YELLOW
        return RED

    def _update(self):
        d = self.collector.data
        if d:
            ram   = d.get('total_ram_gb', 0)
            cpu   = d.get('total_cpu_cores', 0)
            vram  = d.get('total_vram_gb', 0)
            ncores= d.get('n_cores', 1)
            gtotal= d.get('gpu_mem_total', 8.0) or 8.0
            vm    = psutil.virtual_memory()
            ram_t = vm.total / (1 << 30)

            # Mise à jour des max
            self._max_ram  = max(self._max_ram,  ram)
            self._max_cpu  = max(self._max_cpu,  cpu)
            self._max_vram = max(self._max_vram, vram)

            # RAM
            col = self._col(ram, ram_t)
            self._ram_val.config(text=f"{ram:.2f}", fg=col)
            self._ram_bar.set(ram / ram_t, col)
            self._ram_sub.config(text=f"sur {ram_t:.1f} Go total système")
            self._ram_max.config(text=f"max : {self._max_ram:.2f} Go")
            self._ram_spark.set(d.get('hist_ram', []), max_val=max(ram_t, 0.1))

            # CPU
            col = self._col(cpu, ncores)
            self._cpu_val.config(text=f"{cpu:.2f}", fg=col)
            self._cpu_bar.set(cpu / ncores, col)
            self._cpu_sub.config(text=f"sur {ncores} cœurs logiques total")
            self._cpu_max.config(text=f"max : {self._max_cpu:.2f} cœurs")
            self._cpu_spark.set(d.get('hist_cpu', []), max_val=max(ncores, 0.1))

            # GPU
            if GPU_OK:
                col = self._col(vram, gtotal)
                self._gpu_val.config(text=f"{vram:.2f}", fg=col)
                self._gpu_bar.set(vram / gtotal, col)
                temp = d.get('gpu_temp', '?')
                name = d.get('gpu_name', 'GPU')
                self._gpu_sub.config(
                    text=f"{name} | {temp}°C | total VRAM {gtotal:.1f} Go")
                self._gpu_max.config(text=f"max : {self._max_vram:.2f} Go")
                self._gpu_spark.set(d.get('hist_gpu_mem', []),
                                    max_val=max(gtotal, 0.1))

            # ── Tableau par catégorie ─────────────────────────────────────────
            procs = d.get('procs', [])
            n_perc = sum(1 for p in procs if p['category'] == 'perception')
            n_mid  = sum(1 for p in procs if p['category'] == 'middleware')
            n_drv  = sum(1 for p in procs if p['category'] == 'driver')

            ram_py  = d.get('ram_perception_gb',  0)
            cpu_py  = d.get('cpu_perception_cores',0)
            vram_py = d.get('vram_perception_gb', 0)
            ram_ros = d.get('ram_middleware_gb',  0)
            cpu_ros = d.get('cpu_middleware_cores',0)
            ram_drv = d.get('ram_drivers_gb',     0)
            cpu_drv = d.get('cpu_drivers_cores',  0)

            def _fmt_vram(v): return f"{v:.3f}" if v > 0 else "—"

            self._cat_rows['perception']['ram'].config(  text=f"{ram_py:.3f}",  fg=PURPLE)
            self._cat_rows['perception']['cpu'].config(  text=f"{cpu_py:.3f}",  fg=PURPLE)
            self._cat_rows['perception']['vram'].config( text=_fmt_vram(vram_py),fg=PURPLE)
            self._cat_rows['perception']['procs'].config(text=f"{n_perc} actif(s)", fg=PURPLE)

            self._cat_rows['middleware']['ram'].config(  text=f"{ram_ros:.3f}", fg=BLUE)
            self._cat_rows['middleware']['cpu'].config(  text=f"{cpu_ros:.3f}", fg=BLUE)
            self._cat_rows['middleware']['vram'].config( text="—",              fg=GRAY)
            self._cat_rows['middleware']['procs'].config(text=f"{n_mid} actif(s)", fg=BLUE)

            self._cat_rows['drivers']['ram'].config(  text=f"{ram_drv:.3f}", fg=ORANGE)
            self._cat_rows['drivers']['cpu'].config(  text=f"{cpu_drv:.3f}", fg=ORANGE)
            self._cat_rows['drivers']['vram'].config( text="—",              fg=GRAY)
            self._cat_rows['drivers']['procs'].config(text=f"{n_drv} actif(s)", fg=ORANGE)

            self._cat_rows['total']['ram'].config(  text=f"{ram:.3f}",       fg=GREEN)
            self._cat_rows['total']['cpu'].config(  text=f"{cpu:.3f}",       fg=GREEN)
            self._cat_rows['total']['vram'].config( text=_fmt_vram(vram),    fg=GREEN)
            self._cat_rows['total']['procs'].config(text=f"{n_perc+n_mid+n_drv} actif(s)", fg=GREEN)

            # ── Tableau détail processus ──────────────────────────────────────
            CAT_COLOR = {'perception': PURPLE, 'middleware': BLUE, 'driver': ORANGE}
            self._ensure_rows(len(procs))
            for i, row in enumerate(self._proc_rows):
                if i < len(procs):
                    p    = procs[i]
                    cat  = p['category']
                    ccol = CAT_COLOR.get(cat, FG)
                    row['name'].config(text=p['name'][:17],    fg=FG)
                    row['cat'].config( text=cat[:11],          fg=ccol)
                    row['pid'].config( text=str(p['pid']),     fg=GRAY)
                    row['ram'].config( text=f"{p['ram_gb']:.3f}",
                                       fg=self._col(p['ram_gb'], ram_t))
                    row['cpu'].config( text=f"{p['cpu_cores']:.3f}",
                                       fg=self._col(p['cpu_cores'], ncores))
                    row['vram'].config(text=f"{p['vram_gb']:.3f}" if p['vram_gb'] > 0 else "—",
                                       fg=ORANGE if p['vram_gb'] > 0 else GRAY)
                    row['status'].config(
                        text=p['status'],
                        fg=GREEN if p['status'] == 'running' else GRAY)
                else:
                    for lbl in row.values():
                        lbl.config(text='', fg=GRAY)

        self.root.after(1000, self._update)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    collector = DataCollector()
    collector.start()
    root = tk.Tk()
    Dashboard(root, collector)
    root.mainloop()


if __name__ == '__main__':
    main()
