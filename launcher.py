#!/usr/bin/env python3
"""
launcher.py — Interface graphique de lancement des scripts IMT Driverless.
Lance ce fichier depuis la racine du repo :  python3 launcher.py
"""

import os
import subprocess
import tkinter as tk
from tkinter import messagebox

REPO_ROOT   = os.path.dirname(os.path.abspath(__file__))
EXCLUDE_DIRS = {'ros_workspace', '.git'}

# ── Palette couleurs ────────────────────────────────────────────────────────
BG       = "#1e1e2e"
BG2      = "#313244"
FG       = "#cdd6f4"
FG_DIM   = "#a6adc8"
ACCENT   = "#89b4fa"
ACCENT2  = "#b4befe"
GREEN    = "#a6e3a1"
RED      = "#f38ba8"


# ── Utilitaires ─────────────────────────────────────────────────────────────
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


# ── Application ─────────────────────────────────────────────────────────────
class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IMT Driverless — Launcher")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(560, 440)
        self._scripts = find_scripts()
        self._build_ui()
        self._populate()

    # ── UI ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Titre
        hdr = tk.Frame(self, bg=BG, pady=14)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="IMT Driverless Stack",
                 font=("Helvetica", 17, "bold"), bg=BG, fg=ACCENT).pack()
        tk.Label(hdr, text="Double-clic ou bouton Lancer pour exécuter un script",
                 font=("Helvetica", 9), bg=BG, fg=FG_DIM).pack()

        # Séparateur
        tk.Frame(self, bg=BG2, height=1).pack(fill=tk.X)

        # Liste + scrollbar
        list_frame = tk.Frame(self, bg=BG)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=10)

        sb = tk.Scrollbar(list_frame, bg=BG2, troughcolor=BG)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        self.listbox = tk.Listbox(
            list_frame,
            font=("Monospace", 11),
            bg=BG2, fg=FG,
            selectbackground=ACCENT, selectforeground=BG,
            activestyle="none",
            bd=0, highlightthickness=0,
            yscrollcommand=sb.set
        )
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.config(command=self.listbox.yview)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)
        self.listbox.bind("<Double-Button-1>", lambda e: self._launch())

        # Description
        desc_frame = tk.Frame(self, bg=BG, padx=16)
        desc_frame.pack(fill=tk.X)
        tk.Label(desc_frame, text="Description :", font=("Helvetica", 9, "bold"),
                 bg=BG, fg=FG_DIM).pack(anchor="w")
        self.desc_var = tk.StringVar(value="Sélectionne un script…")
        tk.Label(desc_frame, textvariable=self.desc_var,
                 font=("Helvetica", 9, "italic"), bg=BG, fg=FG_DIM,
                 wraplength=520, justify="left").pack(anchor="w")

        # Boutons
        btn_frame = tk.Frame(self, bg=BG, pady=12)
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

    # ── Données ─────────────────────────────────────────────────────────────
    def _populate(self):
        self.listbox.delete(0, tk.END)
        current_cat = None
        for rel, full in self._scripts:
            cat = category(rel)
            if cat != current_cat:
                self.listbox.insert(tk.END, f"── {cat} ──")
                self.listbox.itemconfig(tk.END, fg=ACCENT2, selectbackground=BG2,
                                        selectforeground=ACCENT2)
                current_cat = cat
            self.listbox.insert(tk.END, f"   {friendly_name(rel)}")

    def _refresh(self):
        self._scripts = find_scripts()
        self._populate()
        self.desc_var.set("Sélectionne un script…")

    # ── Sélection ───────────────────────────────────────────────────────────
    def _selected_script(self):
        """Retourne (rel, full) du script sélectionné, ou None si c'est un header."""
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

    # ── Lancement ───────────────────────────────────────────────────────────
    def _launch(self):
        script = self._selected_script()
        if not script:
            messagebox.showwarning("Aucun script",
                                   "Sélectionne un script dans la liste.")
            return
        rel, full = script
        if not os.access(full, os.X_OK):
            os.chmod(full, 0o755)
        subprocess.Popen(
            ["gnome-terminal", "--title", friendly_name(rel),
             "--", "bash", full],
            cwd=os.path.dirname(full)
        )


# ── Main ────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    app = Launcher()
    app.mainloop()
