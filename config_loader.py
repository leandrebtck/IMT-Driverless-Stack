"""
config_loader.py — Chargeur de configuration centralisée.

Charge config.yaml depuis la racine du repo et expose un objet CFG
utilisable par tous les scripts Python de la stack.

Usage dans n'importe quel script :
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from config_loader import CFG

    baseline = CFG['camera']['baseline_m']
    topic    = CFG['topics']['lidar']
"""

import os
import yaml

_ROOT = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_ROOT, 'config.yaml')

with open(_CONFIG_PATH, 'r') as _f:
    CFG = yaml.safe_load(_f)
