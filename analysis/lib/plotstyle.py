#!/usr/bin/env python3
"""플롯 공통 스타일. 라벨이 한국어라 폰트 등록이 필수다.

matplotlib 의 폰트 캐시는 fontconfig 가 아는 나눔폰트를 못 볼 때가 있다 — 이름만 지정하면
**조용히 DejaVu 로 폴백해 두부(□)** 가 되므로 경로로 직접 등록한다.
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PALETTE = ["#2a78d6", "#1baf7a", "#eda100", "#4a3aa7", "#e34948"]
TEXT2, GRID = "#5f5e56", "#e5e4dc"

# 표를 monospace 로 그리면 기본 DejaVu Sans Mono 에는 한글이 없어 두부가 된다 —
# 한글이 되는 고정폭(NanumGothicCoding)을 같이 등록하고 MONO 로 노출한다.
for _p in ('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
           '/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf',
           '/usr/share/fonts/truetype/nanum/NanumGothicCoding.ttf'):
    if os.path.exists(_p):
        matplotlib.font_manager.fontManager.addfont(_p)

plt.rcParams.update({'font.family': ['NanumGothic', 'NanumBarunGothic', 'DejaVu Sans'],
                     'axes.unicode_minus': False,
                     'figure.facecolor': 'white', 'axes.facecolor': 'white',
                     'axes.edgecolor': GRID, 'axes.grid': True, 'grid.color': GRID,
                     'grid.linewidth': 0.6, 'font.size': 9, 'axes.spines.top': False,
                     'axes.spines.right': False, 'legend.frameon': False})

MONO = (['NanumGothicCoding', 'DejaVu Sans Mono']
        if os.path.exists('/usr/share/fonts/truetype/nanum/NanumGothicCoding.ttf')
        else ['DejaVu Sans Mono'])
