"""
lora_lab.py — Interactive LoRa PHY Lab GUI
EE284A — Spring 2026

Imports all signal processing from lora_core.py (no DSP here).

Layout (18×11 in):
  ┌─────────────────────────────────────────────┐
  │  [0,:] Instantaneous Frequency (TX)         │  ← TX side
  ├─────────────────────┬─────────────────────┤
  │  [1,0] Dechirp FFT  │  [1,1] PER vs SNR   │  ← RX & channel
  ├─────────────────────┼─────────────────────┤
  │  [2,0] Pkt Structure│  [2,1] Info panel   │  ← link params
  └─────────────────────┴─────────────────────┘
  ── Sliders: SF | BW | CR | SNR | Flutter fd | Flutter rate ──
  ── Slider:  Dechirp sym # ───────────────────────────────────
  ── TextBox: Payload hex  [Update] [Run PER Sim] [Reset] ──────
  ── Status bar ────────────────────────────────────────────────

Usage:
    python lora_lab.py

Callback tiers (for performance):
  Expensive  (SF/BW/CR/payload/Update): rebuild TX chain + RX + redraw all
  Cheap      (SNR/flutter): reuse tx_result, re-run channel+RX, redraw dechirp+info
  PER button: runs simulate_per_curve (~5-30s), updates only PER panel
"""

from __future__ import annotations
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button, TextBox
from matplotlib.patches import Rectangle
from dataclasses import dataclass, field
from typing import Optional
import threading

import lora_core as lc

matplotlib.rcParams['toolbar'] = 'None'

# ---------------------------------------------------------------------------
# Default payload
# ---------------------------------------------------------------------------
DEFAULT_PAYLOAD_HEX = 'DEADBEEF01020304'

# ---------------------------------------------------------------------------
# LabState — avoids redundant recomputation
# ---------------------------------------------------------------------------

@dataclass
class LabState:
    params: Optional[lc.LoRaParams] = None
    tx_result: Optional[dict] = None
    rx_result: Optional[dict] = None
    per_result: Optional[tuple] = None   # (snr_db, per_nf, per_fl)
    payload: bytes = field(default_factory=lambda: bytes.fromhex(DEFAULT_PAYLOAD_HEX))
    per_running: bool = False


state = LabState()

# ---------------------------------------------------------------------------
# Figure and axes
# ---------------------------------------------------------------------------

fig = plt.figure(figsize=(18, 11))
fig.patch.set_facecolor('#1a1a2e')

# Reserve bottom area for widgets
gs_top = gridspec.GridSpec(3, 2,
                            top=0.96, bottom=0.32,
                            left=0.05, right=0.97,
                            hspace=0.42, wspace=0.28)

ax_ifreq = fig.add_subplot(gs_top[0, :])   # Instantaneous frequency (full top row)
ax_fft   = fig.add_subplot(gs_top[1, 0])   # Dechirp FFT
ax_per   = fig.add_subplot(gs_top[1, 1])   # PER vs SNR
ax_pkt   = fig.add_subplot(gs_top[2, 0])   # Packet structure
ax_info  = fig.add_subplot(gs_top[2, 1])   # Info panel

_DARK_BG = '#05050f'
_PANEL_BG = '#080d10'
for ax in [ax_ifreq, ax_fft, ax_per, ax_pkt, ax_info]:
    ax.set_facecolor(_PANEL_BG)
    ax.tick_params(colors='#ccccdd', labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor('#334466')

# ---------------------------------------------------------------------------
# Widget layout (bottom third)
# ---------------------------------------------------------------------------
WIDGET_LEFT = 0.05
WIDGET_RIGHT = 0.97
SLIDER_H = 0.022
SLIDER_GAP = 0.030
ROW1_Y = 0.285   # top slider row (SF, BW, CR)
ROW2_Y = ROW1_Y - SLIDER_GAP
ROW3_Y = ROW2_Y - SLIDER_GAP
BTN_ROW_Y = ROW3_Y - SLIDER_GAP - 0.005
STATUS_Y = BTN_ROW_Y - 0.028

# ─── Slider geometry helpers ────────────────────────────────────────────────
W_TOTAL = WIDGET_RIGHT - WIDGET_LEFT
SL_W3 = W_TOTAL / 3 - 0.015    # width for 3-per-row sliders
SL_W2 = W_TOTAL / 2 - 0.015    # width for 2-per-row

def _sl_ax(row_y, col, ncols):
    gap = (WIDGET_RIGHT - WIDGET_LEFT) / ncols
    x0 = WIDGET_LEFT + col * gap + 0.01
    w  = gap - 0.125
    return [x0, row_y, w, SLIDER_H]

_slider_kw = dict(color='#2a4a7f', track_color='#1d2b3a',
                  handle_style={'facecolor': '#4fc3f7', 'edgecolor': '#90caf9', 'size': 8})

ax_sl_sf  = fig.add_axes(_sl_ax(ROW1_Y, 0, 3))
ax_sl_bw  = fig.add_axes(_sl_ax(ROW1_Y, 1, 3))
ax_sl_cr  = fig.add_axes(_sl_ax(ROW1_Y, 2, 3))
ax_sl_snr = fig.add_axes(_sl_ax(ROW2_Y, 0, 3))
ax_sl_fd  = fig.add_axes(_sl_ax(ROW2_Y, 1, 3))
ax_sl_fr  = fig.add_axes(_sl_ax(ROW2_Y, 2, 3))

# Sym selector spans left 2/3 of ROW3
_gap3 = (WIDGET_RIGHT - WIDGET_LEFT) / 3
ax_sl_sym = fig.add_axes([WIDGET_LEFT + 0.01, ROW3_Y, 2 * _gap3 - 0.025, SLIDER_H])

sl_sf  = Slider(ax_sl_sf,  'SF',        7,    12,    valinit=11,  valstep=1,   **_slider_kw)
sl_bw  = Slider(ax_sl_bw,  'BW idx',    0,    2,     valinit=1,   valstep=1,   **_slider_kw)
sl_cr  = Slider(ax_sl_cr,  'CR offset', 1,    4,     valinit=1,   valstep=1,   **_slider_kw)
sl_snr = Slider(ax_sl_snr, 'SNR (dB)', -25,   20,    valinit=10,  valstep=0.5, **_slider_kw)
sl_fd  = Slider(ax_sl_fd,  'Flutter fd (Hz)', 0, 200, valinit=0,  valstep=5,   **_slider_kw)
sl_fr  = Slider(ax_sl_fr,  'Flutter fr (Hz)', 0.1, 100, valinit=2, valstep=0.1, **_slider_kw)
sl_sym = Slider(ax_sl_sym, 'Dechirp sym #', 0, 100, valinit=0, valstep=1, **_slider_kw)

# Label all sliders
for sl in [sl_sf, sl_bw, sl_cr, sl_snr, sl_fd, sl_fr, sl_sym]:
    sl.label.set_color('#aaccff')
    sl.label.set_fontsize(8)
    sl.valtext.set_color('#4fc3f7')
    sl.valtext.set_fontsize(8)

# Shift long labels left so they don't overwrite the valmin display
for sl in [sl_bw, sl_fd]:
    x, y = sl.label.get_position()
    sl.label.set_position((x - 0.07, y))

# Payload text box + buttons
TB_X = WIDGET_LEFT
TB_W = 0.22
TB_H = 0.028
ax_tb = fig.add_axes([TB_X, BTN_ROW_Y, TB_W, TB_H])
tb_payload = TextBox(ax_tb, 'Payload (hex) ', initial=DEFAULT_PAYLOAD_HEX,
                     color=_PANEL_BG, hovercolor='#1c3a5e',
                     label_pad=0.01)
tb_payload.label.set_color('#aaccff')
tb_payload.label.set_fontsize(8)
tb_payload.text_disp.set_color('#e0e0ff')

BTN_W = 0.10
BTN_H = 0.028
BTN_GAP = 0.015
_bx = TB_X + TB_W + BTN_GAP

ax_btn_update = fig.add_axes([_bx,                    BTN_ROW_Y, BTN_W, BTN_H])
ax_btn_per    = fig.add_axes([_bx + BTN_W + BTN_GAP,  BTN_ROW_Y, BTN_W, BTN_H])
ax_btn_reset  = fig.add_axes([_bx + 2*(BTN_W+BTN_GAP),BTN_ROW_Y, BTN_W, BTN_H])

_btn_kw = dict(color='#1e3a5f', hovercolor='#2a5080')
btn_update = Button(ax_btn_update, 'Update',      **_btn_kw)
btn_per    = Button(ax_btn_per,    'Run PER Sim', **_btn_kw)
btn_reset  = Button(ax_btn_reset,  'Reset',       **_btn_kw)
for btn in [btn_update, btn_per, btn_reset]:
    btn.label.set_color('#4fc3f7')
    btn.label.set_fontsize(9)

# Symbol display (read-only, right of Reset)
SYMDSP_X = _bx + 3*(BTN_W + BTN_GAP)
ax_sym_disp = fig.add_axes([SYMDSP_X, BTN_ROW_Y, WIDGET_RIGHT - SYMDSP_X, BTN_H])
ax_sym_disp.set_facecolor(_PANEL_BG)
ax_sym_disp.axis('off')
sym_disp_label = ax_sym_disp.text(
    0.002, 0.5, 'Syms (dec): \u2014',
    color='#4fc3f7', fontsize=7.5, va='center',
    transform=ax_sym_disp.transAxes,
    fontfamily='monospace',
)

# Banner image (centered in space below status bar)
_banner_img = plt.imread(
    os.path.join(os.path.dirname(__file__), 'Two_Lora_Transmissions.png'))
_fig_w, _fig_h = fig.get_size_inches()
_img_aspect = _banner_img.shape[1] / _banner_img.shape[0]
_banner_h = _fig_w / (_img_aspect * _fig_h)          # height in figure coords
_banner_y = (3*STATUS_Y - _banner_h) / 4               # vertically centred in [0, STATUS_Y]
ax_banner = fig.add_axes([0.0, _banner_y, 1.0, _banner_h])
ax_banner.imshow(_banner_img, aspect='auto')
ax_banner.axis('off')

# Status bar
ax_status = fig.add_axes([WIDGET_LEFT, STATUS_Y, W_TOTAL, 0.022])
ax_status.axis('off')
status_text = ax_status.text(0, 0.5, 'Ready.', color='#88aacc',
                              fontsize=8, va='center', transform=ax_status.transAxes)

# ---------------------------------------------------------------------------
# BW index → Hz mapping
# ---------------------------------------------------------------------------
BW_MAP = {0: 125e3, 1: 250e3, 2: 500e3}


def _get_params() -> lc.LoRaParams:
    bw = BW_MAP[int(sl_bw.val)]
    p = lc.LoRaParams(
        SF=int(sl_sf.val),
        BW=bw,
        CR=int(sl_cr.val),
        SNR_dB=float(sl_snr.val),
        flutter_fd=float(sl_fd.val),
        flutter_rate=float(sl_fr.val),
    )
    return p


def _update_sym_display():
    if state.tx_result is None:
        sym_disp_label.set_text('Syms (dec): \u2014')
        return
    syms = state.tx_result['gray_syms']
    sym_disp_label.set_text('Syms (dec): ' + '  '.join(str(int(s)) for s in syms))


def _set_status(msg: str, color: str = '#88aacc'):
    status_text.set_text(msg)
    status_text.set_color(color)
    fig.canvas.draw_idle()


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

_CMAP_CHIRP = 'plasma'   # match chirpfab.py


def _plot_spectrogram(ax, iq, p):
    ax.cla()
    ax.set_facecolor(_PANEL_BG)
    try:
        f, t, Sxx_dB = lc.compute_spectrogram(iq, p)
        vmin = np.percentile(Sxx_dB, 5)   # match pychirp.py clim style
        vmax = np.percentile(Sxx_dB, 99)
        ax.pcolormesh(t * 1e3, f / 1e3, Sxx_dB,
                      cmap='viridis', vmin=vmin, vmax=vmax,
                      shading='auto')
        ax.set_xlabel('Time (ms)', color='#aaccff', fontsize=8)
        ax.set_ylabel('Freq (kHz)', color='#aaccff', fontsize=8)
    except Exception as e:
        ax.text(0.5, 0.5, f'Error:\n{e}', color='red', ha='center', va='center',
                transform=ax.transAxes, fontsize=8)
    ax.set_title('Spectrogram (TX)', color='#cce0ff', fontsize=9, pad=3)
    ax.tick_params(colors='#aaaacc', labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor('#334466')


def _plot_inst_freq(ax, iq, p, sym_idx=0):
    """
    Plot instantaneous frequency colored by symbol value.
    Wrap-around discontinuities are broken with NaN so both ramp segments show.
    The data chirp at sym_idx is highlighted.
    """
    ax.cla()
    ax.set_facecolor(_PANEL_BG)

    def _seg(s, e):
        """Slice t/f_inst [s:e], insert NaN at wrap points so both sides show."""
        ts = t[s:e].astype(float) * 1e3        # ms
        fs = f_inst[s:e].astype(float) / 1e3   # kHz
        df = np.diff(fs)
        # A wrap jump is ~±BW kHz; threshold at 0.35*BW kHz
        wraps = np.where(np.abs(df) > p.BW / 1e3 * 0.35)[0] + 1
        for idx in reversed(wraps):
            ts = np.insert(ts, idx, np.nan)
            fs = np.insert(fs, idx, np.nan)
        return ts, fs

    try:
        t, f_inst = lc.compute_inst_freq(iq, p.fs)
        # Fold into [-BW/2, BW/2) — equivalent to real LoRa cyclic frequency wrap.
        # The simulation uses fs=2×BW so chirp_m can reach up to BW/2 + m·BW/N;
        # without folding the upper segment is entirely off screen.
        f_inst = ((f_inst + p.BW / 2) % p.BW) - p.BW / 2
        d = lc.lora_derived(p)
        n = d['n_samples']
        N = d['N']
        cmap = plt.get_cmap(_CMAP_CHIRP)

        if state.tx_result is not None:
            gray_syms = state.tx_result['gray_syms']
            preamble_end = p.preamble_len * n
            sync_end = preamble_end + 2 * n
            skip = sync_end

            # Draw data chirps; highlight the selected one
            for i, m in enumerate(gray_syms):
                s = skip + i * n
                if s >= len(t):
                    break
                e = min(s + n, len(t))
                ts, fs = _seg(s, e)
                color = cmap(float(m) / max(N - 1, 1))
                if i == sym_idx:
                    # White glow halo behind the line
                    ax.plot(ts, fs, color='white', linewidth=4.0, alpha=0.30, solid_capstyle='round')
                    ax.plot(ts, fs, color=color, linewidth=2.4, alpha=1.0)
                else:
                    ax.plot(ts, fs, color=color, linewidth=1.1, alpha=0.75)

            # Preamble upchirps
            ts, fs = _seg(0, preamble_end - 1)
            ax.plot(ts, fs, color='#88bbdd', linewidth=1.2, alpha=0.85, label='preamble')
            # Sync downchirps
            ts, fs = _seg(preamble_end, sync_end - 1)
            ax.plot(ts, fs, color='#ff9944', linewidth=1.4, alpha=1.0, label='sync (↓)')
            ax.legend(fontsize=7, loc='upper right',
                      labelcolor='#ccddff', facecolor=_PANEL_BG, edgecolor='#334466')
        else:
            ts, fs = _seg(0, len(t))
            ax.plot(ts, fs, color='#4488aa', linewidth=0.6)

        ax.set_ylim(-p.BW / 2e3 * 1.1, p.BW / 2e3 * 1.1)
        ax.set_xlabel('Time (ms)', color='#aaccff', fontsize=8)
        ax.set_ylabel('Inst. Freq (kHz)', color='#aaccff', fontsize=8)
    except Exception as e:
        ax.text(0.5, 0.5, f'Error:\n{e}', color='red', ha='center', va='center',
                transform=ax.transAxes, fontsize=8)
    ax.set_title('Instantaneous Frequency (TX)', color='#cce0ff', fontsize=9, pad=3)
    ax.tick_params(colors='#aaaacc', labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor('#334466')


def _plot_dechirp_fft(ax, rx_result, p, sym_idx=0):
    ax.cla()
    ax.set_facecolor(_PANEL_BG)
    try:
        if rx_result is None or not rx_result['dechirp_spectra']:
            ax.text(0.5, 0.5, 'No RX data', color='#88aacc',
                    ha='center', va='center', transform=ax.transAxes, fontsize=9)
        else:
            N = 2 ** p.SF
            spectra = rx_result['dechirp_spectra']
            detected = rx_result['detected_gray']
            cmap = plt.get_cmap(_CMAP_CHIRP)
            sym_idx = int(np.clip(sym_idx, 0, len(spectra) - 1))

            bins = np.arange(N)
            # Faint background: all symbols
            for i, spec in enumerate(spectra[:min(len(spectra), 20)]):
                ax.plot(bins, spec[:N], color='#1a3a5a', linewidth=0.4, alpha=0.5)

            # Highlight selected symbol
            spec_sel = spectra[sym_idx][:N]
            m_sel = detected[sym_idx] if sym_idx < len(detected) else 0
            c_raw = cmap(float(m_sel) / max(N - 1, 1))
            ax.plot(bins, spec_sel, color=(c_raw[0]*0.9, c_raw[1]*0.9, c_raw[2]*0.9),
                    linewidth=1.2, label=f'sym[{sym_idx}] → bin {m_sel}')
            ax.axvline(m_sel, color='#ff6b6b', linewidth=1.0, linestyle='--', alpha=0.8)
            ax.legend(fontsize=7, loc='upper right',
                      labelcolor='#ccddff', facecolor=_PANEL_BG, edgecolor='#334466')
            ax.set_xlabel('FFT bin', color='#aaccff', fontsize=8)
            ax.set_ylabel('Magnitude', color='#aaccff', fontsize=8)
            ax.set_xlim(0, N)
    except Exception as e:
        ax.text(0.5, 0.5, f'Error:\n{e}', color='red', ha='center', va='center',
                transform=ax.transAxes, fontsize=8)
    ax.set_title(f'Dechirp FFT (RX sym[{sym_idx}])', color='#cce0ff', fontsize=9, pad=3)
    ax.tick_params(colors='#aaaacc', labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor('#334466')


def _plot_per(ax, per_result, p):
    ax.cla()
    ax.set_facecolor(_PANEL_BG)
    ax.set_title('Packet Error Rate vs SNR', color='#cce0ff', fontsize=9, pad=3)
    if per_result is None:
        ax.text(0.5, 0.5, 'Press "Run PER Sim"', color='#88aacc',
                ha='center', va='center', transform=ax.transAxes, fontsize=9)
    else:
        snr_db, per_nf, per_fl = per_result
        ax.semilogy(snr_db, np.clip(per_nf, 1e-3, 1.0),
                    color='#4fc3f7', linewidth=1.5, label='No flutter')
        ax.semilogy(snr_db, np.clip(per_fl, 1e-3, 1.0),
                    color='#ff6b6b', linewidth=1.5, linestyle='--',
                    label=f'Flutter fd={p.flutter_fd:.0f}Hz')
        ax.axvline(p.SNR_dB, color='#ffcc44', linewidth=1.0,
                   linestyle=':', alpha=0.8, label='Current SNR')
        ax.set_ylim(5e-3, 1.5)
        ax.legend(fontsize=7, loc='upper right',
                  labelcolor='#ccddff', facecolor=_PANEL_BG, edgecolor='#334466')
        ax.set_xlabel('SNR (dB)', color='#aaccff', fontsize=8)
        ax.set_ylabel('PER', color='#aaccff', fontsize=8)
    ax.tick_params(colors='#aaaacc', labelsize=7)
    ax.grid(True, color='#223355', linewidth=0.5, alpha=0.6)
    for sp in ax.spines.values():
        sp.set_edgecolor('#334466')


def _plot_packet_structure(ax, p, n_payload_bytes):
    ax.cla()
    ax.set_facecolor(_PANEL_BG)
    try:
        parts = lc.packet_structure_info(p, n_payload_bytes)
        colors = ['#2e86ab', '#e84393', '#44bb66']
        x = 0
        for (label, n_syms), color in zip(parts, colors):
            rect = Rectangle((x, 0.2), n_syms, 0.6,
                              facecolor=color, edgecolor='#ffffff', linewidth=0.5, alpha=0.85)
            ax.add_patch(rect)
            ax.text(x + n_syms / 2, 0.5, f'{label}\n{n_syms} sym',
                    color='white', fontsize=7, ha='center', va='center',
                    fontweight='bold')
            x += n_syms
        ax.set_xlim(0, x)
        ax.set_ylim(0, 1)
        ax.axis('off')
    except Exception as e:
        ax.text(0.5, 0.5, f'Error:\n{e}', color='red', ha='center', va='center',
                transform=ax.transAxes, fontsize=8)
    ax.set_title('Packet Structure', color='#cce0ff', fontsize=9, pad=3)
    for sp in ax.spines.values():
        sp.set_edgecolor('#334466')


def _plot_info(ax, p, rx_result, tx_result):
    ax.cla()
    ax.set_facecolor(_PANEL_BG)
    ax.axis('off')
    d = lc.lora_derived(p)
    bw_str = f'{p.BW/1e3:.0f} kHz'
    cr_str = f'4/{4 + p.CR}'

    crc_str = '—'
    corrections_str = '—'
    payload_hex_str = '—'
    if rx_result is not None:
        crc_str = 'OK ✓' if rx_result['crc_ok'] else 'FAIL ✗'
        corrections_str = str(rx_result['n_corrected'])
        try:
            payload_hex_str = rx_result['payload'].hex().upper()
        except Exception:
            payload_hex_str = '—'

    # True throughput: payload bytes / total time-on-air (preamble + sync + data)
    if tx_result is not None:
        toa_s = len(tx_result['packet_iq']) / (2.0 * p.BW)
        n_payload_bytes = len(tx_result['payload'])
        rb_true = (n_payload_bytes * 8) / toa_s
        rb_true_str = f"{rb_true/1e3:.3f} kbit/s"
        toa_str = f"{toa_s*1e3:.2f} ms"
    else:
        rb_true_str = '—'
        toa_str = '—'

    lines = [
        ('SF',           f"{p.SF}"),
        ('BW',           bw_str),
        ('CR',           cr_str),
        ('SNR',          f"{p.SNR_dB:.1f} dB"),
        ('N (chips/sym)',f"{d['N']}"),
        ('Ts',           f"{d['Ts']*1e3:.3f} ms"),
        ('ToA',          toa_str),
        ('Rs',           f"{d['Rs']:.1f} sym/s"),
        ('Rb_raw',       f"{d['Rb_raw']/1e3:.2f} kbit/s"),
        ('Rb_eff',       f"{d['Rb_eff']/1e3:.2f} kbit/s"),
        ('Rb_true',      rb_true_str),
        ('Sensitivity',  f"{d['sensitivity_dBm']:.1f} dBm"),
        ('Flutter fd',   f"{p.flutter_fd:.0f} Hz"),
        ('Flutter fr',   f"{p.flutter_rate:.1f} Hz"),
        ('─'*16,         ''),
        ('CRC',          crc_str),
        ('FEC corrections', corrections_str),
        ('Decoded payload', payload_hex_str),
    ]

    y = 0.97
    dy = 0.057
    for key, val in lines:
        if key.startswith('─'):
            ax.plot([0.02, 0.98], [y, y], color='#334466', linewidth=0.5,
                    transform=ax.transAxes)
            y -= dy * 0.5
            continue
        ax.text(0.03, y, key + ':', color='#7799bb', fontsize=8,
                va='top', transform=ax.transAxes, fontweight='bold')
        ax.text(0.52, y, val, color='#e0e8ff', fontsize=8,
                va='top', transform=ax.transAxes)
        y -= dy

    ax.set_title('Link Parameters', color='#cce0ff', fontsize=9, pad=3)
    for sp in ax.spines.values():
        sp.set_edgecolor('#334466')


def _redraw_all():
    p = state.params
    tx = state.tx_result
    rx = state.rx_result

    if tx is None:
        return

    iq = tx['packet_iq']
    _plot_inst_freq(ax_ifreq, iq, p, int(sl_sym.val))
    _plot_dechirp_fft(ax_fft, rx, p, int(sl_sym.val))
    _plot_per(ax_per, state.per_result, p)
    _plot_packet_structure(ax_pkt, p, len(state.payload))
    _plot_info(ax_info, p, rx, tx)
    fig.canvas.draw_idle()


def _redraw_cheap():
    """Redraw only channel-dependent panels (dechirp FFT + info)."""
    p = state.params
    _plot_dechirp_fft(ax_fft, state.rx_result, p, int(sl_sym.val))
    _plot_per(ax_per, state.per_result, p)
    _plot_info(ax_info, p, state.rx_result, state.tx_result)
    fig.canvas.draw_idle()


# ---------------------------------------------------------------------------
# Core rebuild — expensive path
# ---------------------------------------------------------------------------

def _rebuild(payload: bytes, p: lc.LoRaParams):
    _set_status('Building TX packet…', '#ffcc44')
    try:
        tx = lc.build_packet(payload, p)
    except Exception as e:
        _set_status(f'TX error: {e}', '#ff6b6b')
        return

    _set_status('Running channel + RX…', '#ffcc44')
    try:
        rx_iq = lc.apply_channel(tx['packet_iq'].copy(), p)
        rx = lc.receive_packet(rx_iq, p, tx)
    except Exception as e:
        _set_status(f'RX error: {e}', '#ff6b6b')
        rx = None

    state.params = p
    state.tx_result = tx
    state.rx_result = rx
    state.payload = payload
    _update_sym_display()

    # Update sym slider range to match actual number of data symbols
    n_syms = tx.get('n_data_symbols', 1)
    sl_sym.valmax = max(n_syms - 1, 0)
    sl_sym.ax.set_xlim(0, max(n_syms - 1, 0))
    sl_sym.set_val(min(int(sl_sym.val), max(n_syms - 1, 0)))

    crc_str = ('CRC OK' if rx and rx['crc_ok'] else 'CRC FAIL') if rx else 'RX error'
    d = lc.lora_derived(p)
    _set_status(
        f"SF={p.SF}  BW={p.BW/1e3:.0f}kHz  CR=4/{4+p.CR}  "
        f"Ts={d['Ts']*1e3:.2f}ms  Rb_eff={d['Rb_eff']/1e3:.1f}kbit/s  {crc_str}",
        '#66ff99' if (rx and rx['crc_ok']) else '#ffaa44'
    )
    _redraw_all()


def _update_channel():
    """Cheap path: reuse TX, re-run channel+RX."""
    p = state.params
    if p is None or state.tx_result is None:
        return
    # Update SNR and flutter from sliders
    p.SNR_dB = float(sl_snr.val)
    p.flutter_fd = float(sl_fd.val)
    p.flutter_rate = float(sl_fr.val)
    _set_status('Re-running channel…', '#ffcc44')
    try:
        rx_iq = lc.apply_channel(state.tx_result['packet_iq'].copy(), p)
        rx = lc.receive_packet(rx_iq, p, state.tx_result)
    except Exception as e:
        _set_status(f'Channel error: {e}', '#ff6b6b')
        return
    state.rx_result = rx
    crc_str = 'CRC OK' if rx['crc_ok'] else 'CRC FAIL'
    _set_status(f"SNR={p.SNR_dB:.1f}dB  fd={p.flutter_fd:.0f}Hz  {crc_str}",
                '#66ff99' if rx['crc_ok'] else '#ffaa44')
    _redraw_cheap()


# ---------------------------------------------------------------------------
# Widget callbacks
# ---------------------------------------------------------------------------

def _on_expensive_change(_val=None):
    """Triggered when SF, BW, or CR changes — full rebuild required."""
    payload = state.payload  # keep existing payload
    p = _get_params()
    _rebuild(payload, p)


def _on_cheap_change(_val=None):
    """Triggered when SNR or flutter sliders change."""
    _update_channel()


def _on_update(_event=None):
    """Update button: re-parse payload hex and full rebuild."""
    raw = tb_payload.text.strip().replace(' ', '')
    try:
        payload = bytes.fromhex(raw)
        if len(payload) == 0:
            raise ValueError("Empty payload")
    except Exception as e:
        _set_status(f'Bad hex payload: {e}', '#ff6b6b')
        return
    _rebuild(payload, _get_params())


def _on_per(_event=None):
    """PER button: run simulation in background thread."""
    if state.per_running:
        _set_status('PER simulation already running…', '#ffaa44')
        return
    p = _get_params()
    payload = state.payload

    def _run():
        state.per_running = True
        _set_status('Running PER simulation (this may take 15–30 s)…', '#ffcc44')
        try:
            snr_r = np.arange(-25, 15, 1, dtype=float)
            result = lc.simulate_per_curve(p, payload, snr_range=snr_r, n_trials=200)
            state.per_result = result
            _set_status('PER simulation complete.', '#66ff99')
        except Exception as e:
            _set_status(f'PER error: {e}', '#ff6b6b')
        finally:
            state.per_running = False
        # Schedule PER plot update on the main thread via a one-shot timer
        def _update_on_main():
            _plot_per(ax_per, state.per_result, p)
            fig.canvas.draw_idle()
            timer.stop()
        timer = fig.canvas.new_timer(interval=0)
        timer.add_callback(_update_on_main)
        timer.start()

    t = threading.Thread(target=_run, daemon=True)
    t.start()


def _on_reset(_event=None):
    """Reset all sliders to defaults and rebuild."""
    sl_sf.set_val(11)
    sl_bw.set_val(1)
    sl_cr.set_val(1)
    sl_snr.set_val(10)
    sl_fd.set_val(0)
    sl_fr.set_val(2)
    tb_payload.set_val(DEFAULT_PAYLOAD_HEX)
    state.per_result = None
    _rebuild(bytes.fromhex(DEFAULT_PAYLOAD_HEX), _get_params())


def _on_sym_change(_val=None):
    """Redraw dechirp FFT and inst-freq highlight when sym slider moves."""
    if state.params is None or state.tx_result is None:
        return
    idx = int(sl_sym.val)
    _plot_dechirp_fft(ax_fft, state.rx_result, state.params, idx)
    _plot_inst_freq(ax_ifreq, state.tx_result['packet_iq'], state.params, idx)
    fig.canvas.draw_idle()


# Connect sliders
sl_sf.on_changed(_on_expensive_change)
sl_bw.on_changed(_on_expensive_change)
sl_cr.on_changed(_on_expensive_change)
sl_snr.on_changed(_on_cheap_change)
sl_fd.on_changed(_on_cheap_change)
sl_fr.on_changed(_on_cheap_change)
sl_sym.on_changed(_on_sym_change)

# Connect buttons
btn_update.on_clicked(_on_update)
btn_per.on_clicked(_on_per)
btn_reset.on_clicked(_on_reset)

# TextBox: Enter key triggers update
tb_payload.on_submit(lambda _: _on_update())

# ---------------------------------------------------------------------------
# BW slider label: show actual bandwidth
# ---------------------------------------------------------------------------

def _bw_label_update(val):
    bw = BW_MAP[int(val)]
    sl_bw.valtext.set_text(f'{bw/1e3:.0f}k')

sl_bw.on_changed(_bw_label_update)

# ---------------------------------------------------------------------------
# Initial render
# ---------------------------------------------------------------------------

fig.suptitle('EE284A — LoRa PHY Interactive Lab V2', color='#88bbff', fontsize=13,
             fontweight='bold', y=0.99)

# Trigger initial build
_rebuild(bytes.fromhex(DEFAULT_PAYLOAD_HEX), _get_params())

plt.show()
