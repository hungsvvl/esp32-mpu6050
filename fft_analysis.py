"""
fft_analysis.py  —  Phân Tích Nhịp Thở Realtime từ ESP32 + MPU6050
===================================================================
Sử dụng:
  python fft_analysis.py --port COM7
  python fft_analysis.py --port COM7 --baud 115200

ESP32 cần gửi từng dòng dạng  "Z:<float>\r\n"  tại 50 Hz.
  Ví dụ:  Z:9.7412  (gia tốc trục Z, đơn vị m/s²)
  Tương thích ngược với firmware đầy đủ (field M: hoặc Signal:).

Lưu kết quả:
  · Nhấn  [Luu Ket Qua (PNG + CSV)]  → lưu ngay, chương trình tiếp tục
  · Đóng cửa sổ                      → tự động lưu rồi thoát
"""

import argparse
import csv
import threading
import time
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as mplanim
from matplotlib.widgets import Button
from scipy.signal import butter, filtfilt, find_peaks, welch
from scipy.fft import rfft, rfftfreq

# ══════════════════════════════════ CONFIG ════════════════════════════════════
FS          = 50          # Tần số lấy mẫu (Hz) — khớp với ESP32 delay(20ms)
BUF_S       = 60          # Giây lưu trong rolling buffer (sliding window)
DISP_S      = 60          # Giây hiển thị trên chart tín hiệu (toàn bộ 60s)
UPDATE_MS   = 250         # Chu kỳ refresh animation (ms)
FFT_N       = 512         # Cửa sổ FFT = 512 / 50 = 10.24 giây
BPF_LO      = 0.10        # Hz — ngưỡng thấp bandpass (6 BPM)
BPF_HI      = 0.70        # Hz — ngưỡng cao bandpass (42 BPM)
_N          = FS * BUF_S  # 3 000 mẫu tối đa trong buffer

# ══════════════════════════════════ COLORS ════════════════════════════════════
C_BG     = '#0d1117'
C_PANE   = '#0d1117'
C_BORDER = '#30363d'
C_DIM    = '#484f58'
C_TEXT   = '#8b949e'
C_WHITE  = '#e6edf3'
C_BLUE   = '#58a6ff'
C_GREEN  = '#7ee787'
C_ORANGE = '#f0883e'
C_RED    = '#ff7b72'
C_PURPLE = '#a371f7'
C_YELLOW = '#e3b341'
C_CYAN   = '#39d0d8'

STATUS_MAP = {
    0: ('Khoi dong...',              C_TEXT),
    1: ('BINH THUONG',               C_GREEN),
    2: ('NHIP NHANH  (Tachypnea)',   C_ORANGE),
    3: ('NHIP CHAM   (Bradypnea)',   C_BLUE),
    4: ('NGUNG THO!  (Apnea)',       C_RED),
}

# ══════════════════════════════════ ROLLING BUFFER ════════════════════════════
_lock    = threading.Lock()
_raw     = np.zeros(_N, dtype=np.float32)
_t_sec   = np.zeros(_N, dtype=np.float32)
_wi      = 0      # write-index vong tron
_nf      = 0      # so mau da nap
_running = True   # flag dung toan cuc


def _write(val: float, t: float):
    global _wi, _nf
    with _lock:
        _raw[_wi]   = val
        _t_sec[_wi] = t
        _wi = (_wi + 1) % _N
        if _nf < _N:
            _nf += 1


def _read_all():
    """Tra ve (times, raw) theo thu tu thoi gian (mang lien tuc)."""
    with _lock:
        n  = _nf
        wi = _wi
        raw = _raw.copy()
        t   = _t_sec.copy()
    if n == 0:
        return np.empty(0, np.float32), np.empty(0, np.float32)
    if n < _N:
        return t[:n], raw[:n]
    return np.roll(t, -wi), np.roll(raw, -wi)


# ══════════════════════════════════ BANDPASS FILTER ═══════════════════════════
_nyq        = 0.5 * FS
_bpf_b, _bpf_a = butter(4, [BPF_LO / _nyq, BPF_HI / _nyq], btype='band')
_MIN_LEN    = 27   # filtfilt can toi thieu ~27 mau (order=4)


# ══════════════════════════════════ SERIAL STATS (thread-safe) ══════════════
_pkt_ok  = [0]   # goi nhan thanh cong
_pkt_err = [0]   # goi bi loi / rac


# ══════════════════════════════════ DATA PRODUCER — SERIAL ONLY ══════════════

def _parse_line(raw_bytes: bytes):
    """
    Giai ma mot goi tin tu ESP32. Tra ve float hoac None.
    KHONG nem ngoai le ra ngoai — moi loi duoc xu ly noi bo.

    Ho tro 3 dinh dang:
      [Compact]  Z:9.7412          (sketch esp32_sender.ino)
      [Full v2]  S:23.1,L:23.0,M:22.9,BP:14.5,BF:15.2,A:0.23,ST:1
      [Full v1]  Signal:0.12,Threshold:0.2,BPM(x0.01):0.15,Apnea_Warn:0
    """
    # --- buoc 1: giai ma bytes -> chuoi ---
    try:
        line = raw_bytes.decode('utf-8', errors='replace').strip()
    except Exception:
        return None

    if not line:
        return None

    # Bo qua dong debug (bat dau bang '[' hoac chua chu cai la chu)
    if line.startswith('[') or line.startswith('=') or line.startswith('+') or line.startswith('|'):
        return None

    # --- buoc 2: tach token ---
    try:
        tokens = line.split(',')
    except Exception:
        return None

    # --- buoc 3: tim gia tri trong cac truong uu tien ---
    # Thu tu uu tien: Z (compact), M (v2 smoothed), Signal (v1)
    priority = ('Z', 'M', 'Signal')
    kv = {}
    for tok in tokens:
        try:
            if ':' not in tok:
                continue
            k, _, v = tok.partition(':')
            kv[k.strip()] = v.strip()
        except Exception:
            continue   # bo qua token rac

    for key in priority:
        if key in kv:
            try:
                return float(kv[key])
            except (ValueError, OverflowError):
                continue  # gia tri khong hop le, thu key tiep theo

    return None


def _serial_worker(port: str, baud: int = 115200):
    """
    Thread doc serial lien tuc, KHONG bao gio crash.
    Moi loi (mat goi, rac du lieu, mat ket noi) deu duoc xu ly,
    ghi log ngan gon va tiep tuc chay.
    """
    import serial
    import serial.serialutil

    sn          = 0
    reconnect_n = 0
    ser         = None

    while _running:
        # ── Thiet lap ket noi ────────────────────────────────────────────────
        try:
            ser = serial.Serial(
                port, baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            ser.reset_input_buffer()
            reconnect_n += 1
            ts = datetime.now().strftime('%H:%M:%S')
            print(f'[{ts}][Serial] Ket noi #{reconnect_n}: {port} @ {baud} baud OK')
        except serial.serialutil.SerialException as e:
            ts = datetime.now().strftime('%H:%M:%S')
            print(f'[{ts}][Serial] Khong mo duoc cong {port}: {e}')
            print(f'          --> Thu lai sau 3s...')
            time.sleep(3)
            continue
        except Exception as e:
            print(f'[Serial] Loi khong xac dinh khi mo cong: {e}')
            time.sleep(3)
            continue

        # ── Vong doc chinh ───────────────────────────────────────────────────
        consecutive_err = 0
        try:
            while _running:
                # -- Doc 1 dong bytes --
                try:
                    raw_bytes = ser.readline()   # tra ve b'' neu timeout
                except serial.serialutil.SerialException as e:
                    # Mat ket noi vat ly (rut USB...)
                    ts = datetime.now().strftime('%H:%M:%S')
                    print(f'[{ts}][Serial] Mat ket noi: {e}')
                    break
                except OSError as e:
                    print(f'[Serial] OSError khi doc: {e}')
                    break
                except Exception as e:
                    print(f'[Serial] Loi doc bat thuong: {e}')
                    consecutive_err += 1
                    if consecutive_err > 20:
                        print('[Serial] Qua nhieu loi lien tiep, dong lai cong...')
                        break
                    continue

                if not raw_bytes:
                    # Timeout (2s khong co du lieu)
                    ts = datetime.now().strftime('%H:%M:%S')
                    print(f'[{ts}][Serial] Timeout — khong nhan duoc du lieu trong 2s')
                    consecutive_err += 1
                    if consecutive_err > 5:
                        print('[Serial] Qua 5 lan timeout lien tiep — thu ket noi lai...')
                        break
                    continue

                consecutive_err = 0   # reset khi nhan duoc bytes

                # -- Giai ma va lay gia tri --
                val = _parse_line(raw_bytes)
                if val is None:
                    with _lock:
                        _pkt_err[0] += 1
                    continue

                # -- Ghi vao buffer --
                with _lock:
                    _pkt_ok[0] += 1
                _write(val, sn / FS)
                sn += 1

        finally:
            # Dong cong bat ke ly do gi
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

        if _running:
            ts = datetime.now().strftime('%H:%M:%S')
            print(f'[{ts}][Serial] Ngat ket noi. Thu lai sau 2s...')
            time.sleep(2)


# ══════════════════════════════════ SIGNAL ANALYSIS ═══════════════════════════

def _analyze(raw: np.ndarray):
    n = len(raw)
    if n < _MIN_LEN:
        return None

    filt = filtfilt(_bpf_b, _bpf_a, raw)

    # Peak / Valley detection
    min_dist   = int(FS * 1.2)
    prominence = max(0.02, 0.08 * float(np.std(filt)) * 2)
    pk, _ = find_peaks( filt, distance=min_dist, prominence=prominence)
    vl, _ = find_peaks(-filt, distance=min_dist, prominence=prominence)

    # BPM tu khoang cach dinh
    bpm_pk = 0.0
    bpm_welch = 0.0
    if len(pk) >= 2:
        ivl = np.diff(pk) / FS
        ok  = ivl[(ivl >= 1.5) & (ivl <= 12.0)]
        if len(ok):
            bpm_pk = round(60.0 / float(np.mean(ok[-6:])), 1)

    # FFT -- Hann window tren FFT_N mau cuoi
    fw    = filt[-FFT_N:] if n >= FFT_N else filt
    L     = len(fw)
    fmag  = np.abs(rfft(fw * np.hanning(L)))
    ffreq = rfftfreq(L, d=1.0 / FS)
    mask  = (ffreq >= BPF_LO) & (ffreq <= BPF_HI)

    bpm_fft = 0.0
    dom_f   = 0.0
    if mask.any():
        bi      = int(np.argmax(fmag[mask]))
        dom_f   = float(ffreq[mask][bi])
        bpm_fft = round(dom_f * 60.0, 1)

    # Welch PSD
    welch_f = np.array([])
    welch_p = np.array([])
    welch_dom_f = 0.0
    try:
        nperseg = min(256, n)
        wf, wp = welch(filt, fs=FS, nperseg=nperseg, window='hann')
        wmask  = (wf >= BPF_LO) & (wf <= BPF_HI)
        if wmask.any():
            wi_dom = int(np.argmax(wp[wmask]))
            welch_dom_f = float(wf[wmask][wi_dom])
            bpm_welch = round(welch_dom_f * 60.0, 1)
        welch_f = wf
        welch_p = wp
    except Exception:
        pass

    # Trang thai lam sang
    bpm = bpm_pk if bpm_pk > 0 else bpm_fft
    if   bpm <= 0:  status = 0
    elif bpm > 25:  status = 2
    elif bpm < 10:  status = 3
    else:           status = 1

    return dict(filt=filt, pk=pk, vl=vl,
                bpm_pk=bpm_pk, bpm_fft=bpm_fft, dom_f=dom_f,
                bpm_welch=bpm_welch, welch_dom_f=welch_dom_f,
                welch_f=welch_f, welch_p=welch_p,
                ffreq=ffreq, fmag=fmag, status=status, n=n)


# ══════════════════════════════════ SAVE ══════════════════════════════════════
_fig_ref = [None]


def _do_save():
    fig = _fig_ref[0]
    if fig is None:
        return

    ts  = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    png = f'fft_result_{ts}.png'
    try:
        fig.savefig(png, dpi=150, bbox_inches='tight',
                    facecolor=C_BG, edgecolor='none')
    except Exception as e:
        print(f'[Luu] Loi PNG: {e}')
        return

    t_arr, raw_arr = _read_all()
    csv_path = f'fft_data_{ts}.csv'
    try:
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'raw_signal'])
            for tv, rv in zip(t_arr, raw_arr):
                w.writerow([f'{tv:.4f}', f'{rv:.6f}'])
    except Exception as e:
        print(f'[Luu] Loi CSV: {e}')
        return

    n_s = len(t_arr)
    print(f'\n[Luu]  {png}')
    print(f'[Luu]  {csv_path}  ({n_s} mau = {n_s / FS:.1f}s)\n')


def _on_save_btn(_event):
    _do_save()


def _on_close(_event):
    global _running
    _running = False
    print('\n[Thoat] Dang luu ket qua cuoi...')
    _do_save()


# ══════════════════════════════════ MAIN ══════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(
        description='Phan tich nhip tho realtime tu ESP32 + MPU6050')
    ap.add_argument('--port', required=True,
                    help='COM port cua ESP32, vi du: --port COM7')
    ap.add_argument('--baud', type=int, default=115200,
                    help='Baud rate (default: 115200)')
    args = ap.parse_args()

    mode_label = f'Serial  {args.port}  @  {args.baud} baud'
    threading.Thread(
        target=_serial_worker,
        args=(args.port, args.baud),
        daemon=True, name='SerialProducer',
    ).start()
    print(f'[Che do]  {mode_label}')
    print(f'[Info]    Nhan [Luu Ket Qua] hoac dong cua so de luu PNG + CSV\n')

    # ── Dung Figure ───────────────────────────────────────────────────────
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 10))
    fig.patch.set_facecolor(C_BG)
    _fig_ref[0] = fig
    fig.canvas.mpl_connect('close_event', _on_close)

    # ── Tieu de chinh ─────────────────────────────────────────────────────
    title_main = fig.suptitle(
        'Phân Tích Tín Hiệu Nhịp Thở — FFT & Peak Detection\n'
        'Thật: —  |  FFT: —  |  Welch: —  |  Peak: —',
        color=C_WHITE, fontsize=13, fontweight='bold', y=0.99,
        linespacing=1.6,
    )

    # ── GridSpec: 3 hang x 2 cot ─────────────────────────────────────────
    # Hang 0: ax_sig (toan chieu rong)
    # Hang 1: ax_fft (trai) | ax_welch (phai)
    # Hang 2: ax_bpm (trai) | ax_hist  (phai)
    gs = gridspec.GridSpec(
        3, 2, figure=fig,
        height_ratios=[2.8, 2.0, 2.0],
        hspace=0.52, wspace=0.32,
        left=0.07, right=0.97, top=0.89, bottom=0.10,
    )
    ax_sig   = fig.add_subplot(gs[0, :])
    ax_fft   = fig.add_subplot(gs[1, 0])
    ax_welch = fig.add_subplot(gs[1, 1])
    ax_bpm   = fig.add_subplot(gs[2, 0])
    ax_hist  = fig.add_subplot(gs[2, 1])

    for ax in (ax_sig, ax_fft, ax_welch, ax_bpm, ax_hist):
        ax.set_facecolor(C_PANE)
        for sp in ax.spines.values():
            sp.set_color(C_BORDER)
        ax.tick_params(colors=C_TEXT, labelsize=8)
        ax.grid(True, color=C_BORDER, lw=0.5, alpha=0.5)

    # ─────────────────────────────────────────────────────────────────────
    # Chart 1: Tín hiệu thô → lọc → đỉnh/đáy + đường lý tưởng
    # ─────────────────────────────────────────────────────────────────────
    (ln_raw,)   = ax_sig.plot([], [], color='#888888', lw=0.7, alpha=0.55,
                               label='Tín hiệu thô (+ nhiễu)')
    (ln_filt,)  = ax_sig.plot([], [], color=C_BLUE,   lw=1.8,
                               label='Sau Bandpass (0.1-0.7 Hz)')
    (ln_ideal,) = ax_sig.plot([], [], color=C_WHITE,  lw=0.9, ls='--', alpha=0.4,
                               label='Tín hiệu lý tưởng')
    sc_pk = ax_sig.scatter([], [], c=C_RED,    s=70, zorder=6, marker='^',
                           label='Đỉnh hít vào')
    sc_vl = ax_sig.scatter([], [], c=C_YELLOW, s=70, zorder=6, marker='v',
                           label='Đáy thở ra')
    ax_sig.axhline(0, color=C_BORDER, lw=0.8)
    ax_sig.set_xlabel('Thời gian (giây)', color=C_TEXT, fontsize=9)
    ax_sig.set_ylabel('Biên độ (a.u.)', color=C_TEXT, fontsize=9)
    ax_sig.legend(fontsize=8, loc='upper right', framealpha=0.2,
                  facecolor='#1c2333', edgecolor=C_BORDER,
                  labelcolor=C_WHITE, ncol=3)
    ax_sig.set_title('Tín Hiệu Thở: Thô → Lọc → Phát Hiện Đỉnh/Đáy',
                     color=C_WHITE, fontsize=10, pad=6)

    # ─────────────────────────────────────────────────────────────────────
    # Chart 2: Phổ FFT — toàn dải
    # ─────────────────────────────────────────────────────────────────────
    (ln_fft_line,) = ax_fft.plot([], [], color=C_PURPLE, lw=1.5,
                                  label='Biên độ FFT')
    vl_dom_fft = ax_fft.axvline(0, color=C_RED, lw=1.5, ls='--', alpha=0.9,
                                 label='Tần số chủ đạo')
    ax_fft.axvspan(BPF_LO, BPF_HI, alpha=0.10, color=C_BLUE,
                   label='Dải tìm kiếm (0.1-0.7 Hz)')
    ax_fft.set_xlim(0, 1.5)
    ax_fft.set_ylim(0, 1)
    ax_fft.set_xlabel('Tần số (Hz)', color=C_TEXT, fontsize=9)
    ax_fft.set_ylabel('Biên độ FFT', color=C_TEXT, fontsize=9)
    ax_fft.set_title('Phổ FFT — Toàn dải', color=C_WHITE, fontsize=10, pad=6)
    ax_fft.legend(fontsize=7.5, framealpha=0.2, facecolor='#1c2333',
                  edgecolor=C_BORDER, labelcolor=C_WHITE, loc='upper right')
    txt_fft_dom = ax_fft.text(
        0.98, 0.82, '', transform=ax_fft.transAxes,
        ha='right', fontsize=8, color=C_RED,
        bbox=dict(facecolor='#1c2333', alpha=0.7, edgecolor=C_BORDER, pad=3))

    # ─────────────────────────────────────────────────────────────────────
    # Chart 3: Welch PSD
    # ─────────────────────────────────────────────────────────────────────
    (ln_welch,) = ax_welch.plot([], [], color=C_ORANGE, lw=1.5,
                                 label='Dải ra tìm kiếm')
    vl_dom_welch = ax_welch.axvline(0, color=C_RED, lw=1.5, ls='--', alpha=0.9,
                                     label='Tần số chủ đạo')
    ax_welch.axvspan(BPF_LO, BPF_HI, alpha=0.10, color=C_BLUE)
    ax_welch.set_xlim(0, 1.5)
    ax_welch.set_yscale('log')
    ax_welch.set_ylim(1e-8, 1)   # phai dat ylim duong truoc khi log scale hieu luc
    ax_welch.set_xlabel('Tần số (Hz)', color=C_TEXT, fontsize=9)
    ax_welch.set_ylabel('PSD (log)', color=C_TEXT, fontsize=9)
    ax_welch.set_title('Welch Power Spectral Density', color=C_WHITE, fontsize=10, pad=6)
    ax_welch.legend(fontsize=7.5, framealpha=0.2, facecolor='#1c2333',
                    edgecolor=C_BORDER, labelcolor=C_WHITE, loc='upper right')
    txt_welch_dom = ax_welch.text(
        0.98, 0.82, '', transform=ax_welch.transAxes,
        ha='right', fontsize=8, color=C_RED,
        bbox=dict(facecolor='#1c2333', alpha=0.7, edgecolor=C_BORDER, pad=3))

    # ─────────────────────────────────────────────────────────────────────
    # Chart 4: BPM Theo Thời Gian (cửa sổ trượt 10s)
    # ─────────────────────────────────────────────────────────────────────
    (ln_bpm_trend,) = ax_bpm.plot([], [], color=C_GREEN, lw=1.5,
                                    marker='o', ms=2.5, label='BPM (cửa sổ 10s)')
    bpm_ref_line   = ax_bpm.axhline(0, color=C_RED,    lw=1.2, ls='--', alpha=0.8,
                                     label='Thật: —')
    bpm_mean_line  = ax_bpm.axhline(0, color=C_YELLOW, lw=0.9, ls=':',  alpha=0.8,
                                     label='Trung bình: —')
    ax_bpm.axhspan(10, 25, alpha=0.07, color=C_GREEN)
    ax_bpm.axhline(25, color=C_ORANGE, lw=0.7, ls='--', alpha=0.5)
    ax_bpm.axhline(10, color=C_BLUE,   lw=0.7, ls='--', alpha=0.5)
    ax_bpm.set_xlim(0, BUF_S)
    ax_bpm.set_ylim(0, 35)
    ax_bpm.set_xlabel('Thời gian (giây)', color=C_TEXT, fontsize=9)
    ax_bpm.set_ylabel('BPM', color=C_TEXT, fontsize=9)
    ax_bpm.set_title('BPM Theo Thời Gian (Cửa Sổ Trượt 10s)',
                     color=C_WHITE, fontsize=10, pad=6)
    ax_bpm.legend(fontsize=7.5, framealpha=0.2, facecolor='#1c2333',
                  edgecolor=C_BORDER, labelcolor=C_WHITE)

    # ─────────────────────────────────────────────────────────────────────
    # Chart 5: BPM Histogram
    # ─────────────────────────────────────────────────────────────────────
    ax_hist.set_xlabel('BPM tức thời', color=C_TEXT, fontsize=9)
    ax_hist.set_ylabel('Số lần xuất hiện', color=C_TEXT, fontsize=9)
    ax_hist.set_title('Phân Bố BPM Tức Thời (Histogram)',
                      color=C_WHITE, fontsize=10, pad=6)
    ax_hist.set_xlim(0, 35)
    ax_hist.set_ylim(0, 10)
    vl_hist_mean = ax_hist.axvline(0, color=C_RED,    lw=1.5, ls='--', alpha=0.9,
                                    label='TB: —')
    vl_hist_real = ax_hist.axvline(0, color=C_YELLOW, lw=0.9, ls=':',  alpha=0.8,
                                    label='Thật: —')
    txt_hist_n = ax_hist.text(
        0.02, 0.95, '', transform=ax_hist.transAxes,
        ha='left', va='top', fontsize=8, color=C_WHITE,
        bbox=dict(facecolor='#1c2333', alpha=0.7, edgecolor=C_BORDER, pad=3))
    _hist_bars = [None]   # giu tham chieu bar container de xoa lai

    # ── Status bar ────────────────────────────────────────────────────────
    txt_st  = fig.text(0.5,  0.006, 'Dang lay du lieu...',
                       ha='center', va='bottom', fontsize=10, color=C_TEXT,
                       fontweight='bold')
    txt_cnt = fig.text(0.97, 0.006, '',
                       ha='right', va='bottom', fontsize=8, color=C_DIM)

    # ── Nút lưu ───────────────────────────────────────────────────────────
    ax_btn = fig.add_axes([0.38, 0.007, 0.24, 0.036])
    btn    = Button(ax_btn, 'Luu Ket Qua  (PNG + CSV)',
                    color='#1c2333', hovercolor='#2d3748')
    btn.label.set_color(C_WHITE)
    btn.label.set_fontsize(9)
    btn.on_clicked(_on_save_btn)

    # ── Dữ liệu BPM theo thời gian ────────────────────────────────────────
    bpm_t_lst   = []
    bpm_pk_lst  = []
    _t_last_app = [0.0]
    # BPM tức thời từng chu kỳ (để histogram)
    bpm_instant_lst = []

    # ── Animation update ─────────────────────────────────────────────────
    def _update(_frame):
        times, raw_ord = _read_all()
        n = len(times)
        if n < _MIN_LEN:
            return

        res = _analyze(raw_ord)
        if res is None:
            return

        filt  = res['filt']
        t_now = float(times[-1])

        # ── Chart 1: toàn bộ BUF_S giây (hoặc dữ liệu hiện có) ──────────
        disp_n = n  # hiển thị tất cả dữ liệu đang có (đến 60s)
        t_d    = times[-disp_n:]
        r_d    = raw_ord[-disp_n:]
        f_d    = filt[-disp_n:]
        t_off  = t_d - t_d[0]

        # Xoa DC offset (trong luc ~9.8 m/s2) de hien thi ro tin hieu tho
        r_d_display = r_d - float(np.mean(r_d))
        ln_raw.set_data(t_off, r_d_display)
        ln_filt.set_data(t_off, f_d)

        # Đường lý tưởng: sine giả định từ BPM FFT
        bpm_fft = res['bpm_fft']
        if bpm_fft > 0 and len(t_off) > 1:
            ideal_amp = float(np.std(f_d)) * np.sqrt(2)
            ideal     = ideal_amp * np.sin(2 * np.pi * (bpm_fft / 60.0) * t_off)
            ln_ideal.set_data(t_off, ideal)
        else:
            ln_ideal.set_data([], [])

        ax_sig.set_xlim(0, max(float(t_off[-1]), 1.0))
        # Scale truc Y theo tin hieu LOC (AC), tranh DC offset lam bien do qua nho
        ymax = max(0.05, float(np.percentile(np.abs(f_d), 99)) * 2.5)
        ax_sig.set_ylim(-ymax, ymax)

        # Đỉnh/đáy trong cửa sổ hiển thị
        offset = n - disp_n
        pk_in  = res['pk'][res['pk'] >= offset] - offset
        vl_in  = res['vl'][res['vl'] >= offset] - offset
        sc_pk.set_offsets(
            np.column_stack([t_off[pk_in], f_d[pk_in]])
            if len(pk_in) else np.empty((0, 2)))
        sc_vl.set_offsets(
            np.column_stack([t_off[vl_in], f_d[vl_in]])
            if len(vl_in) else np.empty((0, 2)))

        # ── Chart 2: FFT ─────────────────────────────────────────────────
        ff   = res['ffreq']
        fm   = res['fmag']
        show = ff <= 1.5
        ln_fft_line.set_data(ff[show], fm[show])
        fmax = float(np.max(fm[show])) if show.any() else 1.0
        ax_fft.set_ylim(0, fmax * 1.25)
        dom_f = res['dom_f']
        if dom_f > 0:
            vl_dom_fft.set_xdata([dom_f, dom_f])
        txt_fft_dom.set_text(f'{dom_f:.4f} Hz\n→ {bpm_fft:.1f} BPM')

        # ── Chart 3: Welch PSD ───────────────────────────────────────────
        wf = res['welch_f']
        wp = res['welch_p']
        if len(wf) > 0:
            wshow = wf <= 1.5
            wshow_p = wp[wshow]
            wshow_p = np.where(wshow_p > 0, wshow_p, 1e-12)
            ln_welch.set_data(wf[wshow], wshow_p)
            wpmin = float(np.min(wshow_p[wshow_p > 0])) if np.any(wshow_p > 0) else 1e-12
            wpmax = float(np.max(wshow_p))
            ax_welch.set_ylim(wpmin * 0.5, wpmax * 5)
        bpm_welch   = res['bpm_welch']
        welch_dom_f = res['welch_dom_f']
        if welch_dom_f > 0:
            vl_dom_welch.set_xdata([welch_dom_f, welch_dom_f])
        txt_welch_dom.set_text(f'{welch_dom_f:.4f} Hz\n→ {bpm_welch:.1f} BPM')

        # ── Chart 4: BPM trend — cập nhật mỗi giây ───────────────────────
        bpm_pk = res['bpm_pk']
        if t_now - _t_last_app[0] >= 1.0:
            _t_last_app[0] = t_now
            bpm_val = bpm_pk if bpm_pk > 0 else (bpm_fft if bpm_fft > 0 else 0.0)
            bpm_t_lst.append(t_now)
            bpm_pk_lst.append(bpm_val if bpm_val > 0 else np.nan)
            if bpm_val > 0:
                bpm_instant_lst.append(bpm_val)
            # Giu 60s
            while len(bpm_t_lst) > 1 and bpm_t_lst[-1] - bpm_t_lst[0] > BUF_S:
                bpm_t_lst.pop(0)
                bpm_pk_lst.pop(0)
            if len(bpm_instant_lst) > 200:
                bpm_instant_lst.pop(0)

        if len(bpm_t_lst) >= 2:
            t0 = bpm_t_lst[0]
            bt = [x - t0 for x in bpm_t_lst]
            ln_bpm_trend.set_data(bt, bpm_pk_lst)
            ax_bpm.set_xlim(0, max(float(BUF_S), float(bt[-1])))

            valid_bpm = [v for v in bpm_pk_lst if not np.isnan(v) and v > 0]
            if valid_bpm:
                bpm_mean = float(np.mean(valid_bpm))
                bpm_mean_line.set_ydata([bpm_mean, bpm_mean])
            bpm_now = bpm_pk if bpm_pk > 0 else bpm_fft
            if bpm_now > 0:
                bpm_ref_line.set_ydata([bpm_now, bpm_now])

        # ── Chart 5: Histogram ───────────────────────────────────────────
        if len(bpm_instant_lst) >= 3:
            # Xoa bar cu
            if _hist_bars[0] is not None:
                for bar in _hist_bars[0]:
                    bar.remove()
            arr = np.array(bpm_instant_lst)
            lo  = max(0, arr.min() - 0.5)
            hi  = arr.max() + 0.5
            bins = max(8, int((hi - lo) / 0.1))
            cnt, edges = np.histogram(arr, bins=bins, range=(lo, hi))
            widths = np.diff(edges)
            _hist_bars[0] = ax_hist.bar(
                edges[:-1], cnt, width=widths, align='edge',
                color=C_PURPLE, alpha=0.75, edgecolor=C_BORDER)
            ax_hist.set_xlim(lo, hi)
            ax_hist.set_ylim(0, max(cnt) * 1.3 + 1)
            bpm_mean_h = float(np.mean(arr))
            vl_hist_mean.set_xdata([bpm_mean_h, bpm_mean_h])
            bpm_now2 = bpm_pk if bpm_pk > 0 else bpm_fft
            if bpm_now2 > 0:
                vl_hist_real.set_xdata([bpm_now2, bpm_now2])
            txt_hist_n.set_text(f'n = {len(arr)} khoảng\nTB: {bpm_mean_h:.1f} BPM')

        # ── Tiêu đề chính + status bar ────────────────────────────────────
        bpm_display = bpm_pk if bpm_pk > 0 else bpm_fft
        bpm_str     = f'{bpm_display:.1f}' if bpm_display > 0 else '—'
        fft_str     = f'{bpm_fft:.1f}' if bpm_fft > 0 else '—'
        welch_str   = f'{bpm_welch:.1f}' if bpm_welch > 0 else '—'
        pk_str      = f'{bpm_pk:.1f}' if bpm_pk > 0 else '—'
        title_main.set_text(
            'Phân Tích Tín Hiệu Nhịp Thở — FFT & Peak Detection\n'
            f'Thật: {bpm_str} BPM  |  FFT: {fft_str} BPM'
            f'  |  Welch: {welch_str} BPM  |  Peak: {pk_str} BPM'
        )

        st_label, st_color = STATUS_MAP[res['status']]
        with _lock:
            ok_n  = _pkt_ok[0]
            err_n = _pkt_err[0]
        loss_pct = (err_n / (ok_n + err_n) * 100) if (ok_n + err_n) > 0 else 0.0
        txt_st.set_text(
            f'{st_label}'
            f'   |   Peak: {bpm_pk:.1f} BPM'
            f'   |   FFT: {bpm_fft:.1f} BPM'
            f'   |   Welch: {bpm_welch:.1f} BPM'
            f'   |   Đỉnh:{len(res["pk"])}  Đáy:{len(res["vl"])}'
            f'   |   t={t_now:.0f}s'
            f'   |   Gói lỗi: {loss_pct:.1f}%')
        txt_st.set_color(st_color)
        txt_cnt.set_text(f'{n} mẫu ({n / FS:.0f}s)')

    ani = mplanim.FuncAnimation(
        fig, _update,
        interval=UPDATE_MS,
        blit=False,
        cache_frame_data=False,
    )
    fig._ani = ani   # Giu tham chieu tranh GC

    plt.show()


if __name__ == '__main__':
    main()
