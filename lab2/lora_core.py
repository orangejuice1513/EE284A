"""
lora_core.py — LoRa PHY signal chain (no GUI)
EE284A Interactive LoRa Lab

Full TX→channel→RX pipeline with:
  • CRC-16/CCITT-FALSE
  • 8-bit LFSR whitening  [S3: PRBS-9 approximation, not exact SX127x sequence]
  • Hamming(4+CR, 4) FEC  [S2: standard textbook Hamming; same rate ratios as SX127x]
  • Diagonal interleaver
  • Gray coding
  • Complex baseband chirp modulation
  • AWGN + Doppler flutter channel
  • Coherent dechirp + FFT detection

Simplifications vs. real SX127x (see S-labels in comments):
  S1  Implicit header only (no header parsing / field extraction)
  S2  Textbook Hamming(4+CR,4); not the exact SX127x parity polynomial
  S3  PRBS-9 approx (x^8+x^6+x^5+x^4+1); not exact SX127x whitening sequence
  S4  CRC covers payload only, not header
  S5  Low-data-rate optimisation (LDRO) not implemented
  S6  Perfect timing assumed — no correlator or timing-recovery loop
  S7  Phase continuous within a symbol; not across symbol boundaries
  S10 Sample rate = 2×BW (2 samp/chip); real chips use 1×BW

Run directly for unit tests:  python lora_core.py
"""

from __future__ import annotations
import copy
import multiprocessing
import os
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import numpy as np
from dataclasses import dataclass, field
from scipy import signal as scipy_signal
from typing import Optional


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

@dataclass
class LoRaParams:
    SF: int = 11             # Spreading factor 7..12
    BW: float = 250e3        # Bandwidth Hz: 125k, 250k, or 500k
    CR: int = 1              # Coding-rate offset: 1→4/5, 2→4/6, 3→4/7, 4→4/8
    SNR_dB: float = 10.0     # Channel SNR in dB (for AWGN)
    flutter_fd: float = 0.0  # Peak Doppler deviation (Hz)
    flutter_rate: float = 2.0 # Flutter oscillation rate (Hz)
    preamble_len: int = 8    # Number of preamble upchirps
    fs: float = 0.0          # Computed: 2 × BW  (set by lora_derived)

    def __post_init__(self):
        if self.fs == 0.0:
            self.fs = 2.0 * self.BW   # 2 samples per chip  [S10]


def lora_derived(p: LoRaParams) -> dict:
    """
    Compute all derived LoRa timing / link-budget quantities.

    Returns a dict with:
      N            — chips per symbol = 2^SF
      Ts           — symbol duration  = N / BW  (seconds)
      Tc           — chip duration    = 1 / BW  (seconds)
      n_samples    — samples per symbol = round(Ts * fs)
      slope        — frequency slope BW / Ts  (Hz/s)
      Rs           — symbol rate = 1/Ts  (symbols/s)
      Rb_raw       — uncoded bit rate = SF × Rs  (bit/s)
      Rb_eff       — effective bit rate after FEC = Rb_raw × 4/(4+CR)
      sensitivity_dBm — approximate receiver sensitivity at 10% PER
                        using the −124 dBm baseline for SF=6, BW=125 kHz
    """
    N = 2 ** p.SF
    Ts = N / p.BW
    Tc = 1.0 / p.BW
    fs = 2.0 * p.BW          # always recompute from BW
    n_samples = int(round(Ts * fs))
    slope = p.BW / Ts        # = BW² / N
    Rs = 1.0 / Ts
    Rb_raw = p.SF * Rs
    Rb_eff = Rb_raw * 4.0 / (4 + p.CR)

    # Semtech sensitivity formula (AN1200.13):
    #   sens ≈ −174 + 10·log10(BW) + NF + SNR_req
    # SNR_req ≈ −(SF×4/3 − 3.5) for coherent detection (empirical fit)
    NF_dB = 6.0              # typical module noise figure
    snr_req = -(p.SF * 4.0 / 3.0 - 3.5)
    sensitivity_dBm = -174 + 10 * np.log10(p.BW) + NF_dB + snr_req

    return dict(N=N, Ts=Ts, Tc=Tc, n_samples=n_samples, slope=slope,
                Rs=Rs, Rb_raw=Rb_raw, Rb_eff=Rb_eff,
                sensitivity_dBm=sensitivity_dBm)


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE
# ---------------------------------------------------------------------------

def crc16_ccitt(data: bytes) -> int:
    """
    CRC-16/CCITT-FALSE  (init=0xFFFF, poly=0x1021, no reflect).
    Returns 16-bit integer.  [S4: covers payload only]
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def _append_crc(data: bytes) -> bytes:
    """Append 2-byte big-endian CRC to data."""
    c = crc16_ccitt(data)
    return data + bytes([(c >> 8) & 0xFF, c & 0xFF])


def _check_and_strip_crc(data: bytes) -> tuple[bytes, bool]:
    """Verify and remove trailing 2-byte CRC.  Returns (payload, crc_ok)."""
    if len(data) < 2:
        return data, False
    payload = data[:-2]
    stored = (data[-2] << 8) | data[-1]
    return payload, (crc16_ccitt(payload) == stored)


# ---------------------------------------------------------------------------
# Whitening — 8-bit LFSR   [S3]
# ---------------------------------------------------------------------------

def _lfsr_sequence(length: int, seed: int = 0xFF) -> np.ndarray:
    """
    Generate `length` bytes from an 8-bit LFSR.
    Polynomial: x^8 + x^6 + x^5 + x^4 + 1  (taps at bits 6,5,4).
    [S3: approximation of SX127x PRBS-9; same structure, different polynomial]
    """
    seq = np.zeros(length, dtype=np.uint8)
    state = seed & 0xFF
    for i in range(length):
        seq[i] = state & 0xFF
        # feedback taps: bits 7 (MSB in), 5, 4, 3
        bit = ((state >> 7) ^ (state >> 5) ^ (state >> 4) ^ (state >> 3)) & 1
        state = ((state << 1) | bit) & 0xFF
    return seq


def whiten(data: bytes, seed: int = 0xFF) -> bytes:
    """
    XOR whitening with 8-bit LFSR sequence.
    Applying twice restores original (XOR is its own inverse).
    """
    seq = _lfsr_sequence(len(data), seed)
    return bytes(b ^ s for b, s in zip(data, seq))


# ---------------------------------------------------------------------------
# FEC — Hamming(4+CR, 4)   [S2]
# ---------------------------------------------------------------------------

# Generator matrices for CR 1..4
# Each G is a (4+CR)×4 binary matrix (rows = codeword bits, cols = data bits).
# Parity rows follow standard Hamming construction.
_HAMMING_G = {
    # CR=1 → 5,4 (1 parity bit: p = d0^d1^d2^d3)
    1: np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1],
                 [1,1,1,1]], dtype=np.uint8),
    # CR=2 → 6,4  (2 parity bits: p0=d0^d1^d2, p1=d0^d1^d3)
    2: np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1],
                 [1,1,1,0],
                 [1,1,0,1]], dtype=np.uint8),
    # CR=3 → 7,4  (standard Hamming(7,4))
    3: np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1],
                 [0,1,1,1],
                 [1,0,1,1],
                 [1,1,0,1]], dtype=np.uint8),
    # CR=4 → 8,4  (4 parity bits; extended Hamming)
    4: np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1],
                 [0,1,1,1],
                 [1,0,1,1],
                 [1,1,0,1],
                 [1,1,1,0]], dtype=np.uint8),
}

# Parity-check matrices H (syndrome rows)
_HAMMING_H = {
    1: np.array([[1,1,1,1,1]], dtype=np.uint8),
    2: np.array([[1,1,1,0,1,0],
                 [1,1,0,1,0,1]], dtype=np.uint8),
    3: np.array([[0,1,1,1,1,0,0],
                 [1,0,1,1,0,1,0],
                 [1,1,0,1,0,0,1]], dtype=np.uint8),
    4: np.array([[0,1,1,1,1,0,0,0],
                 [1,0,1,1,0,1,0,0],
                 [1,1,0,1,0,0,1,0],
                 [1,1,1,0,0,0,0,1]], dtype=np.uint8),
}


def _encode_nibble(nibble: int, CR: int) -> int:
    """Encode a 4-bit nibble to (4+CR) codeword bits using Hamming(4+CR,4)."""
    bits = np.array([(nibble >> i) & 1 for i in range(4)], dtype=np.uint8)
    cw_bits = (_HAMMING_G[CR] @ bits) % 2
    # Pack bits to integer: bit i of cw_bits → bit i of codeword (LSB = cw_bits[0])
    cw = 0
    for i, b in enumerate(cw_bits):
        cw |= int(b) << i
    return cw & ((1 << (4 + CR)) - 1)


def _decode_nibble(codeword: int, CR: int) -> tuple[int, int]:
    """
    Decode (4+CR)-bit codeword.  Returns (nibble, n_corrected).
    Corrects up to 1 error for CR≥2 (Hamming distance ≥3 codes).
    CR=1 is detection only (distance=2).
    """
    n_bits = 4 + CR
    bits = np.array([(codeword >> i) & 1 for i in range(n_bits)], dtype=np.uint8)
    H = _HAMMING_H[CR]
    syndrome = (H @ bits) % 2

    n_corrected = 0
    if np.any(syndrome):
        if CR >= 2:
            # Find column of H matching syndrome → flip that bit
            for col in range(n_bits):
                if np.array_equal(H[:, col], syndrome):
                    bits[col] ^= 1
                    n_corrected = 1
                    break

    # Extract data bits (first 4 rows of G were identity)
    nibble = int(bits[0]) | (int(bits[1]) << 1) | (int(bits[2]) << 2) | (int(bits[3]) << 3)
    return nibble, n_corrected


def fec_encode(data: bytes, CR: int) -> list[int]:
    """
    Encode bytes to list of (4+CR)-bit codewords.
    Each input byte → 2 nibbles → 2 codewords.
    """
    codewords = []
    for byte in data:
        lo = byte & 0x0F
        hi = (byte >> 4) & 0x0F
        codewords.append(_encode_nibble(lo, CR))
        codewords.append(_encode_nibble(hi, CR))
    return codewords


def fec_decode(codewords: list[int], CR: int) -> tuple[bytes, int]:
    """
    Decode list of (4+CR)-bit codewords back to bytes.
    Returns (data_bytes, total_bits_corrected).
    Pairs of codewords → lo nibble, hi nibble → byte.
    """
    out = []
    total_corrected = 0
    for i in range(0, len(codewords) - 1, 2):
        lo, nc0 = _decode_nibble(codewords[i], CR)
        hi, nc1 = _decode_nibble(codewords[i + 1], CR)
        out.append((hi << 4) | lo)
        total_corrected += nc0 + nc1
    return bytes(out), total_corrected


# ---------------------------------------------------------------------------
# Interleaver / Deinterleaver
# ---------------------------------------------------------------------------

def interleave(codewords: list[int], SF: int, CR: int) -> np.ndarray:
    """
    Diagonal interleaver.

    Writes codewords into an SF×(4+CR) matrix with cyclic row offsets,
    then reads out columns as LoRa symbols.

    Each codeword has (4+CR) bits.  We fill the matrix column-by-column
    (each column = one bit plane), applying a cyclic row shift equal to
    the column index.  Reading out rows gives SF-bit symbols.

    Returns: np.ndarray of shape (n_symbols,), dtype int, values in [0, 2^SF).
    """
    n_cw = 4 + CR          # bits per codeword
    # Pad codewords so length is a multiple of SF
    pad = (-len(codewords)) % SF
    codewords = list(codewords) + [0] * pad
    n_blocks = len(codewords) // SF

    symbols = []
    for blk in range(n_blocks):
        block = codewords[blk * SF:(blk + 1) * SF]
        # Build SF × n_cw bit matrix
        mat = np.zeros((SF, n_cw), dtype=np.uint8)
        for row, cw in enumerate(block):
            for col in range(n_cw):
                mat[row, col] = (cw >> col) & 1

        # Apply cyclic upward shift by col index on each column
        for col in range(n_cw):
            mat[:, col] = np.roll(mat[:, col], -col)

        # Read out columns as symbols (each column is SF bits → one symbol)
        for col in range(n_cw):
            sym = 0
            for row in range(SF):
                sym |= int(mat[row, col]) << row
            symbols.append(sym)

    return np.array(symbols, dtype=np.int64)


def deinterleave(symbols: np.ndarray, SF: int, CR: int) -> list[int]:
    """
    Inverse diagonal interleaver.  Recovers codeword list from symbol array.
    """
    n_cw = 4 + CR
    # Each block of n_cw symbols → SF codewords
    pad = (-len(symbols)) % n_cw
    symbols = list(symbols) + [0] * pad
    n_blocks = len(symbols) // n_cw

    codewords = []
    for blk in range(n_blocks):
        block = symbols[blk * n_cw:(blk + 1) * n_cw]
        # Rebuild SF × n_cw bit matrix from symbols (columns)
        mat = np.zeros((SF, n_cw), dtype=np.uint8)
        for col, sym in enumerate(block):
            for row in range(SF):
                mat[row, col] = (sym >> row) & 1

        # Undo cyclic shifts: shift column col downward by col
        for col in range(n_cw):
            mat[:, col] = np.roll(mat[:, col], col)

        # Read out rows as codewords
        for row in range(SF):
            cw = 0
            for col in range(n_cw):
                cw |= int(mat[row, col]) << col
            codewords.append(cw)

    return codewords


# ---------------------------------------------------------------------------
# Gray coding
# ---------------------------------------------------------------------------

def gray_encode(symbols: np.ndarray) -> np.ndarray:
    """Standard Gray code: G = n XOR (n >> 1)."""
    s = np.asarray(symbols, dtype=np.int64)
    return s ^ (s >> 1)


def gray_decode(gray: np.ndarray) -> np.ndarray:
    """Inverse Gray code via bit-peeling (MSB first)."""
    g = np.asarray(gray, dtype=np.int64)
    mask = g >> 1
    while np.any(mask):
        g = g ^ mask
        mask = mask >> 1
    return g


# ---------------------------------------------------------------------------
# Chirp generation
# ---------------------------------------------------------------------------

def make_base_upchirp(p: LoRaParams) -> np.ndarray:
    """
    Generate one period of the base upchirp (symbol value 0).

    Phase formula (matching chirpfab.py):
        φ(t) = π·(-BW·t + slope·t²)
    where slope = BW/Ts = BW²/N.

    Frequency sweeps linearly from -BW/2 to +BW/2 over one symbol period Ts.
    The 'π' (not '2π') appears because the instantaneous frequency is
    dφ/dt / (2π) = (-BW + 2·slope·t) / 2, which goes from -BW/2 to +BW/2.
    """
    d = lora_derived(p)
    n = d['n_samples']
    Ts = d['Ts']
    slope = d['slope']
    t = np.arange(n) / p.fs
    base = np.exp(1j * np.pi * (-p.BW * t + slope * t ** 2))
    return base


def make_symbol_chirp(m: int, p: LoRaParams, base: np.ndarray) -> np.ndarray:
    """
    Generate chirp for symbol value m.

    At 2 samp/chip (n = 2N samples), np.roll(base, m) does NOT produce a
    clean integer FFT bin after dechirping, because the linear phase term
    from the roll gives frequency m/(4N) cycles/sample → non-integer bin.

    The correct approach: multiply the base chirp by a complex exponential
    at digital frequency m/n cycles/sample:

        chirp_m[k] = base[k] · exp(j·2π·m·k / n)

    Dechirp product: chirp_m[k]·conj(base[k]) = exp(j·2π·m·k/n)
    → n-point FFT peaks at bin m.  [S10]

    Physical note: this does NOT produce frequency wrapping (real LoRa wraps
    at ±BW/2), but the dechirp FFT detection is identical.
    [S7: phase continuous within symbol; not guaranteed across boundaries]
    """
    n = len(base)
    k = np.arange(n)
    return base * np.exp(1j * 2 * np.pi * m * k / n)


def make_downchirp(p: LoRaParams, base: np.ndarray) -> np.ndarray:
    """
    Base downchirp = conjugate of the base upchirp.
    Frequency sweeps from +BW/2 to -BW/2.
    Used for the 2-symbol sync word.
    """
    return np.conj(base)


# ---------------------------------------------------------------------------
# TX chain
# ---------------------------------------------------------------------------

def build_packet(payload: bytes, p: LoRaParams) -> dict:
    """
    Full TX chain.  Returns dict with every intermediate result.

    Steps:
      1. Append CRC-16  [S4: payload only]
      2. Whiten bytes   [S3]
      3. FEC encode → codewords  [S2]
      4. Interleave → symbols
      5. Gray encode
      6. Modulate: preamble + sync + data chirps → complex IQ
    """
    # 1. CRC
    data_with_crc = _append_crc(payload)

    # 2. Whiten  [S3]
    whitened = whiten(data_with_crc)

    # 3. FEC encode  [S2]
    codewords = fec_encode(whitened, p.CR)

    # 4. Interleave
    symbols = interleave(codewords, p.SF, p.CR)

    # 5. Gray encode
    gray_syms = gray_encode(symbols)

    # 6. Modulate
    base = make_base_upchirp(p)
    downchirp = make_downchirp(p, base)

    # Preamble: p.preamble_len upchirps
    preamble_iq = np.tile(base, p.preamble_len)

    # Sync word: 2 downchirps  [S6: perfect sync assumed at RX]
    sync_iq = np.tile(downchirp, 2)

    # Data chirps
    data_iq = np.concatenate([make_symbol_chirp(int(m), p, base) for m in gray_syms])

    packet_iq = np.concatenate([preamble_iq, sync_iq, data_iq])

    return dict(
        payload=payload,
        data_with_crc=data_with_crc,
        whitened=whitened,
        codewords=codewords,
        symbols=symbols,
        gray_syms=gray_syms,
        base=base,
        downchirp=downchirp,
        packet_iq=packet_iq,
        n_data_symbols=len(gray_syms),
        # S1: implicit header — RX uses this to know how many bytes to decode
        n_data_bytes=len(data_with_crc),
    )


# ---------------------------------------------------------------------------
# Channel
# ---------------------------------------------------------------------------

def add_awgn(sig: np.ndarray, SNR_dB: float, p: LoRaParams,
             rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """
    Add complex Gaussian noise to achieve target SNR.

    SNR is defined per-sample (Es/N0 in the chip domain).
    Signal power is measured from the input; noise power is set accordingly.
    An optional numpy Generator `rng` may be supplied for thread-safe,
    reproducible noise (e.g. when called from ThreadPoolExecutor workers).
    """
    sig_power = np.mean(np.abs(sig) ** 2)
    SNR_linear = 10 ** (SNR_dB / 10.0)
    noise_power = sig_power / SNR_linear
    _rng = rng if rng is not None else np.random.default_rng()
    noise = np.sqrt(noise_power / 2.0) * (
        _rng.standard_normal(len(sig)) + 1j * _rng.standard_normal(len(sig))
    )
    return sig + noise


def add_doppler_flutter(sig: np.ndarray, p: LoRaParams) -> np.ndarray:
    """
    Multiply by exp(j·2π·(fd/fr)·sin(2π·fr·t)) to simulate Doppler flutter.

    Physical context: 915 MHz LoRa through tree canopy at 5 m/s wind gives
    fd ≈ 10–50 Hz with fr ≈ 0.5–3 Hz.  This model captures the sinusoidal
    phase variation produced by a moving scatterer at one frequency.
    Real multipath produces a sum of many such terms (Jakes model).

    fd=0 → no effect (returns signal unchanged).
    """
    if p.flutter_fd == 0.0:
        return sig
    t = np.arange(len(sig)) / p.fs
    fd = p.flutter_fd
    fr = p.flutter_rate
    # Instantaneous phase deviation: integral of fd·sin(2π·fr·t)
    phase = (fd / fr) * np.sin(2 * np.pi * fr * t)  # in cycles
    flutter = np.exp(1j * 2 * np.pi * phase)
    return sig * flutter


def apply_channel(iq: np.ndarray, p: LoRaParams,
                  rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """Apply Doppler flutter then AWGN (order matters: flutter first)."""
    iq = add_doppler_flutter(iq, p)
    iq = add_awgn(iq, p.SNR_dB, p, rng)
    return iq


# ---------------------------------------------------------------------------
# RX chain
# ---------------------------------------------------------------------------

def dechirp_symbol(chunk: np.ndarray, base: np.ndarray) -> np.ndarray:
    """
    Dechirp one symbol chunk: multiply by conjugate of base upchirp, then FFT.
    Returns magnitude spectrum (length = len(chunk)).

    The product removes the linear frequency sweep; the residual is a
    constant-frequency tone at bin m (the transmitted symbol value).
    FFT magnitude peaks at that bin.
    """
    dechirped = chunk * np.conj(base)
    return np.abs(np.fft.fft(dechirped))


def detect_symbol(spectrum: np.ndarray, SF: int) -> int:
    """
    Find the peak bin in the first N=2^SF bins of the dechirp spectrum.
    [S6: no timing recovery; chunk boundaries assumed perfect]
    """
    N = 2 ** SF
    return int(np.argmax(spectrum[:N]))


def receive_packet(rx_iq: np.ndarray, p: LoRaParams, tx_result: dict) -> dict:
    """
    Full RX chain.  Returns dict with every intermediate result.

    Steps:
      1. Skip preamble + sync (known position)   [S6: perfect timing]
      2. Dechirp each data symbol → FFT spectrum
      3. Detect symbol (argmax in first N bins)
      4. Gray decode
      5. Deinterleave
      6. FEC decode
      7. De-whiten
      8. CRC check

    Returns:
      detected_gray    — raw detected symbols (after dechirp, pre-gray-decode)
      gray_decoded     — after gray decode
      deinterleaved    — codewords
      decoded_bytes    — after FEC decode
      payload          — stripped payload (without CRC)
      crc_ok           — bool
      n_corrected      — FEC bit corrections
      dechirp_spectra  — list of FFT arrays, one per symbol (for GUI)
    """
    d = lora_derived(p)
    n = d['n_samples']
    N = d['N']

    base = tx_result['base']
    n_data_syms = tx_result['n_data_symbols']

    # Offset: skip preamble + 2 sync symbols
    skip = (p.preamble_len + 2) * n

    detected_gray = []
    dechirp_spectra = []

    for i in range(n_data_syms):
        start = skip + i * n
        chunk = rx_iq[start:start + n]
        if len(chunk) < n:
            chunk = np.pad(chunk, (0, n - len(chunk)))
        spec = dechirp_symbol(chunk, base)
        dechirp_spectra.append(spec)
        detected_gray.append(detect_symbol(spec, p.SF))

    detected_gray = np.array(detected_gray, dtype=np.int64)

    # 4. Gray decode
    gray_decoded = gray_decode(detected_gray)

    # 5. Deinterleave
    deinterleaved_cw = deinterleave(gray_decoded, p.SF, p.CR)

    # 6. FEC decode
    decoded_bytes, n_corrected = fec_decode(deinterleaved_cw, p.CR)
    # Truncate to expected length: interleave padding produces extra zero-codewords
    # that decode to extra bytes; implicit header (S1) gives us the correct byte count.
    n_expected = tx_result.get('n_data_bytes', len(decoded_bytes))
    decoded_bytes = decoded_bytes[:n_expected]

    # 7. De-whiten (same function, XOR inverse)
    dewhitened = whiten(decoded_bytes)

    # 8. CRC check
    payload, crc_ok = _check_and_strip_crc(dewhitened)

    return dict(
        detected_gray=detected_gray,
        gray_decoded=gray_decoded,
        deinterleaved=deinterleaved_cw,
        decoded_bytes=decoded_bytes,
        payload=payload,
        crc_ok=crc_ok,
        n_corrected=n_corrected,
        dechirp_spectra=dechirp_spectra,
    )


# ---------------------------------------------------------------------------
# PER simulation
# ---------------------------------------------------------------------------

def _per_snr_worker(args: tuple) -> tuple[float, float, float]:
    """Run n_trials Monte-Carlo trials at one SNR point for both flutter configs."""
    snr, packet_iq, base, n_data_symbols, n_data_bytes, p_no_fl, p_fl, n_trials = args

    # Each worker call gets its own independent RNG, seeded from OS entropy.
    # This is thread-safe (no shared global state) and also correct for
    # process-based workers (avoids correlated fork copies).
    rng = np.random.default_rng()

    p_no_fl = copy.copy(p_no_fl)
    p_fl    = copy.copy(p_fl)
    p_no_fl.SNR_dB = snr
    p_fl.SNR_dB    = snr

    tx_result = {'base': base, 'n_data_symbols': n_data_symbols, 'n_data_bytes': n_data_bytes}

    fail_no_fl = fail_fl = 0
    for _ in range(n_trials):
        rx_no = apply_channel(packet_iq.copy(), p_no_fl, rng)
        rx_fl = apply_channel(packet_iq.copy(), p_fl, rng)
        if not receive_packet(rx_no, p_no_fl, tx_result)['crc_ok']:
            fail_no_fl += 1
        if not receive_packet(rx_fl, p_fl,    tx_result)['crc_ok']:
            fail_fl    += 1

    return snr, fail_no_fl / n_trials, fail_fl / n_trials


def simulate_per_curve(
    p: LoRaParams,
    payload: bytes,
    snr_range: np.ndarray = None,
    n_trials: int = 200,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Monte-Carlo PER vs. SNR simulation.

    Runs two curves:
      • per_no_flutter:   p.flutter_fd set to 0 internally
      • per_with_flutter: p.flutter_fd as given in p

    Returns: (snr_db, per_no_flutter, per_with_flutter)

    On Windows, uses ThreadPoolExecutor (threads share the process, so the
    GUI main script is never re-executed in workers).  NumPy FFT and AWGN
    generation release the GIL, so threads genuinely parallelize the heavy
    work.  On macOS/Linux, ProcessPoolExecutor with fork is used instead.
    """
    if snr_range is None:
        snr_range = np.arange(-25, 15, 1, dtype=float)

    p_no_fl = copy.copy(p)
    p_fl    = copy.copy(p)
    p_no_fl.flutter_fd = 0.0

    # Build TX once (same payload, same modulation)
    tx = build_packet(payload, p)

    work_items = [
        (snr, tx['packet_iq'], tx['base'], tx['n_data_symbols'], tx['n_data_bytes'],
         p_no_fl, p_fl, n_trials)
        for snr in snr_range
    ]

    if sys.platform == 'win32':
        # ThreadPoolExecutor avoids spawning subprocesses that would re-execute
        # the GUI main script (lora_lab.py) and open unwanted windows.
        with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
            results = list(executor.map(_per_snr_worker, work_items))
    else:
        with ProcessPoolExecutor(max_workers=os.cpu_count(),
                                 mp_context=multiprocessing.get_context('fork')) as executor:
            results = list(executor.map(_per_snr_worker, work_items))

    per_no_fl = np.array([r[1] for r in results])
    per_fl    = np.array([r[2] for r in results])

    return snr_range, per_no_fl, per_fl


# ---------------------------------------------------------------------------
# Visualization helpers
# ---------------------------------------------------------------------------

def compute_spectrogram(iq: np.ndarray, p: LoRaParams) -> tuple:
    """
    Compute spectrogram of the packet IQ.

    nperseg = n_samples_per_symbol so each column = one LoRa symbol.
    This makes chirp stripes visible as diagonal lines in the time-frequency plot.

    Matches pychirp.py: colormap clipped to 5th–99th percentile.

    Returns: (f_Hz, t_sec, Sxx_dB)
    """
    d = lora_derived(p)
    n = d['n_samples']
    f, t, Sxx = scipy_signal.spectrogram(
        iq, fs=p.fs, nperseg=n, noverlap=0,
        window='boxcar', return_onesided=False
    )
    # Shift frequency axis so DC is in the center
    f = np.fft.fftshift(f)
    Sxx = np.fft.fftshift(Sxx, axes=0)
    Sxx_dB = 10 * np.log10(Sxx + 1e-12)
    return f, t, Sxx_dB


def compute_inst_freq(iq: np.ndarray, fs: float) -> tuple:
    """
    Instantaneous frequency via angle-difference method (matches chirpfab.py style).

    f_inst[n] = angle(iq[n+1] * conj(iq[n])) / (2π / fs)

    Returns: (t_sec, f_inst_Hz) with length len(iq)-1
    """
    phase_diff = np.angle(iq[1:] * np.conj(iq[:-1]))
    f_inst = phase_diff * fs / (2 * np.pi)
    t = np.arange(len(f_inst)) / fs
    return t, f_inst


def packet_structure_info(p: LoRaParams, n_payload_bytes: int) -> list:
    """
    Return list of (label, n_symbols) describing the packet structure.
    Useful for the GUI packet-structure bar chart.

    Labels: 'Preamble', 'Sync', 'Data'
    """
    d = lora_derived(p)
    # Data bytes = payload + 2 CRC bytes; each byte → 2 nibbles → 2 codewords;
    # after interleaving: n_symbols = n_codewords × (4+CR) / SF, padded to multiple
    total_bytes = n_payload_bytes + 2  # +2 for CRC
    whitened_bytes = total_bytes       # whitening doesn't change length
    n_codewords = whitened_bytes * 2   # 2 nibbles per byte
    n_cw_bits = 4 + p.CR
    # interleave pads to multiple of SF, then reads out n_cw columns per block
    n_symbols_raw = n_codewords * n_cw_bits
    # After interleave: each block of SF codewords → n_cw_bits symbols
    n_data_syms = int(np.ceil(n_codewords / p.SF)) * (4 + p.CR)

    return [
        ('Preamble', p.preamble_len),
        ('Sync',     2),
        ('Data',     n_data_syms),
    ]


# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------

def _run_tests():
    import sys
    failures = []

    def check(name, cond):
        status = "PASS" if cond else "FAIL"
        print(f"  [{status}] {name}")
        if not cond:
            failures.append(name)

    print("=" * 60)
    print("lora_core.py — unit tests")
    print("=" * 60)

    # 1. Whitening is its own inverse
    print("\n1. Whitening XOR inverse")
    data = bytes(range(32))
    check("whiten(whiten(data)) == data", whiten(whiten(data)) == data)

    # 2. Hamming round-trip all nibbles, all CR
    print("\n2. Hamming round-trip (all nibbles, all CR)")
    for cr in range(1, 5):
        ok = all(
            _decode_nibble(_encode_nibble(n, cr), cr)[0] == n
            for n in range(16)
        )
        check(f"CR={cr} round-trip all nibbles", ok)

    # 3. Hamming single-bit correction for CR≥3
    # Note: CR=2 uses a (6,4) code with only 2 parity bits → 3 distinct syndromes
    # for 6 bit positions, so single-bit correction is only possible for CR≥3
    # where the minimum Hamming distance is ≥3 across all positions.
    print("\n3. Hamming single-bit correction (CR≥3)")
    for cr in range(3, 5):
        n_bits = 4 + cr
        ok = True
        for nibble in range(16):
            cw = _encode_nibble(nibble, cr)
            for bit in range(n_bits):
                flipped = cw ^ (1 << bit)
                decoded, nc = _decode_nibble(flipped, cr)
                if decoded != nibble or nc != 1:
                    ok = False
                    break
            if not ok:
                break
        check(f"CR={cr} single-bit correction", ok)

    # 4. Deinterleave(interleave(codewords)) == codewords
    print("\n4. Interleave round-trip (SF in [7,9,12])")
    for sf in [7, 9, 12]:
        for cr in range(1, 5):
            p = LoRaParams(SF=sf, CR=cr)
            payload = bytes(range(8))
            cws = fec_encode(payload, cr)
            syms = interleave(cws, sf, cr)
            cws2 = deinterleave(syms, sf, cr)
            # Original codewords may be padded; compare the first len(cws)
            ok = all(a == b for a, b in zip(cws, cws2[:len(cws)]))
            check(f"SF={sf} CR={cr} interleave round-trip", ok)

    # 5. Gray round-trip
    print("\n5. Gray encode/decode round-trip")
    for sf in [7, 12]:
        N = 2 ** sf
        arr = np.arange(N)
        ok = np.all(gray_decode(gray_encode(arr)) == arr)
        check(f"SF={sf} gray round-trip", ok)

    # 6. Dechirp FFT peak at correct bin (high SNR)
    print("\n6. Dechirp FFT peak at correct bin")
    p = LoRaParams(SF=7, SNR_dB=40.0, flutter_fd=0.0)
    base = make_base_upchirp(p)
    for m in [0, 1, 64, 127]:
        chirp = make_symbol_chirp(m, p, base)
        spec = dechirp_symbol(chirp, base)
        peak = detect_symbol(spec, p.SF)
        check(f"symbol m={m} peak at bin {m}", peak == m)

    # 7. Full TX→RX loopback at high SNR
    print("\n7. TX→RX loopback (SNR=40dB, CRC ok)")
    payload = bytes.fromhex('DEADBEEF01020304')
    for sf in [7, 9, 12]:
        p = LoRaParams(SF=sf, SNR_dB=40.0, flutter_fd=0.0)
        tx = build_packet(payload, p)
        # No channel (noiseless)
        rx = receive_packet(tx['packet_iq'], p, tx)
        check(f"SF={sf} crc_ok", rx['crc_ok'])
        check(f"SF={sf} payload match", rx['payload'] == payload)

    # 8. PER curve monotonically decreasing
    print("\n8. PER curve monotone decreasing")
    p = LoRaParams(SF=7, SNR_dB=0.0, flutter_fd=0.0)
    payload = bytes.fromhex('DEADBEEF')
    snr_r = np.arange(-20, 10, 2, dtype=float)
    snr_db, per_nf, per_fl = simulate_per_curve(p, payload, snr_range=snr_r, n_trials=50)
    # Allow non-strict (plateaus at 0 or 1 are ok)
    mono = np.all(np.diff(per_nf) <= 0.05)  # allow tiny noise
    check("PER no-flutter roughly monotone decreasing", mono)

    print("\n" + "=" * 60)
    if failures:
        print(f"FAILED: {len(failures)} test(s): {failures}")
        sys.exit(1)
    else:
        print("All tests passed.")
    print("=" * 60)


if __name__ == "__main__":
    _run_tests()
