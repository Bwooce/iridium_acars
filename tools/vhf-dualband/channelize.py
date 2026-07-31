#!/usr/bin/env python3
# One HydraSDR (10 MSPS @ 133.5 MHz) -> two VHF ACARS bands in parallel.
#   VDL2 : shift 136.9->DC, windowed-sinc FIR decimate /20 -> 500 kHz int16 IQ on stdout
#          (-> sox 500k->420k -> dumpvdl2)
#   POA  : per channel shift->DC, windowed-sinc FIR decimate /800 -> 12.5 kHz AM-demod
#          audio, 4-ch int16 WAV -> fifo (-> acarsdec -f)
#
# numpy only (no scipy). Blackman-windowed-sinc low-pass filters replace the
# earlier boxcar decimation for real alias rejection (~-58 dB sidelobes vs
# -13 dB), so weak frames survive decimation. VDL2 uses overlap-save FIR
# (cross-chunk state, phase-continuous). POA uses a length-D windowed-sinc
# block decimator (D=800 taps is already a sharp filter; no state needed).
# Chunk = 800000 samples is an integer multiple of every NCO period AND every
# decimation factor, so phases stay continuous with no bookkeeping.
import sys, subprocess, numpy as np

FS       = 10_000_000
FCENTER  = 133_500_000
CHUNK    = 800_000
VDL2_C   = 136_900_000
VDL2_DEC = 20                 # -> 500 kHz
VDL2_TAPS= 41                 # overlap-save FIR length (odd)
POA_CH   = [131_550_000, 130_025_000, 130_425_000, 130_450_000]
POA_DEC  = 800                # -> 12_500 Hz exactly

poa_fifo_path = sys.argv[1]

def fir_lowpass(numtaps, fc):
    # windowed-sinc low-pass; fc in cycles/sample (0..0.5). Blackman window.
    n = np.arange(numtaps) - (numtaps - 1) / 2.0
    h = np.sinc(2 * fc * n) * np.blackman(numtaps)
    h /= h.sum()
    return h.astype(np.float32)

def nco(f_hz):
    n = np.arange(CHUNK, dtype=np.float64)
    return np.exp(-2j * np.pi * ((f_hz - FCENTER) / FS) * n).astype(np.complex64)

nco_vdl2 = nco(VDL2_C)
nco_poa  = [nco(f) for f in POA_CH]

# VDL2 overlap-save FIR (cutoff ~90% of the 25 kHz output Nyquist relative to input)
h_vdl2   = fir_lowpass(VDL2_TAPS, 0.45 / VDL2_DEC)
state_v  = np.zeros(VDL2_TAPS - 1, dtype=np.complex64)

# POA: length-800 windowed-sinc, applied per non-overlapping 800-sample block
h_poa    = fir_lowpass(POA_DEC, 0.45 / POA_DEC)   # sharp low-pass at ~6.25 kHz

hrx = subprocess.Popen(
    ["hydrasdr_rx","-f","133.5","-a",str(FS),"-t","2","-g","21","-r","/dev/stdout"],
    stdout=subprocess.PIPE, stderr=open("/tmp/dualband_hrx.err","wb"), bufsize=0)

def wav_hdr(ch, rate, bits, datasz=0x7fffff00):
    import struct
    br = rate*ch*bits//8; ba = ch*bits//8
    return (b"RIFF"+struct.pack("<I",36+datasz)+b"WAVEfmt "
            +struct.pack("<IHHIIHH",16,1,ch,rate,br,ba,bits)+b"data"+struct.pack("<I",datasz))

poa_fifo = open(poa_fifo_path, "wb")     # blocks until acarsdec opens read end
poa_fifo.write(wav_hdr(len(POA_CH), 12_500, 16)); poa_fifo.flush()
out = sys.stdout.buffer

need = CHUNK*2*2
def readexact(f, n):
    parts = []; got = 0
    while got < n:
        b = f.read(n-got)
        if not b: return b"".join(parts)
        parts.append(b); got += len(b)
    return b"".join(parts)

chunks = 0
while True:
    buf = readexact(hrx.stdout, need)
    if len(buf) < need:
        break
    x = np.frombuffer(buf, dtype=np.int16).astype(np.float32)
    c = x[0::2] + 1j*x[1::2]

    # --- VDL2: shift, overlap-save FIR, decimate /20 ---
    v  = c * nco_vdl2
    xv = np.concatenate([state_v, v])
    fv = np.convolve(xv, h_vdl2, "valid")          # CHUNK samples, phase-continuous
    state_v = v[-(VDL2_TAPS-1):]
    d = fv[::VDL2_DEC]
    iq = np.empty(d.size*2, np.int16)
    iq[0::2] = np.clip(d.real, -32767, 32767)
    iq[1::2] = np.clip(d.imag, -32767, 32767)
    out.write(iq.tobytes())

    # --- POA: per-channel shift, length-800 windowed-sinc block decimate, AM-demod ---
    nsamp = CHUNK // POA_DEC
    aud = np.empty((nsamp, len(POA_CH)), np.int16)
    for k, ncok in enumerate(nco_poa):
        p  = (c * ncok).reshape(nsamp, POA_DEC)     # non-overlapping blocks
        p  = p @ h_poa                              # length-800 FIR per block
        am = np.abs(p); am -= am.mean()
        aud[:, k] = np.clip(am*8.0, -32767, 32767)
    poa_fifo.write(aud.tobytes())

    chunks += 1
    if chunks % 12 == 0:
        sys.stderr.write("channelize: %d chunks (%.1fs)\n" % (chunks, chunks*CHUNK/FS)); sys.stderr.flush()
