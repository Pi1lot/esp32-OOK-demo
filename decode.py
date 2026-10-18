#!/usr/bin/env python3
"""
Recepteur OOK pour le montage pedagogique ESP32 -> Nooelec.

Flowgraph GNU Radio (en Python, via gnuradio-python / gr-osmosdr) :
  RTL-SDR source -> filtre passe-bas -> demod AM (magnitude) ->
  lissage -> seuillage -> affichage + decodage simple des bits.

Prerequis (Linux, Ubuntu/Debian-like) :
    sudo apt install gnuradio gr-osmosdr rtl-sdr
ou via conda/radioconda si tu preferes un environnement isole.

Usage :
    python3 ook_rx.py

Ajuste CENTER_FREQ pour qu'il corresponde exactement a la frequence que
ton ESP32 arrive a generer (voir la note dans esp32_ook_tx.ino).
"""

import numpy as np
from gnuradio import gr, blocks, filter as gr_filter, analog
import osmosdr
import time
import threading

# ----------------------------------------------------------------------
# Parametres a adapter a ton montage
# ----------------------------------------------------------------------
CENTER_FREQ = 27_018_470      # Hz - frequence reellement observee sur la cascade
SAMPLE_RATE = 2_000_000       # Hz - taux d'echantillonnage RTL-SDR
BIT_DURATION_S = 0.300        # doit matcher BIT_DURATION_MS cote ESP32
GAIN = 30                     # gain RTL-SDR, ajuste selon la distance au montage


class OokReceiver(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self, "OOK Receiver")

        # --- Source RTL-SDR (Nooelec) ---
        self.src = osmosdr.source(args="numchan=1")
        self.src.set_sample_rate(SAMPLE_RATE)
        self.src.set_center_freq(CENTER_FREQ)
        self.src.set_gain(GAIN)
        self.src.set_bandwidth(200_000)

        # --- Filtre passe-bas pour isoler le canal utile ---
        taps = gr_filter.firdes.low_pass(1, SAMPLE_RATE, 50_000, 10_000)
        self.lpf = gr_filter.fir_filter_ccf(1, taps)

        # --- Demodulation AM simple : magnitude du signal complexe ---
        self.mag = blocks.complex_to_mag(1)

        # --- Lissage (moyenne glissante) pour stabiliser l'enveloppe ---
        avg_len = max(1, int(SAMPLE_RATE * 0.002))  # ~2 ms de moyenne
        self.smooth = gr_filter.fir_filter_fff(1, [1.0 / avg_len] * avg_len)

        # --- Sortie vers un sink vectoriel qu'on lit en Python ---
        self.probe = blocks.probe_signal_f()

        self.connect(self.src, self.lpf, self.mag, self.smooth, self.probe)


def decode_bits(bitstring):
    """Cherche le preambule 1010101011 puis decode les octets suivants en ASCII."""
    preamble = "1010101011"
    idx = bitstring.find(preamble)
    if idx == -1:
        return None
    payload = bitstring[idx + len(preamble):]
    chars = []
    for i in range(0, len(payload) - 7, 8):
        byte_bits = payload[i:i + 8]
        if len(byte_bits) < 8:
            break
        val = int(byte_bits, 2)
        if 32 <= val <= 126:
            chars.append(chr(val))
        else:
            break
    return "".join(chars)


def reconstruct_bits(raw_samples, oversample):
    """Reconstruit la sequence de bits logiques a partir d'un flux
    suréchantillonné, en regroupant les paliers consecutifs (run-length)
    et en arrondissant leur duree au nombre de bits le plus proche.

    C'est ce qui rend le decodage robuste a une petite derive d'horloge
    entre l'ESP32 (delay()) et ce script (time.sleep()) : on mesure
    combien de temps chaque niveau a dure, plutot que de faire confiance
    a un echantillonnage suppose parfaitement aligne sur chaque bit."""
    if not raw_samples:
        return ""

    runs = []  # liste de (symbole, longueur_en_echantillons)
    current = raw_samples[0]
    length = 1
    for s in raw_samples[1:]:
        if s == current:
            length += 1
        else:
            runs.append((current, length))
            current = s
            length = 1
    runs.append((current, length))

    bits = []
    for symbol, length in runs:
        n_bits = max(1, round(length / oversample))
        bits.append(symbol * n_bits)
    return "".join(bits)


def main():
    tb = OokReceiver()
    tb.start()

    OVERSAMPLE = 5  # nombre d'echantillons pris par duree nominale d'un bit
    sample_interval = BIT_DURATION_S / OVERSAMPLE

    print(f"Ecoute sur {CENTER_FREQ/1e6:.3f} MHz, Ctrl+C pour arreter...")
    print(f"Surechantillonnage x{OVERSAMPLE} (echantillon toutes les {sample_interval*1000:.1f} ms) pour tolerer la derive d'horloge.\n")

    raw_samples = []
    RAW_HISTORY_LEN = 400  # ~ plusieurs trames completes
    level_history = []
    HISTORY_LEN = 12 * OVERSAMPLE  # fenetre glissante courte pour calibrer min/max
                                     # (assez courte pour ne pas etre diluee par
                                     # les pauses entre deux envois de message)

    next_tick = time.monotonic()

    try:
        while True:
            # planification par horodatage absolu plutot que sleep() cumulatif,
            # pour limiter la derive due au temps pris par le print/traitement
            next_tick += sample_interval
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

            level = tb.probe.level()

            level_history.append(level)
            level_history = level_history[-HISTORY_LEN:]

            lo = min(level_history)
            hi = max(level_history)
            threshold = (lo + hi) / 2
            contrast = hi - lo

            if contrast < 0.02:
                # pas assez de contraste pour decider -> on ne casse pas le
                # flux de symboles, on reutilise le dernier symbole connu
                symbol = raw_samples[-1] if raw_samples else "0"
            else:
                symbol = "1" if level > threshold else "0"

            raw_samples.append(symbol)
            raw_samples = raw_samples[-RAW_HISTORY_LEN:]

            reconstructed = reconstruct_bits(raw_samples, OVERSAMPLE)
            decoded = decode_bits(reconstructed)

            if decoded:
                print(f"niveau={level:.4f} seuil={threshold:.4f}  ->  message decode : {decoded!r}")
            # sinon on reste silencieux pour ne pas noyer le terminal
            # (decommente la ligne suivante si tu veux voir chaque echantillon)
            # else:
            #     print(f"niveau={level:.4f} seuil={threshold:.4f} symbole={symbol}")

    except KeyboardInterrupt:
        print("\nArret.")
    finally:
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
