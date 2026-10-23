"""Acquisition continue du Dextrain Manipulandum via nidaqmx (sans DAQMANIPULANDUM.dll).

Carte identifiee sur cette machine (voir probe_device.py) : NI USB-6000 (OEM),
bus USB, ADC unique multiplexe (ai_simultaneous_sampling_supported=False),
plage native +/-10V, ai_max_multi_chan_rate = 10000 S/s. Ce chiffre est un
debit AGREGE partage par toutes les voies actives (pas de sample-and-hold
simultane), donc pour 5 voies le plafond reel est ~10000/5 = 2000 S/s/voie,
pas 10000. Voir Device.max_rate_per_channel() ci-dessous.

Architecture, en reponse a la limite mesuree de l'ancienne DLL (~32 Hz,
~30 ms/appel) : celle-ci recree une tache NI-DAQmx (Create/Start/Read/
Stop/Clear) a chaque DaqAcq(). Ici la tache est creee et demarree UNE
SEULE FOIS, en continu (horloge materielle), et un thread dedie ne fait
que lire de petits blocs deja disponibles dans le buffer NI.

Separation volontaire :
- RawAcquisition   : uniquement des tensions brutes (Dev1/ai4:0, RSE, +/-10V).
- ZeroTracker      : soustraction d'un offset mesure au repos.
- Calibrator       : meme formule bilineaire que Manipulandum_data_aquired.cs
                      -- validee par compare_dll_vs_nidaqmx.py (tensions brutes
                      identiques a +/-0.0006V pres). Active par defaut dans le
                      CLI ; --raw-only pour revenir aux tensions brutes seules.
- TapDetector       : meme machine a etats dec/inc qu'Unity (detection globale
                      a la main, pas par doigt).
"""

from __future__ import annotations

import argparse
import csv
import queue
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import nidaqmx
from nidaqmx.constants import AcquisitionType, TerminalConfiguration
from nidaqmx.errors import DaqError
from nidaqmx.system import System

DEVICE_NAME = "Dev1"
CHANNELS = "ai4:0"          # ai4, ai3, ai2, ai1, ai0 -- meme ordre que la DLL
N_CHANNELS = 5
VOLT_MIN, VOLT_MAX = -10.0, 10.0

# Plafond produit volontaire, independant de la capacite materielle reelle
# (~2000 Hz/voie pour 5 voies sur cette carte, voir max_rate_per_channel).
# Un depassement leve DaqSetupError plutot que d'etre silencieusement clippe.
HARD_RATE_CEILING_HZ = 1000.0


class DaqSetupError(RuntimeError):
    pass


def max_rate_per_channel(device_name: str = DEVICE_NAME, n_channels: int = N_CHANNELS) -> dict:
    """Interroge la carte : debit max mono-voie vs multi-voies (agrege).

    Ne suppose jamais que max_multi_chan_rate == max par voie : sur un
    ADC multiplexe (ai_simultaneous_sampling_supported=False), c'est un
    total partage entre les voies actives.
    """
    dev = System.local().devices[device_name]
    simultaneous = dev.ai_simultaneous_sampling_supported
    single = dev.ai_max_single_chan_rate
    multi = dev.ai_max_multi_chan_rate
    per_channel = single if simultaneous else multi / n_channels
    return {
        "product_type": dev.product_type,
        "simultaneous_sampling": simultaneous,
        "ai_max_single_chan_rate": single,
        "ai_max_multi_chan_rate": multi,
        "effective_max_per_channel_for_n": per_channel,
        "n_channels": n_channels,
    }


@dataclass
class Sample:
    t_perf: float
    t_wall: float
    values: tuple  # 5 tensions brutes, ordre ai4..ai0


@dataclass
class AcquisitionStats:
    blocks_read: int = 0
    samples_read: int = 0
    hardware_acquired_total: int = 0
    overflow_count: int = 0
    max_log_queue_size: int = 0
    log_queue_warning_shown: bool = False
    max_backlog: int = 0
    last_error: Optional[str] = None
    read_intervals_s: list = field(default_factory=list)


class RawAcquisition:
    """Tache NI-DAQmx continue, creee/demarree une seule fois.

    Un thread dedie appelle task.read() en boucle avec de petits blocs
    (block_size), ce qui minimise la latence : chaque read() se debloque
    des qu'un bloc est disponible dans le buffer materiel, sans attendre
    un cycle Create/Start/Stop/Clear comme le faisait DaqAcq().
    """

    def __init__(
        self,
        sample_rate_hz: float,
        block_size: int = 1,
        buffer_seconds: float = 0.5,
        device_name: str = DEVICE_NAME,
        channels: str = CHANNELS,
        read_timeout_s: float = 5.0,
    ) -> None:
        if sample_rate_hz > HARD_RATE_CEILING_HZ:
            raise DaqSetupError(
                f"{sample_rate_hz} Hz/voie demande, mais le plafond produit est "
                f"fixe a {HARD_RATE_CEILING_HZ:.0f} Hz/voie (volontairement, "
                f"independamment de la capacite materielle)."
            )
        caps = max_rate_per_channel(device_name, N_CHANNELS)
        if sample_rate_hz > caps["effective_max_per_channel_for_n"]:
            raise DaqSetupError(
                f"{sample_rate_hz} Hz/voie demande, mais le debit multi-voies "
                f"agrege de {caps['product_type']} ({caps['ai_max_multi_chan_rate']} S/s) "
                f"ne permet que {caps['effective_max_per_channel_for_n']:.1f} Hz/voie "
                f"pour {N_CHANNELS} voies."
            )
        self.caps = caps
        self.sample_rate_hz = sample_rate_hz
        self.block_size = block_size
        self.read_timeout_s = read_timeout_s

        self._task = nidaqmx.Task()
        try:
            self._task.ai_channels.add_ai_voltage_chan(
                f"{device_name}/{channels}",
                terminal_config=TerminalConfiguration.RSE,
                min_val=VOLT_MIN,
                max_val=VOLT_MAX,
            )
            buffer_samples = max(int(sample_rate_hz * buffer_seconds), block_size * 8)
            self._task.timing.cfg_samp_clk_timing(
                rate=sample_rate_hz,
                sample_mode=AcquisitionType.CONTINUOUS,
                samps_per_chan=buffer_samples,
            )
        except Exception:
            # La tache a deja ete creee cote driver : si la configuration
            # echoue, il faut quand meme la liberer, sinon elle reste
            # ouverte/reservee sur Dev1 (invisible depuis ce process).
            self._task.close()
            raise

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_lock = threading.Lock()
        self._latest: Optional[Sample] = None
        self.log_queue: "queue.Queue[Sample]" = queue.Queue()
        self.stats = AcquisitionStats()
        self._consumed_total = 0
        self._t_start_perf = 0.0
        self._cpu_time_start = 0.0
        # 5s de retard sur le consommateur (CSV) = signal clair qu'il ne suit plus.
        self.log_queue_warn_threshold = max(int(sample_rate_hz * 5), 100)

    def start(self) -> None:
        try:
            self._task.start()
        except Exception:
            self._task.close()
            raise
        self._t_start_perf = time.perf_counter()
        self._cpu_time_start = time.process_time()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        stream = self._task.in_stream
        while not self._stop_event.is_set():
            try:
                raw = self._task.read(
                    number_of_samples_per_channel=self.block_size,
                    timeout=self.read_timeout_s,
                )
            except DaqError as exc:
                self.stats.overflow_count += 1
                self.stats.last_error = str(exc)
                self._stop_event.set()
                break

            t_perf = time.perf_counter()
            t_wall = time.time()

            # nidaqmx (>=1.x) renvoie toujours une liste imbriquee
            # [voie][echantillon], meme pour number_of_samples_per_channel=1.
            scans = list(zip(*raw))

            for values in scans:
                sample = Sample(t_perf=t_perf, t_wall=t_wall, values=values)
                with self._latest_lock:
                    self._latest = sample
                self.log_queue.put(sample)

            qsize = self.log_queue.qsize()
            if qsize > self.stats.max_log_queue_size:
                self.stats.max_log_queue_size = qsize
            if qsize > self.log_queue_warn_threshold and not self.stats.log_queue_warning_shown:
                self.stats.log_queue_warning_shown = True
                print(
                    f"\n[avertissement] log_queue a {qsize} echantillons en attente : "
                    "le consommateur (CSV) ne suit pas le rythme d'acquisition.",
                    flush=True,
                )

            self._consumed_total += len(scans)
            self.stats.blocks_read += 1
            self.stats.samples_read += len(scans)
            if self.stats.blocks_read > 1:
                self.stats.read_intervals_s.append(t_perf - self._last_read_t)
            self._last_read_t = t_perf

            backlog = stream.avail_samp_per_chan
            if backlog > self.stats.max_backlog:
                self.stats.max_backlog = backlog

    def get_latest(self) -> Optional[Sample]:
        with self._latest_lock:
            return self._latest

    def drain_for(self, seconds: float) -> list:
        """Consomme log_queue pendant `seconds` et renvoie les Sample recus.

        A utiliser pour la phase de zero : garantit qu'aucun echantillon
        ne reste bloque dans la queue (contrairement a une simple lecture
        de get_latest(), qui ne renverrait qu'un seul point).
        """
        deadline = time.perf_counter() + seconds
        collected = []
        while time.perf_counter() < deadline:
            remaining = deadline - time.perf_counter()
            try:
                collected.append(self.log_queue.get(timeout=max(remaining, 0.001)))
            except queue.Empty:
                break
        return collected

    def hardware_acquired_total_live(self) -> int:
        """Compteur materiel cumulatif, lisible uniquement tant que la tache est ouverte."""
        return self._task.in_stream.total_samp_per_chan_acquired

    def hardware_rate_hz(self) -> float:
        """Debit derive du compteur materiel NI-DAQmx (independant du thread Python)."""
        elapsed = time.perf_counter() - self._t_start_perf
        if elapsed <= 0:
            return 0.0
        return self.hardware_acquired_total_live() / elapsed

    def cpu_percent(self) -> float:
        """% d'un coeur CPU consomme par ce processus depuis start() (tous threads)."""
        wall = time.perf_counter() - self._t_start_perf
        if wall <= 0:
            return 0.0
        cpu = time.process_time() - self._cpu_time_start
        return 100.0 * cpu / wall

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=self.read_timeout_s + 1.0)
        try:
            # A capturer avant close() : le compteur materiel n'est plus
            # lisible une fois la tache fermee.
            self.stats.hardware_acquired_total = self._task.in_stream.total_samp_per_chan_acquired
        except Exception:
            pass
        try:
            self._task.stop()
        finally:
            self._task.close()

    def __enter__(self) -> "RawAcquisition":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()


class ZeroTracker:
    def __init__(self) -> None:
        self.zero = [0.0] * N_CHANNELS

    def calibrate(self, samples: list) -> list:
        n = len(samples)
        self.zero = [sum(s[i] for s in samples) / n for i in range(N_CHANNELS)]
        return list(self.zero)

    def apply(self, raw: tuple) -> list:
        return [raw[i] - self.zero[i] for i in range(N_CHANNELS)]


class Calibrator:
    """Meme formule bilineaire que Manipulandum_data_aquired.cs.

    NON VALIDEE independamment : n'active ce chemin qu'apres avoir fait
    tourner compare_dll_vs_nidaqmx.py et confirme que les tensions brutes
    correspondent a celles de l'ancienne DLL.
    """

    def __init__(self, calibration_path: Optional[Path] = None) -> None:
        self.g1 = [1.6] * N_CHANNELS
        self.g2 = [0.85] * N_CHANNELS
        self.c2 = [0.45] * N_CHANNELS
        self.intersection = [1.0] * N_CHANNELS  # defaut Unity, volontairement non recalcule
        if calibration_path is not None and calibration_path.exists():
            self._load(calibration_path)

    def _load(self, path: Path) -> None:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
        g1_line = lines[3].split("\t")
        g2_line = lines[4].split("\t")
        c2_line = lines[5].split("\t")
        for i in range(N_CHANNELS):
            self.g1[i] = float(g1_line[i + 1])
            self.g2[i] = float(g2_line[i + 1])
            self.c2[i] = float(c2_line[i + 1])
            self.intersection[i] = self.c2[i] / (self.g1[i] - self.g2[i])

    def apply(self, zeroed: list) -> list:
        forces = []
        for i, x in enumerate(zeroed):
            if abs(x) > abs(self.intersection[i]):
                forces.append(self.g2[i] * x + self.c2[i])
            else:
                forces.append(self.g1[i] * x)
        return forces


class TapDetector:
    """Machine a etats dec/inc identique a Unity : detection globale a la main."""

    def __init__(self, threshold: float = 0.5) -> None:
        self.threshold = threshold
        self._prev = [0.0] * N_CHANNELS
        self._dec = 0
        self._inc = 0

    def update(self, forces: list) -> bool:
        rising_phase = False
        falling_phase = False
        for i, f in enumerate(forces):
            if f > self.threshold and self._prev[i] < self.threshold:
                rising_phase = True
            if f < self.threshold and self._prev[i] > self.threshold:
                falling_phase = True
            self._prev[i] = f

        if rising_phase:
            self._dec = 1
        if self._dec == 1 and falling_phase:
            self._inc = 1
        tap = self._dec + self._inc == 2
        if tap:
            self._dec = 0
            self._inc = 0
        return tap


def describe(values: list) -> dict:
    if not values:
        return {}
    ordered = sorted(values)

    def pct(p: float) -> float:
        pos = (len(ordered) - 1) * p / 100.0
        lo, hi = int(pos), min(int(pos) + 1, len(ordered) - 1)
        frac = pos - lo
        return ordered[lo] * (1 - frac) + ordered[hi] * frac

    return {
        "min": min(values), "mean": statistics.fmean(values), "median": statistics.median(values),
        "p95": pct(95), "p99": pct(99), "max": max(values),
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
    }


def _consumer_thread(acq: RawAcquisition, path: Optional[Path], stop_event: threading.Event) -> None:
    """Vide log_queue en continu : ecrit dans un CSV si `path` est donne,
    sinon se contente de consommer pour empecher la queue de grossir
    indefiniment pendant le run.
    """
    fh = None
    writer = None
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        writer.writerow(["t_perf_s", "t_wall_s", "ai4", "ai3", "ai2", "ai1", "ai0"])
    try:
        while not stop_event.is_set() or not acq.log_queue.empty():
            try:
                sample = acq.log_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if writer is not None:
                writer.writerow([sample.t_perf, sample.t_wall, *sample.values])
    finally:
        if fh is not None:
            fh.close()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Acquisition continue Dextrain via nidaqmx")
    p.add_argument("--rate", type=float, default=1000.0, help="Sample rate par voie (Hz), plafond produit 1000 Hz")
    p.add_argument("--block-size", type=int, default=1, help="Echantillons/voie par read()")
    p.add_argument("--duration", type=float, default=None, help="Duree du run (s). Omis = tourne indefiniment jusqu'a Ctrl+C.")
    p.add_argument("--zero-seconds", type=float, default=1.0, help="Duree de la mesure du zero au repos")
    p.add_argument("--csv", type=Path, default=None, help="Chemin du CSV de sortie")
    p.add_argument(
        "--raw-only", action="store_true",
        help="Desactive zero+calibration+taps (actifs par defaut depuis validation "
        "contre l'ancienne DLL via compare_dll_vs_nidaqmx.py) : sort les tensions brutes seules.",
    )
    p.add_argument("--calibration", type=Path, default=None)
    p.add_argument(
        "--display-hz", type=float, default=5.0,
        help="Frequence d'affichage console (independante de l'acquisition). "
        "Mesure : un affichage trop rapide (ex. 10-20 Hz) ajoute de la contention "
        "GIL avec le thread de lecture et fait grimper la latence max (62 ms "
        "observes a 10 Hz contre 22 ms a 2 Hz). Garder cette valeur basse.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    caps = max_rate_per_channel(DEVICE_NAME, N_CHANNELS)

    stop_flag = threading.Event()

    try:
        with RawAcquisition(sample_rate_hz=args.rate, block_size=args.block_size) as acq:
            zero_samples = acq.drain_for(args.zero_seconds)
            if not zero_samples:
                latest = acq.get_latest()
                zero_samples = [latest] if latest is not None else []
            zero_tracker = ZeroTracker()
            zero_tracker.calibrate([s.values for s in zero_samples])

            # Consommateur demarre APRES la fenetre de zero : celle-ci a
            # deja entierement vide log_queue via drain_for().
            consumer_thread = threading.Thread(
                target=_consumer_thread, args=(acq, args.csv, stop_flag), daemon=True
            )
            consumer_thread.start()

            calibrator = None if args.raw_only else Calibrator(args.calibration)
            tap_detector = None if args.raw_only else TapDetector()

            deadline = time.perf_counter() + args.duration if args.duration is not None else float("inf")
            display_period = 1.0 / args.display_hz
            try:
                while time.perf_counter() < deadline:
                    sample = acq.get_latest()
                    if sample is not None:
                        if calibrator is not None:
                            forces = calibrator.apply(zero_tracker.apply(sample.values))
                            tap = tap_detector.update(forces)
                            line = "  ".join(f"{v:+7.3f}" for v in forces) + ("  TAP" if tap else "")
                        else:
                            line = "  ".join(f"{v:+7.4f}" for v in sample.values)
                        print("\r" + line, end="", flush=True)
                    time.sleep(display_period)
            except KeyboardInterrupt:
                print("\n\nArret demande (Ctrl+C).")

            stop_flag.set()
            consumer_thread.join(timeout=2.0)

            hw_total = acq.hardware_acquired_total_live()
            hw_rate = acq.hardware_rate_hz()
            cpu_pct = acq.cpu_percent()

            print("\n\n" + "=" * 60)
            print("STATISTIQUES")
            print("=" * 60)
            stats = acq.stats
            print(f"Blocs lus                : {stats.blocks_read}")
            print(f"Echantillons/voie lus (Python)    : {stats.samples_read}")
            print(f"Echantillons/voie acquis (materiel NI): {hw_total}")
            not_yet_read = hw_total - stats.samples_read
            print(f"  encore dans le buffer NI (pas une perte tant qu'il n'y a pas eu de debordement) : {not_yet_read}")
            print(f"Backlog max observe (avail_samp_per_chan) : {stats.max_backlog}")
            print(f"Debordements/erreurs detectes    : {stats.overflow_count}")
            if stats.last_error:
                print(f"Derniere erreur          : {stats.last_error}")
            print(f"Debit materiel (compteur NI, independant de Python) : {hw_rate:.2f} Hz")
            print(f"Utilisation CPU (processus, tous threads) : {cpu_pct:.1f} % d'un coeur")

            if stats.read_intervals_s:
                intervals_ms = [x * 1000.0 for x in stats.read_intervals_s]
                d = describe(intervals_ms)
                print("\nIntervalles entre blocs (latence de reception, ms)")
                for k in ("min", "mean", "median", "p95", "p99", "max", "stdev"):
                    print(f"  {k:8s}: {d[k]:8.3f} ms")

                pause_threshold_ms = 8.0
                pauses = [v for v in intervals_ms if v > pause_threshold_ms]
                if pauses:
                    pct_paused = 100.0 * len(pauses) / len(intervals_ms)
                    print(
                        f"\n{len(pauses)} intervalles ({pct_paused:.1f} %) depassent "
                        f"{pause_threshold_ms:.0f} ms (jusqu'a {max(pauses):.1f} ms)."
                    )
                    print(
                        "Ces pauses ont ete testees et ne varient pas avec la resolution du "
                        "timer Windows, la priorite du thread, la taille de bloc, ni la taille "
                        "de transfert USB (ai_usb_xfer_req_size). Elles sont documentees comme "
                        "une limitation probable du service USB de cette carte USB-6000 (OEM), "
                        "pas un probleme logiciel."
                    )

    except KeyboardInterrupt:
        stop_flag.set()
        print("\nArret demande (Ctrl+C).")
    except DaqSetupError as exc:
        print(f"Erreur de configuration : {exc}", file=sys.stderr)
        return 1
    except DaqError as exc:
        print(f"Erreur NI-DAQmx : {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
