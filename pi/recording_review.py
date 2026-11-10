"""Mac-side recording review. Downloads are read-only; analysis never controls motors."""
from __future__ import annotations

import copy
import csv
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import tarfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parent.parent


def pi_host(value):
    """Literal IP only: it is never interpolated into a shell command."""
    return str(ipaddress.ip_address(str(value).strip()))


def extract_recordings(archive, destination):
    """Only regular numbered JPEGs and log.csv inside dated run folders."""
    destination = Path(destination)
    with tarfile.open(archive) as tf:
        for member in tf:
            if member.isdir():
                continue
            parts = Path(member.name).parts
            if member.name.startswith("/") or ".." in parts or member.issym() or member.islnk():
                raise ValueError("Unsafe path or link in recording archive")
            if not member.isfile() or len(parts) != 2:
                continue
            run, name = parts
            import re
            if not re.fullmatch(r"\d{8}-\d{6}", run):
                continue
            if name != "log.csv" and not re.fullmatch(r"\d+\.jpg", name):
                continue
            target = destination / run / name
            target.parent.mkdir(parents=True, exist_ok=True)
            source = tf.extractfile(member)
            if source is None:
                continue
            with source, target.open("xb") as out:
                import shutil
                shutil.copyfileobj(source, out)


class RecordingReview:
    def __init__(self, root=None):
        self.root = Path(root or ROOT / "analysis_runs")
        self.lock = threading.Lock()
        self.cancelled = threading.Event()
        self.data = {"state": "idle", "stage": "Ready", "progress": 0, "report": None}

    def status(self):
        with self.lock:
            return copy.deepcopy(self.data)

    def publish(self, **values):
        with self.lock:
            self.data.update(values)

    def check_cancel(self):
        if self.cancelled.is_set():
            raise InterruptedError("Analysis cancelled; downloaded files kept")

    def start(self, host, knobs, deep=True):
        host = pi_host(host)
        with self.lock:
            if self.data["state"] == "running":
                raise RuntimeError("An analysis is already running")
            self.cancelled.clear()
            self.data = {"state": "running", "stage": "Connecting to Pi", "progress": 0,
                         "report": None, "host": host, "error": None}
        job = self.root / (time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6])
        threading.Thread(target=self._worker, args=(job, host, dict(knobs), bool(deep)), daemon=True).start()
        return self.status()

    def cancel(self):
        self.cancelled.set()
        return self.status()

    def _download(self, host, job):
        key = Path.home() / ".ssh" / "id_piracer"
        if not key.is_file():
            raise RuntimeError("Pi SSH key is missing on this Mac")
        archive = job / "recordings.tar"
        args = ["ssh", "-i", str(key), "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes",
                "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8", f"pi@{host}",
                "tar -C /home/pi/linefollow/data -cf - ."]
        # No remote mkdir, deployment, process launch or configuration writes.
        with archive.open("xb") as out, (job / "download-errors.txt").open("wb") as errors:
            proc = subprocess.Popen(args, stdout=out, stderr=errors)
            start = time.monotonic()
            try:
                while proc.poll() is None:
                    self.check_cancel()
                    if time.monotonic() - start > 1800:
                        raise TimeoutError("Download timed out; partial local copy kept")
                    self.publish(stage=f"Downloading recordings · {archive.stat().st_size / 1e6:.1f} MB", progress=5)
                    try:
                        proc.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        pass
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill(); proc.wait()
        if proc.returncode:
            raise RuntimeError("Pi download failed. Check its IP, Wi-Fi and SSH key. See download-errors.txt in the job folder.")
        self.publish(stage="Unpacking local copy", progress=12)
        extract_recordings(archive, job / "recordings")
        return job / "recordings"

    def _worker(self, job, host, knobs, deep):
        try:
            job.mkdir(parents=True, exist_ok=False)
            self.publish(folder=str(job))
            recordings = self._download(host, job)
            report = self.analyze(recordings, knobs, deep)
            self.check_cancel()
            (job / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
            self.publish(state="done", stage="Report ready", progress=100, report=report)
        except InterruptedError as e:
            self.publish(state="cancelled", stage=str(e))
        except Exception as e:
            self.publish(state="failed", stage="Analysis needs attention", error=str(e))

    @staticmethod
    def frame_times(folder, files):
        path = folder / "log.csv"
        rows = []
        if path.exists():
            with path.open() as f:
                rows = list(csv.DictReader(f))
        indexed = {str(int(r["n"])): r for r in rows if r.get("n", "").isdigit()}
        try:
            times = [float(indexed[str(int(p.stem))]["t"]) for p in files]
            if not all(b > a for a, b in zip(times, times[1:])):
                raise ValueError("Nonmonotonic timestamps")
            return [t - times[0] for t in times], rows, True
        except (KeyError, ValueError):
            return [i * .05 for i in range(len(files))], rows, False

    def analyze(self, recordings, knobs, deep=True):
        """Cheap inventory -> full replay -> model sweep -> rendered/uncertainty tests."""
        import sys
        for folder in (str(ROOT), str(ROOT / "tools")):
            if folder not in sys.path:
                sys.path.insert(0, folder)
        import numpy as np
        import backtest
        from linetrack.config import Config
        from sim.__main__ import run_one
        from sim.params import Profile
        from sim.tune import monte_carlo_profiles

        folders = sorted(p for p in Path(recordings).iterdir() if p.is_dir() and any(p.glob("*.jpg")))
        if not folders:
            raise RuntimeError("No recorded JPEG runs found in /home/pi/linefollow/data")
        overrides = {k: v for k, v in knobs.items() if k in Config().as_dict() and isinstance(v, (int, float, bool))}
        overrides.update({k: v for k, v in knobs.items() if k.startswith("speed_") and isinstance(v, (int, float))})
        effective_ceiling = min(float(knobs.get("max_auto", .6)), float(knobs.get("top_throttle", .6)), .8)
        overrides["max_throttle"] = effective_ceiling
        for key in ("coast_s", "slow_s", "single_line_stop_s"):
            if key in knobs:
                overrides["pilot_" + key] = knobs[key]
        if "auto_crawl" in knobs:
            overrides["auto_crawl"] = bool(knobs["auto_crawl"])
        report = {"runs": [], "frames": 0, "simulations": [], "robustness": [], "suggestions": [], "inferences": [],
                  "limits": ["Replay shows perception and proposed commands, not how a different controller would have moved the real car.",
                             "Found-line rates can include wrong-line picks; human labels are needed to establish line identity accuracy.",
                             "Grip, braking, actual speed and lane clearance need physical measurements.",
                             "Off-track events now check all four chassis corners against both tape outer edges; chassis axle offsets and all assumed car/track dimensions remain uncertain.",
                             "Simulated winners remain candidates for slow supervised trials; nothing is applied automatically."],
                  "settings": knobs, "simulator_candidate": None}
        report["effective_auto_ceiling"] = effective_ceiling
        if len(folders) < 2:
            report["limits"].append("Only one recording run: independent held-out-run validation is unavailable.")
        inventory = []
        for folder in folders:
            self.check_cancel()
            files = sorted(folder.glob("*.jpg"))
            times, rows, timed = self.frame_times(folder, files)
            inventory.append((folder, files, times, rows, timed))
            report["frames"] += len(files)
        self.publish(stage=f"Inventory complete · {report['frames']:,} images in {len(folders)} runs", progress=18)
        done = 0
        for j, (folder, files, times, recorded, timed) in enumerate(inventory):
            item = {"run": folder.name, "frames": len(files), "timing": "recorded" if timed else "assumed 20 fps",
                    "split": "held_out" if len(folders) > 1 and j >= max(1, int(.75 * len(folders))) else "development",
                    "has_telemetry": bool(recorded), "variants": {}}
            if timed and len(times) > 1:
                item["recorded_fps"] = round(1 / float(np.median(np.diff(times))), 2)
            for label, variant in (("baseline", {}), ("selected_settings", overrides)):
                self.check_cancel()
                self.publish(stage=f"Replay {j+1}/{len(folders)} · {label}", progress=20 + int(35 * done / max(1, 2 * report["frames"])))
                def progress(n):
                    self.check_cancel()
                    self.publish(stage=f"Replay · {folder.name} · {label} · {n:,}/{len(files):,} frames",
                                 progress=20 + int(35 * (done + n) / max(1, 2 * report["frames"])))
                rows = backtest.run(variant, "review", mode="local", frames=([str(p) for p in files], times), progress=progress)
                cfg, *_ = backtest.build(variant)
                metrics = backtest.score(rows, cfg.max_steering)
                metrics["corrupt_images"] = sum(r["status"] == "CORRUPT" for r in rows)
                metrics["sections"] = {s: sum(r["section"] == s for r in rows) for s in ("straight", "corner", "unknown")}
                metrics["throttle_change_mean"] = float(np.mean(np.abs(np.diff([r["throttle"] for r in rows])))) if len(rows) > 1 else 0
                ks = [abs(r["curvature"]) for r in rows if r["curvature"] is not None and np.isfinite(r["curvature"])]
                metrics["absolute_curvature_p50_p95"] = [float(v) for v in np.percentile(ks, [50, 95])] if ks else None
                metrics["section_switches"] = sum(a["section"] != b["section"] for a, b in zip(rows, rows[1:]))
                item["variants"][label] = metrics
                done += len(files)
            report["runs"].append(item)
            self.publish(report=copy.deepcopy(report))

        self.check_cancel()
        self.publish(stage="Comparing PD and section speeds in simulation", progress=58)
        profile = Profile.load()
        report["assumed_physical_parameters"] = profile.assumed_in_use()
        report["boundary_model"] = {
            "off_track_test": "Any of the four chassis footprint corners crossing either tape's outer edge.",
            "minimum_clearance_for_candidate_cm": 2.0,
            "turn_limits": "Steering-angle map sets minimum radius; lateral grip limits v^2/R; chassis footprint is checked against lane boundaries.",
        }
        report["physical_model"] = {
            name: {key: entry[key] for key in ("value", "unit", "status", "range", "source") if key in entry}
            for name, entry in profile.as_dict().items()
        }
        has_speed_plan = float(knobs.get("speed_straight", 0.0) or 0.0) > 0.0
        speed = None
        if has_speed_plan:
            straight = min(float(knobs["speed_straight"]), effective_ceiling)
            corner = min(float(knobs.get("speed_corner", straight) or straight), straight, effective_ceiling)
            speed = (straight, corner)
        report["speed_mode"] = "saved section plan" if has_speed_plan else "auto-crawl / fixed throttle from controller defaults"
        base = {k: v for k, v in overrides.items() if k not in ("speed_straight", "speed_corner")}
        candidates = [("selected", base, speed)]
        for section in ("straight", "corner"):
            for gain, default, ceiling in (("kp", 3.5, 10.), ("kd", .4, 2.)):
                for factor in (.8, 1.2):
                    gains = dict(base)
                    key = section + "_" + gain
                    gains[key] = min(ceiling, max(.05, float(knobs.get(key) or knobs.get(gain) or default) * factor))
                    candidates.append((f"{section} {gain.upper()} {factor:.1f}x", gains, speed))
        plan_speed = speed or (effective_ceiling, effective_ceiling)
        candidates.append(("section speed plan at ceiling", base, plan_speed))
        candidates.append(("slower corners", base, (plan_speed[0], max(0., plan_speed[1] - .03))))
        for i, (name, ov, spd) in enumerate(candidates):
            self.check_cancel()
            w, _ = run_one(profile, seconds=40, speed=spd, overrides=ov, seed=3)
            report["simulations"].append({"name": name, "metrics": w.metrics.summary(), "settings": ov, "speed": spd})
            self.publish(progress=60 + int(16 * (i+1) / len(candidates)), report=copy.deepcopy(report))
        # Rank only simulated evidence; never claim the replay identifies a faster real lap.
        def rank(row):
            m = row["metrics"]
            return (m["off_track"], m.get("lap_s") is None, m.get("lap_s") or 1e6, m["err_p95_cm"])
        winner = min(report["simulations"], key=rank)
        observed_loss = 1 - sum(r["variants"]["selected_settings"]["tracked%"] * r["frames"] for r in report["runs"]) / (100 * report["frames"])
        cadence = [r["recorded_fps"] for r in report["runs"] if "recorded_fps" in r and 5 <= r["recorded_fps"] <= 60]
        report["inferences"] = [f"Replay tracking failures: {100*observed_loss:.1f}% of frames; this mixes perception errors and line visibility, so it is not a measured random-dropout rate.",
                                 "Curvature distributions and section switches are recorded per run. Compare with measured corner radius before accepting the geometry."]
        hypothesis = {"meas_dropout": min(.4, max(float(profile.meas_dropout), observed_loss))}
        if cadence:
            hypothesis["camera_fps"] = min(float(profile.camera_fps), float(np.median(cadence)))
            report["inferences"].append(f"Median recording cadence: {float(np.median(cadence)):.1f} frames/s. Used as a conservative timing hypothesis, not a measured camera calibration.")
        self.check_cancel()
        q = profile.with_values(**hypothesis)
        w, _ = run_one(q, seconds=40, speed=winner["speed"], overrides=winner["settings"], seed=11)
        report["robustness"].append({"test": "Data-informed dropout/timing hypothesis", "hypothesis": hypothesis, "metrics": w.metrics.summary()})
        if deep:
            self.publish(stage="Rendered camera and uncertain-car tests", progress=78)
            for label, q, vision in [("rendered camera", profile, "rendered")] + [
                    (f"uncertain car {i+1}", q, "geometric") for i, q in enumerate(monte_carlo_profiles(profile, n=6, seed=7))]:
                self.check_cancel()
                w, _ = run_one(q, seconds=20, speed=winner["speed"], overrides=winner["settings"], seed=7, vision=vision)
                report["robustness"].append({"test": label, "metrics": w.metrics.summary()})
                self.publish(progress=78 + len(report["robustness"]) * 2, report=copy.deepcopy(report))
        winner_clearance = min(winner["metrics"].get("min_body_clear_left_cm") or 0.0,
                               winner["metrics"].get("min_body_clear_right_cm") or 0.0)
        robustness_safe = all(
            r["metrics"]["off_track"] == 0
            and min(r["metrics"].get("min_body_clear_left_cm") or 0.0,
                    r["metrics"].get("min_body_clear_right_cm") or 0.0) >= 2.0
            for r in report["robustness"]
        )
        report["simulator_candidate"] = (winner["name"] if winner["metrics"]["laps"] >= 2
                                         and winner["metrics"]["off_track"] == 0
                                         and winner_clearance >= 2.0 and robustness_safe else None)
        if report["simulator_candidate"] is None:
            report["suggestions"].append("No candidate completed two laps with zero off-track events and at least 2 cm body clearance in every robustness case")
        unknown = sum(r["variants"]["selected_settings"]["sections"]["unknown"] for r in report["runs"])
        tracked = sum(r["variants"]["selected_settings"]["tracked%"] * r["frames"] for r in report["runs"]) / report["frames"]
        report["suggestions"].append("Check detection and camera geometry before raising speed" if tracked < 90 or unknown > .2 * report["frames"]
                                     else "Validate straight, curve and transitions slowly on the car before raising speed")
        if any(r["metrics"]["off_track"] for r in report["robustness"]):
            report["suggestions"].append("The simulated candidate failed some uncertainty tests; lower speed and measure the car before using it")
        report["suggestions"].append("Measure lane width, corner radius, steering lock and timed speed at several throttle settings")
        report["summary"] = f"Reviewed {report['frames']:,} frames across {len(folders)} runs; selected settings tracked {tracked:.1f}% of recorded frames."
        return report
