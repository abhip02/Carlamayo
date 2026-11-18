"""Run explicitly selected frozen driving jobs under model/render/port leases."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time

from local.experiments.model_score_benchmark import ROOT
from local.experiments.nf4_closed_loop import verify_manifest
from local.experiments.resource_lease import lease, check_memory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--precision", choices=("bf16", "nf4"), required=True)
    parser.add_argument("--model-gpu", "--model-gpus", dest="model_gpu", required=True,
                        help="One UUID, or the three qualified model UUIDs comma-separated")
    parser.add_argument("--render-gpu", required=True)
    parser.add_argument("--container", required=True)
    parser.add_argument("--cpu-offload", action="store_true")
    parser.add_argument("--wall-timeout-s", type=float, default=1800.)
    parser.add_argument("--cases", nargs="+", type=int, choices=(9, 10), default=[9, 10])
    parser.add_argument("--headroom-seconds", type=float, default=60.)
    args = parser.parse_args()
    model_gpus = args.model_gpu.split(",")
    if args.render_gpu in model_gpus or len(model_gpus) != len(set(model_gpus)):
        raise ValueError("Separate model and render GPUs are required")
    manifest = json.loads(args.manifest.read_text())
    jobs = [j for number in args.cases for j in manifest["jobs"]
            if j["precision"] == args.precision and j["name"].startswith(f"{number:02d}-")]
    if len(jobs) != len(set(args.cases)) or len(args.cases) != len(set(args.cases)):
        raise ValueError("Select unique cases present in the frozen manifest")
    sharded = len(model_gpus) == 3 and args.precision == "bf16" and all("--device-map" in j["command"] for j in jobs)
    if len(model_gpus) != 1 and not sharded:
        raise ValueError("Multiple model GPUs require the qualified BF16 map")
    output = args.manifest.parent
    status_path = output / (args.precision + "-status.json")
    if status_path.exists():
        raise ValueError("Preserve previous outcomes; prepare a new pilot directory")
    status = dict(state="prepared", requested=[j["name"] for j in jobs], results=[],
                  fresh_matched_pair_complete=False, cpu_offload=args.cpu_offload,
                  model_gpus=model_gpus, renderer_gpu=args.render_gpu,
                  wall_timeout_s_per_case=args.wall_timeout_s)
    def save():
        temporary = status_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(status, indent=2) + "\n")
        temporary.replace(status_path)
    save()
    env = dict(os.environ, CARLAMAYO_MODEL_GPUS=args.model_gpu, CUDA_VISIBLE_DEVICES=args.model_gpu,
               CARLAMAYO_RENDER_GPU=args.render_gpu, CARLAMAYO_PROJECTION_CONTAINER=args.container)
    server = ["bash", str(ROOT / "local/experiments/projected_server.sh")]
    with lease([*model_gpus, args.render_gpu], port=2000):
        # Offloaded BF16 and NF4 both fit the validated single-device memory envelope.
        minimum = 10. if args.precision == "nf4" or args.cpu_offload else 22.5
        minimums = [10.5, 9.8, 4.5] if sharded else [minimum]
        try:
            status["headroom_samples"] = []
            ready_at = time.monotonic() + args.headroom_seconds
            while True:
                free = check_memory([*model_gpus, args.render_gpu], [*minimums, 9.8])
                status["headroom_samples"].append(dict(unix_time=time.time(), free_gib=free))
                save()
                if time.monotonic() >= ready_at:
                    break
                time.sleep(min(5., max(0., ready_at-time.monotonic())))
            status["free_gib_before_start"] = free
            with socket.socket() as probe:
                probe.settimeout(1.)
                if probe.connect_ex(("127.0.0.1", 2000)) == 0:
                    raise RuntimeError("CARLA port2000 is already listening; ownership not established")
        except Exception as error:
            status.update(state="blocked_before_start", error=repr(error))
            save()
            raise
        try:
            subprocess.run(server + ["start"], env=env, cwd=ROOT, check=True)
            import carla
            for attempt in range(60):
                try:
                    client = carla.Client("127.0.0.1", 2000)
                    client.set_timeout(2.)
                    client.get_server_version()
                    break
                except RuntimeError:
                    time.sleep(1.)
            else:
                raise RuntimeError("Owned CARLA server did not become ready")
            for job in jobs:
                command = job["command"]
                def value(flag):
                    return Path(command[command.index(flag)+1])
                verify_manifest(args.manifest, value("--output"), value("--config"), value("--options"))
                if value("--output").exists():
                    raise ValueError("Capture output already exists")
                status.update(state="running", current=job["name"])
                save()
                print("START " + job["name"], flush=True)
                launched = command + (["--cpu-offload"] if args.cpu_offload else [])
                started = time.monotonic()
                with (output / (job["name"] + ".log")).open("w") as stream:
                    try:
                        completed = subprocess.run(launched, env=env, cwd=ROOT, stdout=stream,
                            stderr=subprocess.STDOUT, timeout=args.wall_timeout_s)
                        returncode, timed_out = completed.returncode, False
                    except subprocess.TimeoutExpired:
                        returncode, timed_out = 124, True
                row = dict(name=job["name"], returncode=returncode, timed_out=timed_out,
                           wall_seconds=time.monotonic()-started)
                if returncode == 0:
                    with (output / (job["name"] + "-audit.log")).open("w") as stream:
                        audit = subprocess.run(job["audit_command"], env=env, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
                    row["audit_returncode"] = audit.returncode
                    audit_path = value("--output") / "projection-audit.json"
                    if audit_path.exists():
                        row["audit"] = json.loads(audit_path.read_text())
                status["results"].append(row)
                save()
                print(json.dumps(dict(finished=job["name"], returncode=row["returncode"],
                                      passed=row.get("audit", {}).get("passed"))), flush=True)
                if returncode != 0:
                    break
            status.update(state="completed" if len(status["results"]) == len(jobs) else "stopped_after_error", current=None)
        except BaseException as error:
            status.update(state="failed", error=repr(error))
            raise
        finally:
            status["server_stop_returncode"] = subprocess.run(server + ["stop"], env=env, cwd=ROOT).returncode
            save()


if __name__ == "__main__":
    main()
