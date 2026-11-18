"""Persistent CarlaMayo simulation client for the JSON VerifAI server adapter."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import socket

from local.rollout_trace import load_rollout_result


def receive_json(connection):
    with connection.makefile("rb") as stream:
        message = stream.readline()
    if not message:
        raise ConnectionError("Peer closed without sending a result")
    return json.loads(message)


def send_json(connection, message):
    connection.sendall((json.dumps(message, allow_nan=False) + "\n").encode())


def run_client(port, simulate, timeout=1200):
    """Send a {summary, trace} result per sample, keeping the model session alive."""
    index = 0
    while True:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout) as connection:
            request = receive_json(connection)
            if request.get("index") != index or type(request.get("last")) is not bool:
                raise ValueError("Unexpected episode request from VerifAI")
            try:
                result = simulate(request["spec"], index)
                send_json(connection, {"index": index, "summary": result["summary"],
                                       "trace": result["trace"]})
            except Exception as exc:
                send_json(connection, {"index": index, "summary": {
                    "termination": "error", "error": f"{type(exc).__name__}: {exc}",
                }, "trace": None})
                raise
        # Closing the client signals EOF to Server.receive(). Letting the client
        # close first also allows immediate reuse of the coordinator's port.
        index += 1
        if request["last"]:
            return index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--verifai-port", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=1200)
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port")
    parser.add_argument("--checkpoint")
    args = parser.parse_args()

    def stop(signum, frame):
        # Let rollout/session finally blocks clean up when the coordinator exits.
        raise SystemExit(128 + signum)
    signal.signal(signal.SIGTERM, stop)

    # Only this process imports CARLA/PyTorch and loads the model.
    from local.run_intersection import rollout_session, rollout_specs, write_json

    config = json.loads(args.config.read_text())
    summaries = []
    with rollout_session(args, config, args.output) as episode:
        def simulate(spec, index):
            rollout_specs({"rollouts": [spec]})
            name = f"{index:03d}_verifai"
            print(f"Starting {name}: {spec}", flush=True)
            summary = episode(spec, args.output / name)
            summaries.append(summary)
            write_json(args.output / "batch-summary.json", summaries)
            return load_rollout_result(args.output / name, summary)

        count = run_client(args.verifai_port, simulate, args.timeout)
    print(f"Client finished {count} rollouts: {args.output}", flush=True)


if __name__ == "__main__":
    main()
