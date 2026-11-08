"""Paired, scene-clustered analysis of raw-plan quantization sensitivity."""
from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np

from local.experiments.quant_explore import OLD, VARIANTS
from local.experiments.model_score_benchmark import write_json


def cluster_interval(values, scene_labels, repetitions=20000):
    """Resample scene means, not correlated snapshots or repeated seeds."""
    scenes = sorted(set(scene_labels))
    means = np.array([np.mean([v for v, s in zip(values, scene_labels) if s == scene]) for scene in scenes])
    rng = np.random.default_rng(20261107)
    samples = means[rng.integers(len(means), size=(repetitions, len(means)))].mean(axis=1)
    return dict(mean=float(means.mean()), ci95=np.quantile(samples, [.025, .975]).tolist(),
                scene_clusters=len(scenes), matched_predictions=len(values),
                degenerate_bootstrap=bool(np.ptp(means) == 0),
                uncertainty_note="With no observed scene variation, bootstrap spread is zero; this does not imply zero population uncertainty." if np.ptp(means) == 0 else None)


def original_statistics(output):
    records = {v: json.loads((OLD / v / "scores.json").read_text()) for v in ("bf16", "nf4")}
    lookup = {v: {(r["case"], r["seed"]): r for r in rows} for v, rows in records.items()}
    metrics = ("route_rmse_m_3s", "route_rmse_m_6s", "any_corner_outside_6s", "recorded_reference_ade_m_6s")
    intervals = {}
    for metric in metrics:
        delta = [float(lookup["nf4"][k][metric]) - float(r[metric]) for k, r in lookup["bf16"].items()]
        intervals[metric] = cluster_interval(delta, [k[0] for k in lookup["bf16"]])
    seed_distance, quant_distance = [], []
    for case in sorted({k[0] for k in lookup["bf16"]}):
        trajectories = {}
        for name, seed in lookup["bf16"]:
            if name != case:
                continue
            label = f"{case}_seed{seed}.npz"
            trajectories[seed] = np.load(OLD / "bf16" / label)["trajectory_ego"][:, :2]
            other = np.load(OLD / "nf4" / label)["trajectory_ego"][:, :2]
            quant_distance.append(np.linalg.norm(other - trajectories[seed], axis=1).mean())
        for a, b in itertools.combinations(trajectories.values(), 2):
            seed_distance.append(np.linalg.norm(a - b, axis=1).mean())
    result = dict(nf4_minus_bf16=intervals,
        bf16_different_seed_mean_displacement_m=float(np.mean(seed_distance)),
        matched_nf4_mean_displacement_m=float(np.mean(quant_distance)),
        displacement_horizon_s=6.4,
        note="Ten initial observations; six seeds. Inter-seed distance is stochastic variation, not an accuracy measure. Historical-reference ADE does not establish improved driving.")
    write_json(output / "original-statistics.json", result)
    return result


def supplementary(raw, case):
    from local.experiments.validated_selection import pose_transform, predicted_corners
    velocity = np.asarray(raw["signed_velocity_mps"])[:30]
    result = dict(native_min_speed_mps_3s=float(velocity.min()), native_max_speed_mps_3s=float(velocity.max()))
    if case["mission"] is None:
        result["moving_native_speed_band_ok_3s"] = bool(np.all((velocity >= 1) & (velocity <= 5.5)))
    else:
        line = case["mission"]["stop_line"]
        angle = np.radians(line["yaw"])
        corners = predicted_corners(pose_transform(case["context"]["world_pose"]),
                                    np.array(raw["trajectory_ego"]), np.array(raw["candidate_yaw_rad"])[0],
                                    case["context"]["vehicle_bounds"])
        gaps = -np.max((corners[:30, :, :2] - [line["x"], line["y"]]) @ [np.cos(angle), np.sin(angle)], axis=1)
        result["red_planned_stop_final_2s"] = bool(np.all(gaps >= 0) and
            np.all((gaps[9:] >= 0) & (gaps[9:] <= 8)) and np.all(np.abs(velocity[9:]) <= .2))
    return result


def plot_results(output, result, cases):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = [v for v in VARIANTS if v in result["summary"]]
    labels = {"bf16": "BF16", "nf4": "NF4 weights", "nf4_a8": "NF4 + A8*", "nf4_a4": "NF4 + A4*"}
    colors = {"bf16": "#234b75", "nf4": "#279b8e", "nf4_a8": "#dcaa34", "nf4_a4": "#ca5b54"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.8))
    measures = [("route_rmse_m_3s", "Route centerline RMSE at 3 s (m)"),
                ("any_corner_outside_3s", "Plans leaving the lane corridor at 3 s (%)"),
                ("moving_progress_m_3s", "Moving-scene route progress at 3 s (m)"),
                ("recorded_reference_ade_m_3s", "Historical-path ADE at 3 s (m)")]
    for ax, (key, title) in zip(axes.flat, measures):
        values = [(result["summary"][v].get(key) if result["summary"][v].get(key) is not None else np.nan)
                  * (100 if key == "any_corner_outside_3s" else 1) for v in names]
        bars = ax.bar([labels[v] for v in names], values, color=[colors[v] for v in names], width=.65)
        digits = 3 if key in ("route_rmse_m_3s", "recorded_reference_ade_m_3s") else 2
        ax.bar_label(bars, labels=[f"{v:.{digits}f}" if np.isfinite(v) else "" for v in values], padding=4, fontsize=10)
        ax.set_title(title, fontsize=11)
        upper = max([v for v in values if np.isfinite(v)] or [1.]) * 1.2 + .01
        ax.set_ylim(0, upper)
        for i, value in enumerate(values):
            if not np.isfinite(value):
                ax.text(i, upper*.03, "N/A", ha="center", fontsize=10)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
    validity = "; ".join(f"{labels[v]} {result['summary'][v]['valid']}/{result['summary'][v]['attempted']} valid"
                        + (f" ({result['summary'][v]['not_attempted']} not attempted)" if result['summary'][v]['not_attempted'] else "") for v in names)
    fig.suptitle("Raw Alpamayo plans on fixed CARLA maneuver observations", fontsize=15)
    fig.text(.5, .045, validity + "\n*Activation error emulation, BF16 compute. Historical path is controller-assisted, not expert ground truth.\n30 snapshots from 10 scenes × 2 seeds. No downstream controller in these scores.", ha="center", fontsize=9)
    fig.tight_layout(rect=[0, .11, 1, .95])
    fig.savefig(output / "comparison.png", dpi=180)
    fig.savefig(output / "comparison.pdf")
    plt.close(fig)
    if "nf4" not in result["comparisons"]:
        return
    pairs = sorted(result["comparisons"]["nf4"]["paired_predictions"], key=lambda p:p["mean_displacement_m_3s"], reverse=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ax, pair in zip(axes, pairs[:2]):
        case, seed = cases[pair["case"]], pair["seed"]
        folder = Path(case["source"]).parent
        reference = np.load(folder / "recorded_reference_world.npy")
        origin = np.load(output / "bf16" / f"{pair['case']}_seed{seed}.npz")["world"][0, :2]
        route = json.loads((folder / "route.json").read_text())
        points = np.array([[p["x"], p["y"]] for p in route["points"]]) - origin
        ax.plot(points[:, 0], points[:, 1], color=".7", linestyle="--", label="Commanded centerline")
        ax.plot(reference[:, 0] - origin[0], reference[:, 1] - origin[1], color=".3", linestyle=":", label="Historical driven path")
        trajectories = [reference[:, :2] - origin]
        for variant in ("bf16", "nf4"):
            world = np.load(output / variant / f"{pair['case']}_seed{seed}.npz")["world"][:30, :2] - origin
            trajectories.append(world)
            ax.plot(world[:, 0], world[:, 1], color=colors[variant], linewidth=2, label=labels[variant])
            ax.scatter(world[-1, 0], world[-1, 1], color=colors[variant], s=25)
        visible = np.concatenate(trajectories)
        low, high = visible.min(axis=0)-2, visible.max(axis=0)+2
        midpoint, half_span = (low+high)/2, max(high-low)/2
        ax.set_xlim(midpoint[0]-half_span, midpoint[0]+half_span)
        ax.set_ylim(midpoint[1]-half_span, midpoint[1]+half_span)
        ax.set_aspect("equal")
        ax.set_title(f"{pair['case']}, seed {seed}\nNF4–BF16 mean displacement: {pair['mean_displacement_m_3s']:.2f} m", fontsize=10)
        ax.set_xlabel("World x offset (m)"); ax.set_ylabel("World y offset (m)")
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle("Two largest NF4 trajectory changes in the 3 s window\nSelected after scoring for illustration; all 60 pairs remain in the analysis", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, .91])
    fig.savefig(output / "largest-nf4-changes.png", dpi=180)
    fig.savefig(output / "largest-nf4-changes.pdf")
    plt.close(fig)


def analyze(output):
    old = original_statistics(output)
    protocol = json.loads((output / "protocol.json").read_text())
    cases = {c["name"]: c for c in protocol["cases"]}
    records, lookups = {}, {}
    for variant in VARIANTS:
        if not (output / variant / "scores.json").exists():
            continue
        rows = json.loads((output / variant / "scores.json").read_text())
        for row in rows:
            raw_file = output / variant / f"{row['case']}_seed{row['seed']}.json"
            if row["valid"]:
                row.update(supplementary(json.loads(raw_file.read_text()), cases[row["case"]]))
        records[variant] = rows
        lookups[variant] = {(r["case"], r["seed"]): r for r in rows}
    summaries = {}
    for variant, rows in records.items():
        valid = [r for r in rows if r["valid"]]
        summary = dict(attempted=len(rows), valid=len(valid), invalid=len(rows)-len(valid),
                       not_attempted=len(cases)*len(protocol["seeds"])-len(rows),
                       complete=(output / variant / "complete.json").exists())
        keys = sorted({k for r in valid for k in r if k.endswith(("_1s", "_3s", "_2s"))})
        for key in keys:
            summary[key] = float(np.mean([r[key] for r in valid if key in r]))
        for key in ("inference_seconds", "peak_allocated_gib"):
            values = [r[key] for r in rows if key in r]
            summary[f"p50_{key}"] = float(np.median(values)) if values else None
            summary[f"p95_{key}"] = float(np.percentile(values, 95)) if values else None
            summary[f"max_{key}"] = float(max(values)) if values else None
        moving = [r for r in valid if not r["scene"].startswith("10-")]
        red = [r for r in valid if r["scene"].startswith("10-")]
        summary["moving_progress_m_3s"] = float(np.mean([r["route_progress_m_3s"] for r in moving])) if moving else None
        summary["corner_departure_count_3s"] = sum(r["any_corner_outside_3s"] for r in valid)
        summary["red_crossing_count_3s"] = sum(r["red_stop_line_crossed_3s"] for r in red)
        summary["red_count"] = len(red)
        summary["red_planned_stop_count"] = sum(r["red_planned_stop_final_2s"] for r in red)
        report = json.loads((output / variant / "precision.json").read_text())
        summary["cpu_offload"] = report["cpu_offload"]
        summaries[variant] = summary
    metrics = ("route_rmse_m_3s", "any_corner_outside_3s", "recorded_reference_ade_m_3s",
               "recorded_reference_fde_m_3s", "route_progress_m_3s", "minimum_corner_margin_m_3s")
    comparisons = {}
    if "bf16" in records:
        for variant in records:
            if variant == "bf16":
                continue
            pairs = []
            for key, baseline in lookups["bf16"].items():
                other = lookups[variant].get(key)
                if other is None or not (baseline["valid"] and other["valid"]):
                    continue
                assert baseline["input_sha256"] == other["input_sha256"]
                # Actual diffusion noise is created on local cuda:0. Sharding adds
                # devices but all are reset independently to the same flow seed.
                assert baseline["diffusion_rng"][0] == other["diffusion_rng"][0]
                assert len(set(baseline["diffusion_rng"] + other["diffusion_rng"])) == 1
                label = f"{key[0]}_seed{key[1]}"
                a = np.load(output / "bf16" / (label + ".npz"))["trajectory_ego"][:, :2]
                b = np.load(output / variant / (label + ".npz"))["trajectory_ego"][:, :2]
                distance = np.linalg.norm(a - b, axis=1)
                raw_a = json.loads((output / "bf16" / (label + ".json")).read_text())
                raw_b = json.loads((output / variant / (label + ".json")).read_text())
                pairs.append(dict(case=key[0], scene=baseline["scene"], seed=key[1],
                    mean_displacement_m_3s=float(distance[:30].mean()),
                    mean_displacement_m_6_4s=float(distance.mean()),
                    final_displacement_m_3s=float(distance[29]),
                    final_displacement_m_6_4s=float(distance[-1]),
                    identical_tokens=raw_a["generated_token_ids"] == raw_b["generated_token_ids"],
                    **{m: float(other[m]) - float(baseline[m]) for m in metrics}))
            comparisons[variant] = dict(paired_predictions=pairs, excluded_pairs=len(lookups["bf16"])-len(pairs),
                paired_scene_intervals={m: cluster_interval([p[m] for p in pairs], [p["scene"] for p in pairs])
                    for m in (*metrics, "mean_displacement_m_3s", "mean_displacement_m_6_4s")},
                identical_tokens=sum(p["identical_tokens"] for p in pairs),
                lane_discordance=dict(worse=sum(p["any_corner_outside_3s"] > 0 for p in pairs),
                                      better=sum(p["any_corner_outside_3s"] < 0 for p in pairs),
                                      unchanged=sum(p["any_corner_outside_3s"] == 0 for p in pairs)),
                displacement_by_reasoning_token_match={str(match): dict(
                    count=sum(p["identical_tokens"] == match for p in pairs),
                    mean_displacement_m_3s=float(np.mean([p["mean_displacement_m_3s"] for p in pairs if p["identical_tokens"] == match])))
                    for match in (True, False) if any(p["identical_tokens"] == match for p in pairs)}) if pairs else dict(paired_predictions=[], excluded_pairs=len(lookups["bf16"]))
    seed_variation = []
    if "bf16" in records:
        for case in cases:
            if not all(lookups["bf16"].get((case, seed), {}).get("valid") for seed in (42, 43)):
                continue
            a = np.load(output / "bf16" / f"{case}_seed42.npz")["trajectory_ego"][:, :2]
            b = np.load(output / "bf16" / f"{case}_seed43.npz")["trajectory_ego"][:, :2]
            distance = np.linalg.norm(a - b, axis=1)
            seed_variation.append(dict(case=case, scene=cases[case]["scene"],
                mean_displacement_m_3s=float(distance[:30].mean()), mean_displacement_m_6_4s=float(distance.mean())))
    result = dict(summary=summaries, comparisons=comparisons, bf16_seed_variation=seed_variation,
                  original_statistics=old, population_limit=protocol["population_limit"],
                  reference_limit=protocol["reference_limit"], timing_limit=protocol["timing_limit"],
                  metrics_note="Trajectory metrics condition on valid predictions; invalid counts and excluded pairs reported separately. Unavailable collision/traffic metrics are not passes.")
    write_json(output / "comparison.json", result)
    if summaries:
        plot_results(output, result, cases)
    flat = [{"variant": v, **r} for v, rows in records.items() for r in rows]
    if flat:
        with (output / "per_inference.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=sorted({k for r in flat for k in r}))
            writer.writeheader()
            writer.writerows(flat)
        with (output / "summary.csv").open("w") as stream:
            rows = [{"variant": v, **r} for v, r in summaries.items()]
            writer = csv.DictWriter(stream, fieldnames=sorted({k for r in rows for k in r}))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(dict(summary=summaries,
        paired={v: {"intervals": c.get("paired_scene_intervals"), "identical_tokens": c.get("identical_tokens")} for v,c in comparisons.items()}), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    analyze(parser.parse_args().output.resolve())
