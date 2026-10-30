"""Opt-in capture choosing only original model samples that pass route checks.

No trajectory is repaired or replaced with map waypoints. Exhausted sampling
terminates the experiment explicitly. Lane checks do not model obstacles.
"""
import copy
import json
import math
import sys

import numpy as np

from local.experiments import inference_probe as native
from local.experiments.audit_turn import project
from local.experiments.validated_selection import assess, choose, require_green_progress, selection_rank
from module.inference import extract_trajectory_samples
from module.trajectory_follower import planned_speeds


class RouteExitReached(RuntimeError):
    def __init__(self, output, context):
        super().__init__("Configured route exit reached; capture ends before the next maneuver")
        self.output, self.context = output, context


class NoValidatedPlan(RuntimeError):
    """All new model samples failed admission; distinct from inference errors."""


class ValidatingProbe(native.Probe):
    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        if not context.get("closed_loop"):
            raise ValueError("Use this wrapper for live capture; audit frozen pools separately")
        if not options.get("verify_signal"):
            raise ValueError("Validated sampling requires the image-based signal check")
        if not hasattr(self, "validation_route"):
            from pathlib import Path
            routes = json.loads(Path(options["validation_routes"]).read_text())
            self.validation_route = routes["routes"][options["validation_route"]]
            self.navigation_phase = "approach"
            if routes["junction_id"] != context["route_entry"]["junction_id"]:
                raise ValueError("Validation route and capture junction differ")
            native.write_json(self.output / "route-reference.json", {
                **routes, "note": "Used to validate and select native model samples; never substituted as an executed path."})
        route = self.validation_route
        phase = self.navigation_phase
        pose = context["world_pose"]
        distance, arc, heading, width, exit_arc = project(np.array([[pose["x"], pose["y"]]]), route)
        aligned = abs((pose["yaw"] - heading[0] + 180) % 360 - 180) <= 20
        end = route["exit"]
        exit_yaw = math.radians(end["yaw"])
        beyond = ((pose["x"] - end["x"]) * math.cos(exit_yaw) +
                  (pose["y"] - end["y"]) * math.sin(exit_yaw)) >= 0
        reached = beyond and arc[0] >= exit_arc and distance[0] <= width[0] and aligned
        if reached and options.get("validation_stop_at_exit", False):
            native.write_json(self.output / "route-exit-reached.json", {
                "context": context, "route": options["validation_route"],
                "center_distance_m": float(distance[0]),
                "note": "Reaching the exit does not certify the preceding path or stopping behavior."})
            raise RouteExitReached(self.output, context)
        if options["validation_route"] == "left":
            turn = (route["entry"]["yaw"] - pose["yaw"] + 180) % 360 - 180
            if reached:
                phase = "exit"
            elif phase == "approach" and turn >= 60:
                phase = "finish_turn"
            if phase == "finish_turn":
                instruction = "Finish the left turn, then continue straight along the exit lane."
            elif phase == "exit":
                instruction = "Continue straight along the current lane."
        else:
            instruction = options.get("instruction", instruction)
        self.navigation_phase = phase
        root = self.output
        proposals = root / "proposals"
        proposals.mkdir(exist_ok=True)
        attempts, best = [], None
        quality_limit = options.get("validation_resample_above_target_mps")
        preferred_reserve = options.get("validation_preferred_reserve_m")
        if quality_limit is not None and (not math.isfinite(quality_limit) or quality_limit <= 0):
            raise ValueError("The resampling target preference must be finite and positive")
        for attempt in range(options.get("validation_max_pools", 3)):
            trial = {**options, "instruction": instruction,
                     "flow_seed": options.get("flow_seed", 42) + attempt,
                     "seed": options.get("seed", context.get("seed", 42)),
                     "select_route_continuity": False}
            if attempt and (options.get("validation_resample_language", False) or
                            all(not m["marker_present"] for m in attempts[-1]["metrics"])):
                trial["seed"] += attempt
            temperatures = options.get("validation_flow_temperatures")
            if temperatures:
                trial["flow_temperature"] = temperatures[min(attempt, len(temperatures) - 1)]
            proposal_label = f"{label}_pool_{attempt}"
            self.output = proposals
            try:
                prediction, extra = super().evaluate(
                    adapter, data, context, instruction, trial, proposal_label,
                    capture=capture and attempt == 0, capture_failures=capture_failures)
            finally:
                self.output = root
            record = json.loads((proposals / (proposal_label + ".json")).read_text())
            candidates = extract_trajectory_samples(prediction)
            metrics = assess(candidates, context, route, record["candidate_markers"],
                             record["candidate_velocities"], options.get("validation_horizon_s", 3.),
                             yaws=record.get("candidate_yaw_rad"),
                             check_body=options.get("validation_check_body", False),
                             body_margin_m=options.get("validation_body_margin_m", 0.))
            require_green_progress(metrics, self.visual_signal,
                                   options.get("validation_green_min_target_mps", 0.))
            preferred_horizon = options.get("validation_prefer_horizon_s")
            if preferred_horizon and self.visual_signal == "green":
                longer = assess(candidates, context, route, record["candidate_markers"],
                                record["candidate_velocities"], preferred_horizon,
                                yaws=record.get("candidate_yaw_rad"),
                                check_body=options.get("validation_check_body", False),
                                body_margin_m=options.get("validation_body_margin_m", 0.))
                for item, forecast in zip(metrics, longer):
                    item.update(preferred_horizon_valid=forecast["valid"],
                                preferred_horizon_s=forecast["horizon_s"],
                                preferred_horizon_reasons=forecast["reasons"])
            chosen = choose(metrics, candidates, self.visual_signal,
                            context["measured_speed_mps"], options.get("validation_desired_speed_mps", 5.))
            attempts.append({"proposal": proposal_label, "metrics": metrics,
                             "flow_seed": trial["flow_seed"], "seed": trial.get("seed", 43),
                             "flow_temperature": trial.get("flow_temperature", 1.),
                             "inference_seconds": record["inference_seconds"], "chosen": chosen})
            if chosen is None:
                continue
            # Retry a fast but admitted selection only when explicitly enabled.
            # Preserve the best admitted native sample even if later pools fail.
            metric = metrics[chosen]
            if preferred_reserve is not None and "preferred_reserve_valid" not in metric:
                raise ValueError("Preferred reserve requires tracking_probe's native and tracking annotations")
            key = selection_rank(metric)
            if best is None or key < best["key"]:
                best = dict(key=key, pool=attempt, chosen=chosen, record=record, metric=metric,
                            selected=prediction[:, :, chosen:chosen + 1].detach().clone(),
                            extra=copy.deepcopy({
                                k: v[..., chosen:chosen + 1] if isinstance(v, np.ndarray) else v
                                for k, v in extra.items()}))
            retry_speed = (quality_limit is not None and self.visual_signal == "green" and
                           best["metric"]["target_speed_mps"] > quality_limit)
            retry_reserve = (preferred_reserve is not None and
                             not best["metric"]["preferred_reserve_valid"])
            if quality_limit is not None:
                attempts[-1]["retry_for_speed_preference"] = retry_speed
            if preferred_reserve is not None:
                attempts[-1]["retry_for_reserve_preference"] = retry_reserve
            if not (retry_speed or retry_reserve):
                break
        if best is not None:
            selected, extra, record = best["selected"], best["extra"], best["record"]
            chosen, proposal_label = best["chosen"], best["record"]["label"]
            trajectory = extract_trajectory_samples(selected)[0]
            tokens = record["candidate_generated_tokens"][chosen]
            velocities = record["candidate_velocities"][chosen]
            speed = planned_speeds(trajectory)
            start = round(native.runner.defaults.PID_SPEED_LEAD_SEC / .1)
            end = round((native.runner.defaults.PID_SPEED_LEAD_SEC + native.runner.defaults.PID_SPEED_PREVIEW_SEC) / .1)
            record.update(label=label, navigation_phase=phase, instruction=instruction,
                          inference_seconds=sum(a["inference_seconds"] for a in attempts),
                          trajectory_ego=trajectory.tolist(), reasoning=adapter.extract_cot_text(extra),
                          generated_token_ids=tokens, generated_tokens=len(tokens),
                          raw_generated_text=self.model.tokenizer.decode(tokens, skip_special_tokens=False),
                          trajectory_marker_present=record["candidate_markers"][chosen],
                          trajectory_marker_generated=self.model.tokenizer.convert_tokens_to_ids("<|traj_future_start|>") in tokens,
                          signed_velocity_mps=velocities,
                          acceleration_mps2=record["candidate_accelerations"][chosen],
                          raw_target_speed_mps=float(speed[10:15].mean()),
                          controller_preview_speed_mps=float(speed[start:end].mean()),
                          first_second_speed_mps=float(speed[:10].mean()),
                          end_forward_m=float(trajectory[-1, 0]), end_left_m=float(trajectory[-1, 1]),
                          reverse_velocity_steps=sum(v < -.05 for v in velocities),
                          selection={"mode": "validated_native_sample", "chosen": chosen,
                                     "proposal": proposal_label, "attempts": attempts,
                                     "route": options["validation_route"],
                                     "signal": self.visual_signal})
            if quality_limit is not None:
                record["selection"]["speed_quality_preference"] = {
                    "resample_above_target_mps": quality_limit,
                    "applied": self.visual_signal == "green",
                    "met": best["metric"]["target_speed_mps"] <= quality_limit,
                    "note": "Retry preference only; an admitted native plan may remain above it when the budget is exhausted."}
            if preferred_reserve is not None:
                record["selection"]["reserve_preference"] = {
                    "preferred_clearance_m": preferred_reserve,
                    "minimum_native_clearance_m": options["validation_body_margin_m"],
                    "minimum_tracking_clearance_m": options["validation_tracking_margin_m"],
                    "met": best["metric"]["preferred_reserve_valid"],
                    "note": "Search the existing pool budget for the preferred reserve; otherwise retain the best sample meeting both hard minima."}
            native.write_json(root / (label + ".json"), record)
            print(json.dumps({"selected": label, "pool": best["pool"], "candidate": chosen,
                              "navigation_phase": phase, "signal": self.visual_signal,
                              "target_speed_mps": record["raw_target_speed_mps"],
                              "max_predicted_lane_distance_m": best["metric"]["max_lane_distance_m"]}), flush=True)
            return selected, extra
        native.write_json(root / "validation-failure.json", {
            "label": label, "context": context, "navigation_phase": phase,
            "instruction": instruction, "attempts": attempts,
            "reason": "No model trajectory passed validation; no fallback path was executed."})
        inputs = root / "inputs"
        inputs.mkdir(exist_ok=True)
        native.torch.save({"data": {key: value.detach().cpu().clone() for key, value in data.items()},
                           "context": context, "instruction": instruction}, inputs / (label + ".pt"))
        raise NoValidatedPlan(f"No valid model-generated plan at {label} after {len(attempts)} pools")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "replay":
        raise SystemExit("Use inference_probe replay, then validated_selection to audit frozen candidates")
    original = native.Probe
    native.Probe = ValidatingProbe
    try:
        native.main()
    except RouteExitReached as done:
        # The runner's finally block has already closed videos and restored CARLA.
        # Only this explicit completion signal is converted to a normal result.
        path = done.output / "000_verifai" / "summary.json"
        summary = json.loads(path.read_text())
        summary.pop("error", None)
        summary.update(termination="route_exit_reached", final_state={
            **done.context["world_pose"], "speed": done.context["measured_speed_mps"]})
        native.write_json(path, summary)
        native.write_json(done.output / "batch-summary.json", [summary])
        print(json.dumps({"termination": "route_exit_reached", "time_s": done.context["time_s"]}), flush=True)
    finally:
        native.Probe = original


if __name__ == "__main__":
    main()
