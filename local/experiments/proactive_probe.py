"""Compare fresh and recent native plans using the same current-state score.

All existing retention bounds apply. Fresh proposals are archived even when a
better cached path is executed. This entrypoint does not edit any trajectory.
"""
import copy
import json

from local.experiments import retained_probe as retained
from local.experiments import inference_probe as native
from local.experiments.validated_selection import choose


class ProactiveProbe(retained.RetainingProbe):
    def evaluate(self, adapter, data, context, instruction, options, label,
                 capture=False, capture_failures=False):
        if options.get("validation_prefer_horizon_s"):
            raise ValueError("Proactive comparison currently requires the common 1.5 s ranking without forecast preference")
        prediction, extra = super().evaluate(adapter, data, context, instruction, options, label,
                                             capture=capture, capture_failures=capture_failures)
        fresh = json.loads((self.output / (label + ".json")).read_text())
        if "retention" in fresh:
            return prediction, extra  # Existing exhaustion handling already chose a cached plan.

        # Give every admitted fresh plan an immutable source file. A root frame
        # may subsequently describe execution of a different, cached sample.
        fresh_plan = self.history[-1]
        source_label = "accepted_" + label
        fresh_plan.record["label"] = source_label
        fresh_plan.record["planning_label"] = label
        native.write_json(self.output / (source_label + ".json"), fresh_plan.record)
        fresh["accepted_source_label"] = source_label
        native.write_json(self.output / (label + ".json"), fresh)

        previous = self.history[:-1]
        metrics = [dict(retained.assess_retained(p, context, self.validation_route,
                                               self.visual_signal, self.navigation_phase, options), candidate=i)
                   for i, p in enumerate(previous)]
        chosen = choose(metrics, [p.record["trajectory_ego"] for p in previous],
                        self.visual_signal, context["measured_speed_mps"],
                        options.get("validation_desired_speed_mps", 5.))
        selection = fresh["selection"]
        attempt = next(a for a in selection["attempts"] if a["proposal"] == selection["proposal"])
        fresh_metric = attempt["metrics"][selection["chosen"]]
        if chosen is None or metrics[chosen]["selection_cost"] >= fresh_metric["selection_cost"]:
            return prediction, extra

        plan = previous[chosen]
        age = context["time_s"] - plan.record["context"]["time_s"]
        record = copy.deepcopy(plan.record)
        record.update(label=label, context=context, navigation_phase=self.navigation_phase,
                      visual_signal=self.visual_signal, visual_signal_answer=self.visual_signal_answer,
                      visual_signal_check_index=self.visual_signal_index,
                      inference_seconds=fresh["inference_seconds"],
                      raw_target_speed_mps=metrics[chosen]["target_speed_mps"],
                      controller_preview_speed_mps=metrics[chosen]["target_speed_mps"],
                      selection={"mode": "retained_native_sample", "chosen": plan.record["selection"]["chosen"],
                                 "proposal": plan.record["selection"]["proposal"],
                                 "route": options["validation_route"], "signal": self.visual_signal},
                      retention={"reason": "cached_plan_ranked_better", "source_label": plan.record["label"],
                                 "source_context": plan.record["context"], "age_s": age,
                                 "chosen_cache_index": chosen, "metrics": metrics,
                                 "fresh_candidate_source": source_label,
                                 "fresh_selection_cost": fresh_metric["selection_cost"]})
        native.write_json(self.output / (label + ".json"), record)
        self.executing_plan, self.executing_age = plan, age
        print(json.dumps({"retained": label, "source": plan.record["label"], "age_s": age,
                          "reason": "cached_plan_ranked_better", "fresh_target_mps": fresh_metric["target_speed_mps"],
                          "target_speed_mps": metrics[chosen]["target_speed_mps"]}), flush=True)
        return plan.prediction, copy.deepcopy(plan.extra)


def main():
    original = retained.RetainingProbe
    retained.RetainingProbe = ProactiveProbe
    try:
        retained.main()
    finally:
        retained.RetainingProbe = original


if __name__ == "__main__":
    main()
