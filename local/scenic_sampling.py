"""Sample Scenic initial-state parameters for the existing intersection runner.

Scenic owns distributions and initial-state constraints. CARLAMayo owns the
map, actor creation, simulation clock, sensors, and ego policy. The single Scenic
Object is a marker required by ScenicSampler; its pose is not sent to CARLA.
"""
from __future__ import annotations

import math
from numbers import Real

from scenic.core.distributions import needsSampling
from verifai.samplers import ScenicSampler

from local.run_verifai import FIELDS


def parameters_to_spec(parameters):
    """Translate only the four supported parameters into the JSON wire format."""
    spec = {"name": "verifai"}
    for field in FIELDS:
        if field not in parameters:
            raise ValueError(f"Scenic scenario must define param {field}")
        value = parameters[field]
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(f"Scenic param {field} must be a finite number")
        if field in ("distance_before_intersection_m", "speed_mps") and value < 0:
            raise ValueError(f"Scenic param {field} must be non-negative")
        spec[field] = float(value)
    return spec


class IntersectionScenicSampler(ScenicSampler):
    """A real ScenicSampler restricted to static, single-ego initialization."""

    def __init__(self, scenario, maxIterations=None, ignoredProperties=None):
        if scenario.egoObject is None or len(scenario.objects) != 1:
            raise ValueError("Scenic intersection scenarios need exactly one ego Object; "
                             "additional actors are not supported")
        dynamic = scenario.dynamicScenario
        runtime_features = (
            scenario.simulator, scenario.monitors, scenario.terminationConditions,
            scenario.terminateSimulationConditions, scenario.recordedExprs,
            scenario.recordedInitialExprs, scenario.recordedFinalExprs,
            dynamic._compose, dynamic._timeLimit is not None,
            scenario.egoObject.behavior is not None,
        )
        if any(runtime_features) or any(
            req.proposition.has_temporal_operator for req in scenario.requirements
        ):
            raise ValueError("Only Scenic initial-state constraints are supported; "
                             "behaviors, runtime requirements, records, and simulators "
                             "must be handled by CARLAMayo")
        for field in FIELDS:
            if field not in scenario.params:
                raise ValueError(f"Scenic scenario must define param {field}")
        # Fail invalid constants before starting the expensive simulation client.
        constants = {field: (0 if needsSampling(scenario.params[field])
                             else scenario.params[field]) for field in FIELDS}
        parameters_to_spec(constants)
        super().__init__(scenario, maxIterations=maxIterations,
                         ignoredProperties=ignoredProperties)

    def rollout_spec(self, sample):
        return parameters_to_spec(self.paramDictForSample(sample))
