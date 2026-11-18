"""Explicit single-sample policies for the no-map driving experiment."""
from contextlib import contextmanager

SEED_POLICIES = ('fixed', 'advancing_separate', 'advancing_joint')


def _seed(value, name):
    # NumPy's legacy RNG accepts uint32; reject overflow instead of silently wrapping.
    if type(value) is not int or not 0 <= value < 2**32:
        raise ValueError(f'{name} must be an integer in [0, 2**32)')
    return value


def resolve_sampling(options, context):
    """Derive seeds from the inference ordinal, never the control tick or outcome.

    Keep requested options intact so replay cannot increment effective seeds twice.
    Joint mode seeds once before language generation and lets diffusion consume
    that stream; existing VQA calls use their own forked RNG.
    """
    mode = options.get('seed_policy', 'fixed')
    if mode not in SEED_POLICIES:
        raise ValueError(f'Unknown seed policy: {mode}')
    effective = dict(options)
    base = _seed(options.get('seed', context.get('seed', 42)), 'seed')
    flow = options.get('flow_seed')
    if flow is not None:
        _seed(flow, 'flow_seed')
    ordinal = options.get('inference_index', context.get('inference_index'))
    if mode != 'fixed':
        if type(ordinal) is not int or ordinal < 0:
            raise ValueError('Advancing seeds require a nonnegative inference_index')
        if options.get('num_samples', 1) != 1 or options.get('speed_selection_attempts', 1) != 1:
            raise ValueError('Advancing seed experiment requires one unselected prediction')
        effective['seed'] = _seed(base + ordinal, 'effective seed')
        if mode == 'advancing_separate':
            if flow is None:
                raise ValueError('Separate streams require an explicit flow_seed')
            effective['flow_seed'] = _seed(flow + ordinal, 'effective flow_seed')
        else:
            effective.pop('flow_seed', None)
    return effective, dict(mode=mode, inference_index=ordinal, base_seed=base,
                          base_flow_seed=flow, effective_seed=effective.get('seed', base),
                          effective_flow_seed=effective.get('flow_seed'),
                          diffusion_stream='separate' if 'flow_seed' in effective else 'joint')


@contextmanager
def camera_postprocessing(config, options):
    """Scope the one camera change to capture, including exception cleanup."""
    previous = config.CAMERA_ENABLE_POSTPROCESS_EFFECTS
    value = options.get('camera_enable_postprocess_effects', previous)
    if type(value) is not bool:
        raise ValueError('camera_enable_postprocess_effects must be a boolean')
    config.CAMERA_ENABLE_POSTPROCESS_EFFECTS = value
    try:
        yield value
    finally:
        config.CAMERA_ENABLE_POSTPROCESS_EFFECTS = previous
