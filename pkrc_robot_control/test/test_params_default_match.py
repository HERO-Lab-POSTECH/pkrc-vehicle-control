"""Source-of-truth invariant: config/pkrc.yaml values must equal
_params.PARAM_DEFAULTS values bit-exact.

Rationale: yaml is the operator-visible surface; PARAM_DEFAULTS is the
declare_parameter fallback. They must agree, otherwise yaml-less
``ros2 run`` and yaml-loaded ``ros2 launch`` produce different behavior.
"""
from pathlib import Path

import yaml

from pkrc_robot_control._params import PARAM_DEFAULTS


def _yaml_path() -> Path:
    here = Path(__file__).resolve().parent
    return here.parent / 'config' / 'pkrc.yaml'


def test_yaml_matches_defaults():
    yaml_data = yaml.safe_load(_yaml_path().read_text())
    yaml_params = yaml_data['hero_main_control']['ros__parameters']

    yaml_keys = set(yaml_params.keys())
    default_keys = set(PARAM_DEFAULTS.keys())

    missing_in_yaml = default_keys - yaml_keys
    extra_in_yaml = yaml_keys - default_keys
    assert not missing_in_yaml, f'yaml missing keys: {sorted(missing_in_yaml)}'
    assert not extra_in_yaml, f'yaml has unexpected keys: {sorted(extra_in_yaml)}'

    mismatches = []
    for key, default in PARAM_DEFAULTS.items():
        if yaml_params[key] != default:
            mismatches.append((key, yaml_params[key], default))
    assert not mismatches, (
        'yaml/default mismatches (key, yaml, default): '
        + ', '.join(f'{k}: {y!r} vs {d!r}' for k, y, d in mismatches)
    )


def test_param_count_matches_spec():
    """Sentinel: spec says 52 params. Catches accidental key add/remove."""
    assert len(PARAM_DEFAULTS) == 52, f'expected 52 params, got {len(PARAM_DEFAULTS)}'
