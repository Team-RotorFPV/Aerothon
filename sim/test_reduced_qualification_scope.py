"""Scope exclusions preserve moderate weather and physical actuator response."""
import ast
from pathlib import Path
from run_navigation_campaign import reduced_weather_exclusion


def test_calm_and_moderate_field_weather_remain_in_scope():
    for preset in ('calm','field'):
        for seed in range(1,6):assert reduced_weather_exclusion(preset,seed) is None


def test_strong_preset_is_explicitly_excluded():
    for seed in range(1,6):
        exclusion=reduced_weather_exclusion('worst',seed)
        assert exclusion and exclusion['mean_wind_mps']==8 and exclusion['gust_mps']==4


def test_random_weather_is_selected_by_concrete_wind_not_label():
    exclusions=[reduced_weather_exclusion('random',seed) for seed in range(1,6)]
    assert any(r is None for r in exclusions) and any(r is not None for r in exclusions)
    for r in exclusions:
        if r:assert r['mean_wind_mps']>4 or r['gust_mps']>2


def test_no_extra_delay_option_preserves_normal_actuator_default():
    tree=ast.parse((Path(__file__).parent/'headless_world.py').read_text())
    response=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and n.args
              and isinstance(n.args[0],ast.Constant) and n.args[0].value=='--vel-response-s']
    assert len(response)==1
    assert next(k.value.value for k in response[0].keywords if k.arg=='default')==.6
