"""Booting the saved hardware profile must not require a browser or a props checkbox."""
import asyncio
import json
from types import SimpleNamespace

import pytest

from aerothon_runtime.contracts import profile
from aerothon_runtime.web import Host


@pytest.mark.asyncio
async def test_boot_connects_unreviewed_drone_without_a_browser(tmp_path):
    state = tmp_path / 'state'
    state.mkdir()
    drone = profile(dict(id='drone', transport='serial', device='/dev/ttyAMA2',
                         hardware_reviewed=False))
    (state / 'profiles.json').write_text(json.dumps({'drone': drone}))
    launches = []

    async def launch(p):
        launches.append(p)
        return SimpleNamespace(returncode=None)

    h = Host(tmp_path, state, launch=launch, aggregator='http://127.0.0.1:1')
    await h.start(None)
    try:
        for _ in range(20):
            if launches:
                break
            await asyncio.sleep(.01)
        assert len(launches) == 1, 'boot left the Pi waiting for GCS Connect'
        assert h.profile_id == 'drone'
        assert not h.controlled, 'connecting must not start aircraft execution'
        assert h.execution is None
    finally:
        await h.close(None)


def test_hardware_autoconnect_default_and_explicit_opt_out():
    assert profile(dict(id='drone', transport='serial', device='/dev/ttyAMA2')).get('autoconnect') is True
    assert profile(dict(id='drone', transport='serial', device='/dev/ttyAMA2', autoconnect=False))['autoconnect'] is False
    assert profile(dict(id='sim', host='laptop', gateway='http://laptop:8090')).get('autoconnect') is False


def test_selected_profile_and_opt_out_survive_web_restarts(tmp_path):
    h=Host(tmp_path,tmp_path/'state')
    h.profiles={'a':profile(dict(id='a',transport='serial',device='/dev/ttyAMA2')),
                'b':profile(dict(id='b',transport='serial',device='/dev/ttyAMA3',autoconnect=False))}
    h.write_profiles();h.select_connection('b')
    restarted=Host(tmp_path,tmp_path/'state')
    assert restarted.boot_profile() is None
    restarted.select_connection('a')
    assert restarted.boot_profile()['id']=='a'


def test_router_boot_does_not_depend_on_a_profile_or_props_checkbox():
    import importlib.util
    from pathlib import Path
    source=Path(__file__).resolve().parents[1]/'scripts/pi_runtime/profile_router.py'
    spec=importlib.util.spec_from_file_location('tested_profile_router',source)
    router=importlib.util.module_from_spec(spec);spec.loader.exec_module(router)
    env={'AEROTHON_FCU_DEVICE':'/dev/ttyAMA2','AEROTHON_FCU_BAUD':'115200'}
    assert router.routing_settings({},env)==('/dev/ttyAMA2',115200,'',True)
    assert router.routing_settings({'transport':'serial','device':'/dev/ttyAMA2','hardware_reviewed':False},env)==('/dev/ttyAMA2',115200,'',True)
    assert router.routing_settings({'transport':'tcp','vehicle':'simulation'},env) is None
