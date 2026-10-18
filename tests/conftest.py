"""Run `async def` tests without pytest-asyncio.

The web-host tests (tests/test_pi_runtime.py) are coroutines marked
`pytest.mark.asyncio`. Where pytest-asyncio is not installed (the WSL ROS
environment) pytest skipped all ten of them silently, so the Pi web host's
own regression never ran. With the plugin present this hook stands aside.
"""
import asyncio
import inspect

import pytest

try:
    import pytest_asyncio  # noqa: F401
    HAVE_PLUGIN = True
except ImportError:
    HAVE_PLUGIN = False


def pytest_configure(config):
    config.addinivalue_line("markers", "asyncio: run the coroutine test in an event loop")


@pytest.hookimpl(tryfirst=True)
def pytest_pyfunc_call(pyfuncitem):
    if HAVE_PLUGIN or not inspect.iscoroutinefunction(pyfuncitem.obj):
        return None
    names = pyfuncitem._fixtureinfo.argnames
    kwargs = {name: pyfuncitem.funcargs[name] for name in names}
    asyncio.run(pyfuncitem.obj(**kwargs))
    return True
