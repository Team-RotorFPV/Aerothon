"""Process ownership for the single Pi flight commander."""
import fcntl
import os
from pathlib import Path

def acquire(path=None):
    path = Path(path or Path(os.environ.get('AEROTHON_STATE_DIR', Path.home()/'.local/state/aerothon'))/'command-owner.lock')
    path.parent.mkdir(parents=True,exist_ok=True)
    handle=path.open('a')
    try:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('another production commander already owns this Pi') from None
    return handle
