#!/usr/bin/env python3
"""Refuse container shutdown until the web supervisor permits disconnect."""
import json
import urllib.request

try:
    request = urllib.request.Request('http://127.0.0.1:8899/api/disconnect', data=b'{}',
                                     headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request, timeout=25) as response:
        result = json.load(response)
    if result['runtime'] != 'Stopped':
        raise RuntimeError('runtime has not stopped')
except Exception as exc:
    raise SystemExit(f'Runtime shutdown refused; recover and confirm disarm first: {exc}')
