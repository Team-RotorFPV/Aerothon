"""Transmitter calibration. No channel assignment is invented by the GCS."""
from .contracts import number

def validate(config):
    for key in ('forward', 'left', 'yaw', 'climb', 'takeover'):
        item = config.get(key)
        if not isinstance(item, dict):
            raise ValueError(f'configure transmitter {key} channel and calibration')
        item['channel'] = int(number(item.get('channel'), f'{key} channel', 1, 18))
        item['min'] = number(item.get('min'), f'{key} minimum', 800, 2200)
        item['max'] = number(item.get('max'), f'{key} maximum', 800, 2200)
        if item['min'] >= item['max']:
            raise ValueError(f'{key} calibration is inverted')
        if key != 'takeover':
            item['trim'] = number(item.get('trim'), f'{key} trim', item['min'], item['max'])
            if not item['min'] < item['trim'] < item['max']:
                raise ValueError(f'{key} trim must be within endpoints')
    number(config.get('max_speed'), 'manual maximum speed', .1, 2.5)
    number(config.get('max_yaw_rate'), 'manual maximum yaw rate', .05, 1)
    number(config.get('max_climb_rate'), 'manual maximum climb rate', .05, 1)
    number(config['takeover'].get('active_above'), 'takeover threshold', 800, 2200)
    return config

def intent(channels, config):
    def value(key):
        c = config[key]
        index = c['channel'] - 1
        if index >= len(channels) or not c['min']-100 <= channels[index] <= c['max']+100:
            raise ValueError('transmitter channel missing or invalid')
        pwm = channels[index]
        if key == 'takeover':
            return pwm >= c['active_above']
        delta = pwm-c['trim']
        v = delta/(c['max']-c['trim'] if delta >= 0 else c['trim']-c['min'])
        v = 0 if abs(v) < .05 else max(-1., min(1., v))
        return -v if c.get('reversed') else v
    return {'takeover': value('takeover'), 'forward': value('forward')*config['max_speed'],
            'left': value('left')*config['max_speed'], 'yaw': value('yaw')*config['max_yaw_rate'],
            'climb': value('climb')*config['max_climb_rate']}
