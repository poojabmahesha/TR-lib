
from confetti import Config
from slash.utils.conf_utils import Doc

__all__ = ["config"]

config = Config({
    'treerunner': {
        'ip': '10.2.0.6' // Doc("Treerunner target IP address"),
        'username': 'root' // Doc("Sync target SSH user name"),
        'password': 'root',
    },
    'serial_port': {
        'port': None,
        'baudrate': '115200'
    }
})
