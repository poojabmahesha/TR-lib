# -*- coding: utf-8 -*-

"""Top-level package for treerunner-lib."""
from pkg_resources import get_distribution, DistributionNotFound

__author__ = """Flavio Coronel"""
__email__ = 'fcorone2@ford.com'

try:
    __version__ = get_distribution(__name__).version
except DistributionNotFound:
    # package is not installed
    __version__ = 'unknown'
