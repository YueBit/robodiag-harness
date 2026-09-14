#!/usr/bin/env python3
"""Compatibility shim.

All metadata lives in setup.cfg; this file only exists so that very old
setuptools / `pip install --no-build-isolation` can build the project.
"""
from setuptools import setup

setup()
