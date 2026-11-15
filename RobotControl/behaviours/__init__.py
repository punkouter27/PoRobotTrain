"""Behaviours that run unchanged on the MuJoCo twin, the Unreal (URLab) twin and the real R1.

Each behaviour declares the joints it powers; every backend leaves all other joints unpowered.
Run with ``python -m robotcontrol.behaviours.runner <behaviour> --target sim|urlab|real``.
"""
