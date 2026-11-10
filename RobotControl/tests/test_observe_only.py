"""Safety test: observe-only mode must never create a rt/lowcmd publisher or touch the motion switcher."""
import sys
import threading
import types

import numpy as np

from robotcontrol.configs.joint_map import JointMap
from robotcontrol.interfaces.base_interface import MotorCommand
from robotcontrol.interfaces import unitree_interface as ui

CALLS = []


def _install_fake_sdk(monkeypatch):
    class Motor:
        def __init__(self):
            self.q, self.dq, self.tau_est, self.temperature = 0.1, 0.0, 0.0, [30, 30]

    class LowState:
        def __init__(self):
            self.motor_state = [Motor() for _ in range(35)]
            self.mode_machine = 7
            self.crc = 0
            self.imu_state = types.SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[0, 0, 0],
                                                   accelerometer=[0, 0, 9.81], rpy=[0, 0, 0])

    class ChannelSubscriber:
        def __init__(self, topic, typ):
            CALLS.append(("subscriber", topic))

        def Init(self, handler, depth):
            threading.Timer(0.05, lambda: handler(LowState())).start()

        def Close(self):
            pass

    class ChannelPublisher:
        def __init__(self, topic, typ):
            CALLS.append(("PUBLISHER", topic))

        def Init(self):
            pass

        def Write(self, msg):
            CALLS.append(("WRITE", None))

    class MotionSwitcherClient:
        def __init__(self):
            CALLS.append(("MOTION_SWITCHER", None))

    class CRC:
        def Crc(self, msg):
            return 0

    mods = {
        "unitree_sdk2py": types.ModuleType("unitree_sdk2py"),
        "unitree_sdk2py.core": types.ModuleType("unitree_sdk2py.core"),
        "unitree_sdk2py.core.channel": types.SimpleNamespace(ChannelSubscriber=ChannelSubscriber,
                                                             ChannelPublisher=ChannelPublisher,
                                                             ChannelFactoryInitialize=lambda *a, **k: None),
        "unitree_sdk2py.idl": types.ModuleType("unitree_sdk2py.idl"),
        "unitree_sdk2py.idl.default": types.SimpleNamespace(unitree_hg_msg_dds__LowCmd_=lambda: None),
        "unitree_sdk2py.idl.unitree_hg": types.ModuleType("unitree_sdk2py.idl.unitree_hg"),
        "unitree_sdk2py.idl.unitree_hg.msg": types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg"),
        "unitree_sdk2py.idl.unitree_hg.msg.dds_": types.SimpleNamespace(LowCmd_=object, LowState_=object),
        "unitree_sdk2py.utils": types.ModuleType("unitree_sdk2py.utils"),
        "unitree_sdk2py.utils.crc": types.SimpleNamespace(CRC=CRC),
        "unitree_sdk2py.comm": types.ModuleType("unitree_sdk2py.comm"),
        "unitree_sdk2py.comm.motion_switcher": types.ModuleType("unitree_sdk2py.comm.motion_switcher"),
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client":
            types.SimpleNamespace(MotionSwitcherClient=MotionSwitcherClient),
    }
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.setattr(ui, "ensure_channel_factory", lambda *a, **k: None)


def test_observe_only_never_publishes(monkeypatch):
    CALLS.clear()
    _install_fake_sdk(monkeypatch)
    jm = JointMap.load()
    itf = ui.UnitreeSDK2Interface(jm, network_interface="lo", observe_only=True, state_timeout_s=2.0)
    itf.connect()
    st = itf.get_low_state()
    assert st.q.shape == (26,) and np.allclose(st.q, 0.1)
    # commands are computed but must not be sent
    itf.send_low_cmd(jm.defaults.copy(), jm.kp.copy(), jm.kd.copy())
    itf.close()
    kinds = [c[0] for c in CALLS]
    assert ("subscriber", jm.lowstate_topic) in CALLS
    assert "PUBLISHER" not in kinds, CALLS
    assert "WRITE" not in kinds, CALLS
    assert "MOTION_SWITCHER" not in kinds, CALLS
    assert itf.last_shadow_cmd is not None
