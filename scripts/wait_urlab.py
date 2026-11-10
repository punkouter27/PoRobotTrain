"""Poll until the URLab editor RPC server answers on tcp://localhost:5559."""
import sys, time
from urlab_client import URLabClient
deadline = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 600)
while time.time() < deadline:
    try:
        c = URLabClient("tcp://localhost", mujoco_version_check=False, recv_timeout_ms=3000)
        c.discover()
        print("connected: urlab", c.urlab_version, "mujoco", c.mujoco_version, "manager_present", c.manager_present, flush=True)
        c.close(); sys.exit(0)
    except Exception as e:
        print("waiting:", type(e).__name__, str(e)[:120], flush=True); time.sleep(10)
sys.exit(1)
