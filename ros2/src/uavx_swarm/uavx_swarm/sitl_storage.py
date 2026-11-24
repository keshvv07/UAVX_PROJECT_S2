"""Prepare per-aircraft SITL parameter storage before starting the shared DDS agent."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from pymavlink import mavutil


def _namespace_boot(executable, directory, uav, *, configure):
    # One loopback socket, allocated by the OS, dedicated to this child firmware.
    link = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=254)
    port = link.port.getsockname()[1]
    sysid = uav["sysid"]
    command = [
        str(executable), "--model", "quad", "--instance", str(uav["instance"]),
        "--sysid", str(sysid), "--serial0", f"udpclient:127.0.0.1:{port}",
        "--defaults", str(directory / "prepare.parm"), "--speedup", "3",
    ]
    try:
        with (directory / "prepare.log").open("ab") as log, subprocess.Popen(
            command, cwd=directory, stdout=log, stderr=subprocess.STDOUT,
        ) as child:
            try:
                deadline = time.monotonic() + 45
                connected = False
                while time.monotonic() < deadline and child.poll() is None:
                    msg = link.recv_match(type=["HEARTBEAT", "PARAM_VALUE"],
                                          blocking=True, timeout=1)
                    if msg is None or msg.get_srcSystem() != sysid or msg.get_srcComponent() != 1:
                        continue
                    if msg.get_type() == "HEARTBEAT":
                        connected = True
                        link.mav.param_request_read_send(sysid, 1, b"DDS_USE_NS", -1)
                    elif connected and msg.param_id == "DDS_USE_NS":
                        if msg.param_value == 1:
                            # AP_Param queues writes. The second boot independently
                            # checks the persisted value before we use this storage.
                            if configure:
                                time.sleep(1)
                            return
                        if not configure:
                            raise RuntimeError(f"{uav['id']}: DDS_USE_NS did not survive reboot")
                        link.mav.param_set_send(sysid, 1, b"DDS_USE_NS", 1,
                                                mavutil.mavlink.MAV_PARAM_TYPE_INT8)
                raise RuntimeError(f"{uav['id']}: no DDS_USE_NS confirmation; see {directory}/prepare.log")
            finally:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
    finally:
        link.close()


def prepare_namespaces(executable, run_dir, vehicles):
    for uav in vehicles:
        instance_dir = Path(run_dir) / str(uav["instance"])
        instance_dir.mkdir(exist_ok=True)
        storage = instance_dir / "eeprom.bin"
        # Start from existing storage when resuming an explicitly selected run.
        # Publish the new image only after a second firmware boot verifies it.
        with tempfile.TemporaryDirectory(dir=instance_dir) as tmp:
            directory = Path(tmp)
            if storage.exists():
                shutil.copy2(storage, directory / storage.name)
            (directory / "prepare.parm").write_text(
                "DDS_ENABLE 1\nDDS_USE_NS 0\nDDS_UDP_PORT 29819\n", encoding="ascii")
            try:
                _namespace_boot(executable, directory, uav, configure=True)
                _namespace_boot(executable, directory, uav, configure=False)
            finally:
                log = directory / "prepare.log"
                if log.exists():
                    shutil.copyfile(log, instance_dir / "prepare.log")
            os.replace(directory / storage.name, storage)
        print(f"UAV-X {uav['id']}: persisted DDS_USE_NS=1 verified after reboot", flush=True)
