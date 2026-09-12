#!/usr/bin/env python3
"""Exercise the compiled bridge's exit lifecycle in a loopback-only network namespace.

Refuses to run if any non-loopback interface exists. No control is enabled.
Source ROS and the workspace first; --log-dir retains process evidence.
"""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client


def stop_process(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", required=True, type=Path)
    args = parser.parse_args()
    if {name for _, name in socket.if_nameindex()} != {"lo"}:
        raise SystemExit("REFUSED: run inside a network namespace with only lo")
    root = Path(__file__).resolve().parents[1]
    binary = root / "devel/lib/go2_control/go2_sdk_bridge_real_node"
    if not binary.is_file():
        raise SystemExit("Build go2_sdk_bridge_real_node first")
    args.log_dir.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:11349",
               ROS_HOSTNAME="127.0.0.1", ROS_IP="127.0.0.1",
               ROS_LOG_DIR=str(args.log_dir / "ros"))
    socket.setdefaulttimeout(2)
    master = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
    with (args.log_dir / "master.log").open("w") as master_log:
        core = subprocess.Popen(["roscore", "-p", "11349"], env=env,
                                stdout=master_log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            for _ in range(100):
                try:
                    if master.getPid("/shutdown_lifecycle_check")[0] == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                time.sleep(0.1)
            else:
                raise RuntimeError("isolated ROS master did not start")
            for label, sig in (("sigint", signal.SIGINT),
                               ("sigterm", signal.SIGTERM),
                               ("sighup", signal.SIGHUP), ("ros_shutdown", None)):
                logfile = args.log_dir / (label + ".log")
                name = "shutdown_probe_" + label
                with logfile.open("w") as output:
                    proc = subprocess.Popen(
                        ["stdbuf", "-oL", "-eL", str(binary),
                         "_network_interface:=lo", "__name:=" + name], env=env,
                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        for _ in range(200):
                            if proc.poll() is not None:
                                raise RuntimeError(label + ": bridge exited during initialization")
                            if "REAL GO2 SDK bridge started DISABLED" in logfile.read_text():
                                break
                            time.sleep(0.1)
                        else:
                            raise RuntimeError(label + ": initialization timed out")
                        start = time.monotonic()
                        if sig is None:
                            code, _, uri = master.lookupNode("/shutdown_lifecycle_check", "/" + name)
                            assert code == 1
                            reply = xmlrpc.client.ServerProxy(uri).shutdown(
                                "/shutdown_lifecycle_check", "isolated exit test")
                            assert reply[0] == 1
                        else:
                            proc.send_signal(sig)
                        result = proc.wait(timeout=8)
                        body = logfile.read_text()
                        assert result == 0, (label, result)
                        assert "already idle; no new SDK stop request" in body, body
                        assert "STOP UNCONFIRMED" not in body and "StopMove sent" not in body, body
                        print("PASS:", label, "idle exit in %.3fs" % (time.monotonic() - start), flush=True)
                    finally:
                        stop_process(proc)
        finally:
            stop_process(core)


if __name__ == "__main__":
    main()
