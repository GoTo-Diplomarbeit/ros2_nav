import subprocess

CON_NAME = "GoGuideAP"
IFNAME = "wlan0"  # dongle interface name on the robot


def _run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    return r.returncode == 0, r.stderr.strip()


def _create_profile():
    _run(["nmcli", "connection", "delete", CON_NAME])
    ok, err = _run(
        [
            "nmcli",
            "connection",
            "add",
            "type",
            "wifi",
            "ifname",
            IFNAME,
            "con-name",
            CON_NAME,
            "autoconnect",
            "no",
            "ssid",
            "GoGuide",
            "802-11-wireless.mode",
            "ap",
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            "yourpassword123",
            "ipv4.method",
            "shared",
            "ipv6.method",
            "ignore",
        ]
    )
    if not ok:
        raise RuntimeError(f"Hotspot profile creation failed: {err}")


def hotspot_start():
    ok, _ = _run(["nmcli", "connection", "up", CON_NAME])
    if not ok:  # profile missing or broken: recreate and retry
        _create_profile()
        ok, err = _run(["nmcli", "connection", "up", CON_NAME])
        if not ok:
            raise RuntimeError(f"Hotspot start failed: {err}")


def hotspot_stop():
    ok, err = _run(["nmcli", "connection", "down", CON_NAME])
    if not ok:
        raise RuntimeError(f"Hotspot stop failed: {err}")


def hotspot_active():
    _, _ = None, None
    r = subprocess.run(
        ["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    return CON_NAME in r.stdout.splitlines()
