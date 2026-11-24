import subprocess

CON_NAME = "GoGuideAP"
IFNAME = "wlp1s0"  # replace with dongle interface name on robot


def _run(cmd, timeout=15):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode == 0, r.stdout.strip(), r.stderr.strip()


def _active_con_on_ifname(ifname):
    ok, out, _ = _run(["nmcli", "-t", "-f", "NAME,DEVICE", "connection", "show", "--active"])
    if not ok:
        return None
    for line in out.splitlines():
        if ":" in line:
            name, dev = line.rsplit(":", 1)
            if dev == ifname:
                return name
    return None


def create_hotspot_once(ssid="GoGuide", password="yourpassword123"):
    """Recreates the saved AP profile bound to IFNAME."""
    _run(["nmcli", "connection", "delete", CON_NAME])
    ok, _, err = _run(
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
            ssid,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.band",
            "bg",
            "802-11-wireless.channel",
            "6",
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            password,
            "ipv4.method",
            "shared",
            "ipv6.method",
            "ignore",
        ]
    )
    if not ok:
        raise RuntimeError(f"Hotspot profile creation failed: {err}")


def hotspot_start():
    # Ensure profile exists and matches IFNAME
    ok, out, _ = _run(["nmcli", "-t", "-f", "connection.interface-name", "connection", "show", CON_NAME])
    if not ok or out.split(":", 1)[-1].strip() != IFNAME:
        create_hotspot_once()

    active = _active_con_on_ifname(IFNAME)
    if active and active != CON_NAME:
        _run(["nmcli", "connection", "down", active])

    ok, out, err = _run(["nmcli", "connection", "up", CON_NAME])
    if not ok:
        raise RuntimeError(f"Hotspot start failed: {err}")
    return out


def hotspot_stop():
    ok, out, err = _run(["nmcli", "connection", "down", CON_NAME])
    if not ok:
        raise RuntimeError(f"Hotspot stop failed: {err}")
    return out


def hotspot_active():
    ok, out, _ = _run(["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"])
    return ok and CON_NAME in out.splitlines()
