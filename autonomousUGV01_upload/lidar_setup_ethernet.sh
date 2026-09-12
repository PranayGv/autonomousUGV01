#!/usr/bin/env bash
#
# lidar_setup_ethernet.sh
# Network setup for the Unitree L2 LiDAR in Ethernet/UDP mode.
# The L2 ships at IP 192.168.1.62; the host must sit on the same /24 subnet.
#
# Uses a dedicated NetworkManager profile so the static IP PERSISTS and is not
# flushed by NetworkManager on link events (a plain `ip addr add` does not
# survive, which breaks the driver's UDP bind). Requires sudo.
#
set -euo pipefail

IFACE="${1:-enp57s0}"
HOST_IP="192.168.1.2/24"       # matches local_ip in the driver
LIDAR_IP="192.168.1.62"        # L2 default
CON="unitree-l2"

echo "==> Interface: $IFACE"
[ -e "/sys/class/net/$IFACE" ] || { echo "ERR: no interface '$IFACE'. Available:"; ls /sys/class/net/; exit 1; }

if command -v nmcli >/dev/null && systemctl is-active --quiet NetworkManager 2>/dev/null; then
  echo "==> Configuring persistent NetworkManager profile '$CON'"
  if nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$CON"; then
    sudo nmcli connection modify "$CON" ipv4.method manual ipv4.addresses "$HOST_IP" ipv6.method ignore
  else
    sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$CON" \
      ipv4.method manual ipv4.addresses "$HOST_IP" ipv6.method ignore autoconnect no
  fi
  sudo nmcli connection up "$CON"
else
  echo "==> nmcli/NetworkManager not active; falling back to 'ip addr' (non-persistent)"
  sudo ip link set "$IFACE" up
  sudo ip addr del "$HOST_IP" dev "$IFACE" 2>/dev/null || true
  sudo ip addr add "$HOST_IP" dev "$IFACE"
fi

echo "==> Address on $IFACE:"; ip -brief addr show "$IFACE"

echo "==> Pinging LiDAR at $LIDAR_IP ..."
if ping -c 3 -W 2 "$LIDAR_IP"; then
  echo "OK: LiDAR reachable. Now run:  ./lidar_run.sh"
else
  echo "WARN: no reply from $LIDAR_IP. Check the L2 is powered and the cable is in $IFACE,"
  echo "      and that the L2 is in Ethernet mode. Serial fallback:  ./lidar_run.sh serial"
fi
