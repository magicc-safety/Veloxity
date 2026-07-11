#!/usr/bin/env bash
# Repair the CM4 Wi-Fi mode restore service on an offline mounted rootfs.
set -euo pipefail

ROOTFS="${1:-/mnt/cm4-root}"

if [[ ! -d "$ROOTFS/etc/systemd/system" ]]; then
  echo "Not a mounted CM4 root filesystem: $ROOTFS" >&2
  exit 1
fi

install -d -m 0755 "$ROOTFS/var/lib/veloxity-wifi"

if [[ -e "$ROOTFS/usr/local/sbin/veloxity-wifi" ]]; then
  cp -a "$ROOTFS/usr/local/sbin/veloxity-wifi" \
    "$ROOTFS/usr/local/sbin/veloxity-wifi.pre-retry.bak"
fi

tee "$ROOTFS/usr/local/sbin/veloxity-wifi" >/dev/null <<'EOF'
#!/usr/bin/env bash
set -u -o pipefail

STATE_DIR=/var/lib/veloxity-wifi
STATE_FILE="$STATE_DIR/mode"
AP_PROFILE=drone-ap
CLIENT_PROFILE=netplan-wlan0-MAGICC

activate_profile() {
  local profile="$1"

  # A CM4 association can take around 6 seconds. Do not issue a second
  # activation until the first has had enough time to complete; a short
  # timeout would otherwise disconnect a just-established link.
  for _ in $(seq 1 4); do
    if /usr/bin/nmcli -w 30 connection up "$profile" ifname wlan0; then
      return 0
    fi
    /usr/bin/sleep 2
  done

  echo "Timed out activating Wi-Fi profile: $profile" >&2
  return 1
}

mkdir -p "$STATE_DIR"

case "${1:-restore}" in
  ap)
    printf '%s\n' ap > "$STATE_FILE"
    activate_profile "$AP_PROFILE"
    ;;
  client)
    printf '%s\n' client > "$STATE_FILE"
    activate_profile "$CLIENT_PROFILE"
    ;;
  restore)
    mode=client
    [[ -r "$STATE_FILE" ]] && read -r mode < "$STATE_FILE"

    case "$mode" in
      ap) activate_profile "$AP_PROFILE" ;;
      client) activate_profile "$CLIENT_PROFILE" ;;
      *) echo "Invalid saved Wi-Fi mode: $mode" >&2; exit 1 ;;
    esac
    ;;
  *)
    echo "Usage: veloxity-wifi {ap|client|restore}" >&2
    exit 2
    ;;
esac
EOF

tee "$ROOTFS/etc/systemd/system/veloxity-wifi-mode.service" >/dev/null <<'EOF'
[Unit]
Description=Restore selected Veloxity Wi-Fi mode
Wants=NetworkManager.service
After=NetworkManager.service

[Service]
Type=oneshot
TimeoutStartSec=150
ExecStart=/usr/local/sbin/veloxity-wifi restore

[Install]
WantedBy=multi-user.target
EOF

chmod 0755 "$ROOTFS/usr/local/sbin/veloxity-wifi"

# Recover through the known-good infrastructure Wi-Fi first. The normal
# `veloxity-wifi ap` / `client` commands will persist the next selected mode.
printf '%s\n' client > "$ROOTFS/var/lib/veloxity-wifi/mode"

sync
echo "Repaired Wi-Fi restore service; next boot will use client mode (MAGICC)."
