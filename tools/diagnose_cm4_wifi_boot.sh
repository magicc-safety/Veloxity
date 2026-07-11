#!/usr/bin/env bash
# Report offline CM4 Wi-Fi boot configuration and previous-boot service logs.
set -euo pipefail

ROOTFS="${1:-/mnt/cm4-root}"

if [[ ! -d "$ROOTFS/etc/systemd/system" ]]; then
  echo "Not a mounted CM4 root filesystem: $ROOTFS" >&2
  exit 1
fi

echo '== Saved Wi-Fi mode =='
cat "$ROOTFS/var/lib/veloxity-wifi/mode" 2>&1 || true

echo
echo '== Restore service =='
sed -n '1,160p' "$ROOTFS/etc/systemd/system/veloxity-wifi-mode.service" 2>&1 || true

echo
echo '== Service enabled link =='
ls -l "$ROOTFS/etc/systemd/system/multi-user.target.wants/veloxity-wifi-mode.service" 2>&1 || true

echo
echo '== Restore script =='
sed -n '1,220p' "$ROOTFS/usr/local/sbin/veloxity-wifi" 2>&1 || true

echo
echo '== Netplan Wi-Fi configuration =='
sed -n '1,220p' "$ROOTFS/etc/netplan"/*.yaml 2>&1 || true

echo
echo '== Previous boot: restore service =='
journalctl --directory="$ROOTFS/var/log/journal" \
  -u veloxity-wifi-mode.service -b -0 --no-pager 2>&1 || true

echo
echo '== Previous boot: NetworkManager Wi-Fi events =='
journalctl --directory="$ROOTFS/var/log/journal" \
  -u NetworkManager.service -b -0 --no-pager 2>&1 | \
  grep -Ei 'wlan0|wifi|drone|MAGICC|unmanaged|error|fail' || true
