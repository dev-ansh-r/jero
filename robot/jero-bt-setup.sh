#!/bin/sh
# Jero: Bluetooth powered + SSP off + connectable, so the PS4 pad reconnects by itself (PS button).
#  - connectable: otherwise the Pi ignores the pad calling it after a reboot
#  - SSP off: with Secure Simple Pairing the kernel refuses the pad's HID channel ("security
#    block"): the pad asks for it ~10 ms before encryption is up and never retries. The existing
#    pairing key still authenticates and encrypts the link. To pair a NEW device: btmgmt ssp on.
# btmgmt hangs without a terminal, so each call gets a pseudo-terminal (script) and a time limit.
# bluetoothd re-enables SSP when it initialises the adapter: retry until the settings stick.
# Installed by robot/setup_pi.sh as /usr/local/bin/jero-bt-setup (jero-bt-connectable.service).

bt() {
  timeout 8 script -qec "btmgmt $*" /dev/null </dev/null 2>/dev/null | tr -d '\r'
}

i=0
while [ "$i" -lt 30 ]; do
  s=$(bt info | grep "current settings")
  case "$s" in
    *powered*connectable*)
      case "$s" in
        *" ssp "*) ;;
        *) echo "ok: $s"; exit 0 ;;
      esac
      ;;
  esac
  bt power on >/dev/null
  bt ssp off >/dev/null
  bt connectable on >/dev/null
  sleep 2
  i=$((i + 1))
done
echo "failed: $(bt info | grep 'current settings')" >&2
exit 1
