#!/usr/bin/env bash

set -euo pipefail

PEER_NAME="ADS1292"
IMAGE_PATH="build/PPG/zephyr/zephyr.signed.bin"
WAIT_SECONDS=30
SKIP_CONFIRM=0

usage() {
    cat <<'EOF'
Usage:
  ble_ota_update.sh [--peer NAME] [--image PATH] [--wait SECONDS] [--skip-confirm]

Options:
  --peer NAME         BLE peer name for mcumgr. Default: ADS1292
  --image PATH        Signed application image to upload.
                      Default: build/PPG/zephyr/zephyr.signed.bin
  --wait SECONDS      Max seconds to wait after reset for the device to come back.
                      Default: 30
  --skip-confirm      Leave the new image unconfirmed after reboot.
  -h, --help          Show this help.

Environment:
  MCUMGR_BIN          Override mcumgr executable path. Default: mcumgr

Example:
  ./scripts/ble_ota_update.sh --peer ADS1292 --image build/PPG/zephyr/zephyr.signed.bin
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --peer)
            PEER_NAME="$2"
            shift 2
            ;;
        --image)
            IMAGE_PATH="$2"
            shift 2
            ;;
        --wait)
            WAIT_SECONDS="$2"
            shift 2
            ;;
        --skip-confirm)
            SKIP_CONFIRM=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

MCUMGR_BIN="${MCUMGR_BIN:-mcumgr}"
CONNSTRING="peer_name=${PEER_NAME}"

if [[ ! -f "${IMAGE_PATH}" ]]; then
    echo "Image not found: ${IMAGE_PATH}" >&2
    exit 1
fi

run_mcumgr() {
    "${MCUMGR_BIN}" --conntype ble --connstring "${CONNSTRING}" "$@"
}

get_image_list() {
    run_mcumgr image list
}

extract_hash_for_slot() {
    local slot="$1"
    awk -v slot="${slot}" '
        $1 == "image=0" && $2 == "slot=" slot { in_slot=1; next }
        in_slot && $1 == "hash:" { print $2; exit }
        $1 == "image=0" && $2 != "slot=" slot { in_slot=0 }
    '
}

extract_active_hash() {
    awk '
        $1 == "image=0" && $2 ~ /^slot=/ { in_slot=1; slot=$2; flags=""; hash="" ; next }
        in_slot && $1 == "flags:" {
            flags=substr($0, index($0, ":") + 1)
        }
        in_slot && $1 == "hash:" {
            hash=$2
            if (flags ~ /active/) {
                print hash
                exit
            }
        }
    '
}

echo "Using mcumgr: ${MCUMGR_BIN}"
echo "Peer name: ${PEER_NAME}"
echo "Image: ${IMAGE_PATH}"

echo
echo "Current images:"
before_list="$(get_image_list)"
printf '%s\n' "${before_list}"

echo
echo "Uploading new image..."
run_mcumgr image upload "${IMAGE_PATH}"

echo
echo "Images after upload:"
after_upload_list="$(get_image_list)"
printf '%s\n' "${after_upload_list}"

slot1_hash="$(printf '%s\n' "${after_upload_list}" | extract_hash_for_slot 1)"
if [[ -z "${slot1_hash}" ]]; then
    echo "Failed to determine slot 1 hash after upload." >&2
    exit 1
fi

echo
echo "Testing uploaded image in slot 1: ${slot1_hash}"
run_mcumgr image test "${slot1_hash}"

echo
echo "Resetting target..."
run_mcumgr reset

echo
echo "Waiting for device to come back..."
active_hash=""
for ((i = 1; i <= WAIT_SECONDS; i++)); do
    if list_output="$(get_image_list 2>/dev/null)"; then
        active_hash="$(printf '%s\n' "${list_output}" | extract_active_hash)"
        if [[ -n "${active_hash}" ]]; then
            echo "Device is back after ${i}s."
            break
        fi
    fi
    sleep 1
done

if [[ -z "${active_hash}" ]]; then
    echo "Timed out waiting for device to reconnect after reset." >&2
    exit 1
fi

echo
echo "Images after reboot:"
printf '%s\n' "${list_output}"

if [[ "${active_hash}" != "${slot1_hash}" ]]; then
    echo "Warning: active hash (${active_hash}) does not match uploaded slot 1 hash (${slot1_hash})." >&2
    echo "The device may have reverted or booted a different image." >&2
    exit 1
fi

if [[ "${SKIP_CONFIRM}" -eq 0 ]]; then
    echo
    echo "Confirming active image..."
    run_mcumgr image confirm

    echo
    echo "Final image state:"
    get_image_list
else
    echo
    echo "Skipping confirmation as requested."
fi

