#!/usr/bin/env bash
# Detects this Mac's LAN IP and hostname, then writes them into firmware/network.env.
# Run this whenever you switch networks before flashing firmware.
#
# Usage:
#   bash scripts/discover_host.sh
#   bash scripts/discover_host.sh --dry-run   # print without writing

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENV_FILE="$REPO_ROOT/firmware/network.env"

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
fi

# Detect LAN IP (try common interfaces in order)
IP=""
for iface in en0 en1 en2 en3; do
    IP=$(ipconfig getifaddr "$iface" 2>/dev/null || true)
    if [[ -n "$IP" ]]; then
        echo "Detected IP $IP on $iface"
        break
    fi
done

if [[ -z "$IP" ]]; then
    echo "ERROR: Could not detect a LAN IP on any interface (en0..en3)."
    echo "Make sure you are connected to Wi-Fi or Ethernet."
    exit 1
fi

HOSTNAME=$(scutil --get LocalHostName 2>/dev/null || hostname -s)
MDNS_HOST="${HOSTNAME}.local"

echo "Detected hostname: $MDNS_HOST"

if [[ "$DRY_RUN" == true ]]; then
    echo ""
    echo "Dry run — would write to $ENV_FILE:"
    echo "  SERVER_HOST=$MDNS_HOST"
    echo "  SERVER_FALLBACK_IP=$IP"
    exit 0
fi

if [[ ! -f "$ENV_FILE" ]]; then
    echo "ERROR: $ENV_FILE not found."
    echo "Copy firmware/network.env.example to firmware/network.env first."
    exit 1
fi

# Update SERVER_HOST and SERVER_FALLBACK_IP in place, preserve other lines
TMPFILE=$(mktemp)
while IFS= read -r line || [[ -n "$line" ]]; do
    if [[ "$line" =~ ^SERVER_HOST= ]]; then
        echo "SERVER_HOST=$MDNS_HOST"
    elif [[ "$line" =~ ^SERVER_FALLBACK_IP= ]]; then
        echo "SERVER_FALLBACK_IP=$IP"
    else
        echo "$line"
    fi
done < "$ENV_FILE" > "$TMPFILE"
mv "$TMPFILE" "$ENV_FILE"

echo ""
echo "Updated $ENV_FILE"
echo "  SERVER_HOST=$MDNS_HOST"
echo "  SERVER_FALLBACK_IP=$IP"
echo ""
echo "Re-flash the firmware to apply: cd firmware && pio run -e nanorp2040connect -t upload"
