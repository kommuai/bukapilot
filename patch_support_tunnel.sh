#!/bin/bash

FILE="/usr/kommu/support_tunnel.py"

# Skip if not found
[ -f "$FILE" ] || exit 0

# Make sure file is not immutable (ignore errors)
sudo chattr -i "$FILE" 2>/dev/null || true

# Make sure file is writeable
sudo chmod u+w "$FILE" 2>/dev/null || true

# Replace the SERVER line
sed -i 's|SERVER = "web.kommu.ai"|SERVER = "x.kommu.ai"|' "$FILE" 2>/dev/null || true

echo "Done."

