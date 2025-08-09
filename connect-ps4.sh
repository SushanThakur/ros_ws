#!/bin/bash

CONTROLLER_MAC="61:B6:AD:21:16:B6"

# Unpair if already paired
if bluetoothctl paired-devices | grep -iq "$CONTROLLER_MAC"; then
    echo "Device is paired. Removing..."
    bluetoothctl remove "$CONTROLLER_MAC"
else
    echo "Device is not paired. Skipping removal..."
fi

# Restart Bluetooth service
sudo systemctl restart bluetooth

echo "Put your DS4 into pairing mode (hold Share + PS until light blinks)..."
bluetoothctl agent on
bluetoothctl default-agent
sleep 5

# Scan until found
bluetoothctl scan on &
SCAN_PID=$!
FOUND=0
for i in {1..20}; do
    if bluetoothctl devices | grep -iq "$CONTROLLER_MAC"; then
        FOUND=1
        break
    fi
    sleep 1
done
kill $SCAN_PID

if [ "$FOUND" -eq 0 ]; then
    echo "Controller not found. Exiting."
    exit 1
fi

# Pair, trust, connect
bluetoothctl pair "$CONTROLLER_MAC"
bluetoothctl trust "$CONTROLLER_MAC"
bluetoothctl connect "$CONTROLLER_MAC"

# Load DS4 driver
# sudo modprobe hid_sony
sudo modprobe hid_playstation
sudo udevadm settle
sudo modprobe joydev
sudo udevadm trigger --subsystem-match=input

