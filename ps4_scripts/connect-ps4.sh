#!/bin/bash

CONTROLLER_MAC="61:B6:AD:21:16:B6"

# Restart Bluetooth service
sudo systemctl restart bluetooth

echo "Put your DS4 into pairing mode (hold Share + PS until light blinks)..."

# Start Bluetooth agent
bluetoothctl agent on
bluetoothctl default-agent

sleep 5

# Start scanning
bluetoothctl scan on &
SCAN_PID=$!

FOUND=0
for i in {1..20}; do
    if bluetoothctl devices | grep -iq "$CONTROLLER_MAC"; then
        FOUND=1
        break
    fi
    sleep 2
done

kill $SCAN_PID

if [ "$FOUND" -eq 0 ]; then
    echo "Controller not found. Exiting."
    exit 1
fi

# Pair, trust, and connect
bluetoothctl pair "$CONTROLLER_MAC"
bluetoothctl trust "$CONTROLLER_MAC"
bluetoothctl connect "$CONTROLLER_MAC"

sleep 2

# Reload drivers to ensure proper input setup
sudo modprobe -r joydev hid_playstation
sudo modprobe joydev
sudo modprobe hid_playstation

sudo udevadm settle
sudo udevadm trigger --subsystem-match=input

BATTERY_PATH="/sys/class/power_supply/ps-controller-battery-61:b6:ad:21:16:b6/capacity"

echo "PS4 Controller Battery: $(cat "$BATTERY_PATH")%"

