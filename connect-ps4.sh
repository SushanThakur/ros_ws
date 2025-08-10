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
    sleep 1
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

# Unbind and rebind kernel driver to reset controller device
DEVICE_PATH=$(grep -l "Wireless Controller" /sys/class/hidraw/*/device/uevent | sed 's|/uevent||')
if [ -n "$DEVICE_PATH" ]; then
    echo "Unbinding and rebinding device driver..."
    echo -n "$DEVICE_PATH" | sudo tee /sys/bus/hid/drivers/sony/unbind
    sleep 1
    echo -n "$DEVICE_PATH" | sudo tee /sys/bus/hid/drivers/sony/bind
else
    echo "Device path not found for driver rebind."
fi

# Reload drivers to ensure proper input setup
sudo modprobe -r joydev hid_playstation
sudo modprobe hid_playstation
sudo modprobe joydev

sudo udevadm settle
sudo udevadm trigger --subsystem-match=input

echo "Controller should be connected and drivers loaded."

