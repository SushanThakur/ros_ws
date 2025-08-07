#!/bin/bash

CONTROLLER_MAC="61:B6:AD:21:16:B6"

# Check if the device is paired
if bluetoothctl paired-devices | grep -iq "$CONTROLLER_MAC"; then
    echo "Device is paired. Removing..."
    bluetoothctl remove "$CONTROLLER_MAC"
else
    echo "Device is not paired. Skipping removal..."
fi

# Restart Bluetooth
bluetoothctl power off
sleep 0.5
bluetoothctl power on
sleep 5

# Pair, trust, and connect
bluetoothctl pair "$CONTROLLER_MAC"
bluetoothctl trust "$CONTROLLER_MAC"
bluetoothctl connect "$CONTROLLER_MAC"

