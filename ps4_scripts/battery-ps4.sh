#!/bin/bash

BATTERY_PATH="/sys/class/power_supply/ps-controller-battery-61:b6:ad:21:16:b6/capacity"

echo "PS4 Controller Battery: $(cat "$BATTERY_PATH")%"
