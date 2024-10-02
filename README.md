# Overview

WiBotic ROS CAN Connector is a set of ROS packages that supports interacting with a WiBotic Onboard Charger over a CAN bus.

The master branch of this repository remains synchronized with the latest WiBotic firmware release. Running firmware that matches with the equivalent WiBotic ROS CAN Connector version is strongly recommended. See the firmware update page on the WiBotic system GUI for more information about updating firmware.

## Add can interface

```bash
sudo slcand -o -s6 -t hw -S 3000000 /dev/ttyACM0
sudo ip link set up can0 type can bitrate 500000
```
