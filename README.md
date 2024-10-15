# wibotic_ros

The repository contains `wibotic_connector_can` and `wibotic_msgs` packages. It reads a CAN Bus thanks to the uavcan library and sends the measurements to ROS 2.

## ROS Nodes

### wibotic_connector_can

It reads a CAN Bus thanks to the uavcan library and sends the measurements to ROS 2.

#### Publishes

- `wibotic_info` [*wibotic_msgs/WiboticInfo*]: Wibotic charger measurements.

#### Parameters

- `~can_iface_name` [*string*, default: **can0**]: CAN BUS interface used for Wibotic receiver.
- `uavcan_node_id_` [*int*, default: **20**]: Uavcan node ID.
- `uavcan_node_name_` [*string*, default: **can0**]: Uavcan node name.
- `update_time_s_` [*string*, default: **can0**]: The period of reading WiboticInfo on a CAN BUS.

## Add CAN interface

```bash
sudo slcand -o -s6 -t hw -S 3000000 /dev/ttyACM0
sudo ip link set up can0 type can bitrate 500000
```
