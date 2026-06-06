
This file contains information on how to test the state synchronisation.

## Starting

Run the following commands in separate terminals to start the two nodes
and monitor the log outputs.
```bash
ros2 run tb3_state_dt rob_water_level
ros2 run tb3_state_dt sim_water_level
```

## Manipulating

To request a fill-up of the tank, use the command below with the
target percentage.
```bash
ros2 topic pub /cmd_water_fill std_msgs/Float64 'data: .8' --once
```

Request watering of the tank with the command below. The parameter
indicates the percentage of the tank to release.
```bash
ros2 topic pub /cmd_water_use std_msgs/Float64 'data: .2' --once
```

A water usage can also be validated before the actual request.
This will return `true` if the requested amount of water is currently
available in the simulated tank.
```bash
ros2 service call /validate_water_usage custom_interfaces/srv/ValidateWaterUsage 'amount: .8'
```