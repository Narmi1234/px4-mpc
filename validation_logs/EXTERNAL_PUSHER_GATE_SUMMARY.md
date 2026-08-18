# External pusher interface gate

Date: 2026-08-18

Source ULog (kept in the PX4 SITL log directory, not duplicated here):

```text
/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-18/06_10_06.ulg
```

NMPC final status:

```text
abort_reason=external_pusher_test_timeout
last_offboard_duration=12.00s
solver_failures=0
max commanded_pusher=0.050
```

ULog measurements:

```text
PX4/Gazebo Offboard duration=10.436s
peak pusher actuator=0.0500
final pusher actuator=0.0000
peak horizontal speed=0.069m/s
altitude span=0.154m
VTOL state=MC only
```

Result: PASS. The full ROS gate completed, PX4 actuator motor 5 received the
bounded command and returned to zero, and all flight-envelope checks passed.
The shorter ULog duration is caused by Gazebo running below real time: NMPC
schedules this first gate in ROS wall time, while ULog uses simulation time.
