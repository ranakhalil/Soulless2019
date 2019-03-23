rostopic pub /enable_disable roscco/EnableDisable -1 '{header: {stamp: now}, enable_control: true}'
rostopic pub /speed/setpoint std_msgs/Float64 -1 '{data: 20}'

