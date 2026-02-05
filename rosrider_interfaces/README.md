# 🕸️ ROSRider Interfaces Package

This package defines the custom messages used for internode communication across the robot's control and diagnostics systems.  

### Messages

`rosrider_interfaces/msg/Diagnostics.msg`

This message provides a comprehensive snapshot of the robot's low-level operational status, including motor control values and system health metrics. It is typically published periodically by the low-level controller node.

| Field         | Type    | Description                                                     |
|:--------------|:--------|-----------------------------------------------------------------|
| pwm_left      | int16   | 	Current PWM value being applied to the left motor.             |
| pwm_right     | int16   | 	Current PWM value being applied to the right motor.            |  
| omega_left    | float32 | 	Measured Radians per Second (RPS) of the left wheel.           |  
| omega_right   | float32 | 	Measured RPS of the right wheel.                               | 
| target_left   | float32 | 	Desired speed (RPM) for the left wheel (controller setpoint).  |
| target_right  | float32 | 	Desired speed (RPM) for the right wheel (controller setpoint). | 
| cs_left       | float32 | 	Current sense metric for the left wheel.                       |
| cs_right      | float32 | 	Current sense  metric for the right wheel.                     | 
| bus_voltage   | float32 | 	Measured voltage of the motor power bus (Volts).               |
| bus_current   | float32 | 	Measured current draw of the motor power bus (Amperes).        | 
| power_status  | uint8   | 	Status flags for the power system (e.g. low battery).          | 
| motor_status  | uint8   | 	Status flags for the motors (e.g., error, overheating).        | 
| system_status | uint8   | 	Overall system status or operational mode flags.               | 
| phase_error   | int16   | 	Data sampling phase error.                                     |

**POWER STATUS**

The `power_status` field serves as an **8-bit register** that summarizes all critical software and electrical fault states monitored on the board.
This register reports various protection triggers, including the **main current fuse** `(b3)` and motor specific **software fuse trips** `(b5, b4)`,
along with common power supply issues such as bus **over-voltage** `(b2)` or **under-voltage** `(b1)`. Additionally,
it tracks non-physical faults like a **command timeout** `(b7)` and the status of the auxiliary power output `(b0)`

| Bit | Name          | Description                            |
|-----|---------------|----------------------------------------|
| b7	 | CMD_TIMEOUT   | The last applied command has timed out |
| b6	 | POWER_BAD     | Power problem                          |
| b5	 | RIGHT_AMP     | Right motor Software Fuse Triggered    |
| b4	 | LEFT_AMP      | Left motor Software Fuse Triggered     |
| b3	 | MAIN_FUSE     | Main Current Fuse Triggered            |
| b2	 | OVER_VOLTAGE  | Bus voltage exceeds the limit          |
| b1	 | UNDER_VOLTAGE | Bus voltage is below the limit         |
| b0	 | AUX_PWR       | Aux power output status                |

**MOTOR STATUS**

The `motor_status` field is an **8-bit register** that provides a comprehensive summary of the motor system's operational state,
detailing the **fault status** for both the left and right motors `(b7, b6)`, their current direction of rotation `(b5, b4)`,
the activation of two different driver modes `(b3, b2)`, and the current drive mode `(b1, b0)`

| Bit | Name                     | Description           |
|-----|--------------------------|-----------------------|
| b7  | FAULT_RIGHT              | Right motor fault     |
| b6  | FAULT_LEFT               | Left motor fault      |
| b5  | RIGHT_REV (0=REV, 1=FWD) | Right motor direction |         
| b4  | LEFT_REV (0=REV, 1=FWD)  | Left motor direction  |
| b3  | MODE_2                   | Mode 2 Enabled        |
| b2  | MODE_1                   | Mode 1 Enabled        |
| b1  | DRIVE_MODE_MSB           | Drive mode MSB        |
| b0  | DRIVE_MODE_LSB           | Drive mode LSB        |

**Please Note:** The driver's fault flags will always be on if the motor bus voltage is below 12V.

**SYSTEM STATUS**

The `system_status` field is an **8-bit status register** used to report the state of the embedded system,
with key bits indicating critical conditions such as a successful **EEPROM initialization** `b7=0`,
an EEPROM write failure `b0=1`, or the necessity of a system restart `b6=1`

| Bit | Name                       | Description                                   |
|-----|----------------------------|-----------------------------------------------|
| b7  | 	EPROM_INIT_OK = 0         | If 1, EEPROM has failed during initialization |
| b6  | 	RESTART_REQUIRED = 1      | System Restart required                       |
| b1  | INITIAL_UPDATE_ERROR = 1   | Inital update error. Reset to factory values  |
| b0  | 	EEPROM_WRITE_WRITE_OK = 0 | If 1, EEPROM write operation failed           |

### Documentation

For complete and comprehensive guides on all aspects of the ROSRider project, please refer to the dedicated documentation site: [https://docs.acada.dev/rosrider_doc](https://docs.acada.dev/rosrider_doc)

---
#### ACADA Robotics ● [https://acada.dev](https://acada.dev)  
[![ACADA Robotics](https://docs.acada.dev/rosrider_doc/images/logo.svg)](https://acada.dev)