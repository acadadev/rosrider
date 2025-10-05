# 🖧 ROSRider Interfaces Package

This package defines the custom messages `msg` and services `srv` used for internode communication across the robot's control and diagnostics systems.  

### 📨 Messages

`rosrider_interfaces/msg/Diagnostics.msg`

This message provides a comprehensive snapshot of the robot's low-level operational status, including motor control values and system health metrics. It is typically published periodically by the low-level controller node.

| Field         | Type    | Description                                                        |
|:--------------|:--------|--------------------------------------------------------------------|
| pwm_left      | int16   | 	Current PWM value being applied to the left motor.                |
| pwm_right     | int16   | 	Current PWM value being applied to the right motor.               |  
| rpm_left      | float32 | 	Measured Revolutions Per Minute (RPM) of the left wheel.          |  
| rpm_right     | float32 | 	Measured RPM of the right wheel.                                  | 
| target_left   | float32 | 	Desired speed (RPM) for the left wheel (controller setpoint).     |
| target_right  | float32 | 	Desired speed (RPM) for the right wheel (controller setpoint).    | 
| cs_left       | float32 | 	Current sense metric for the left wheel.                          |
| cs_right      | float32 | 	Current sense  metric for the right wheel.                        | 
| bus_voltage   | float32 | 	Measured voltage of the motor power bus (Volts).                  |
| bus_current   | float32 | 	Measured current draw of the motor power bus (Amperes).           | 
| power_status  | uint8   | 	Status flags for the power system (e.g. low battery).             | 
| motor_status  | uint8   | 	Status flags for the motors (e.g., error, overheating).           | 
| system_status | uint8   | 	Overall system status or operational mode flags.                  | 
| packet_age    | uint16  | 	Time elapsed since this data was sampled on the micro-controller. |

#### POWER STATUS

The `power_status` field serves as an **8-bit register** that summarizes all critical software and electrical fault states monitored on the board.
This register reports various protection triggers, including the **main current fuse** `(b3)` and motor specific **software fuse trips** `(b5, b4)`,
along with common power supply issues such as bus **over-voltage** `(b2)` or **under-voltage** `(b1)`. Additionally,
it tracks non-physical faults like a **command timeout** `(b7)` and the status of the auxiliary power output `(b0)`.

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

#### MOTOR STATUS

The `motor_status` field is an **8-bit register** that provides a comprehensive summary of the motor system's operational state,
detailing the **fault status** for both the left and right motors `(b7, b6)`, their current direction of rotation `(b5, b4)`,
the activation of two different driver modes `(b3, b2)`, and the current drive mode `(b1, b0)`.

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

| Brake Mode | Bit Values |
|------------|------------|
| MODE_BRAKE | 00         |  
| MODE_PWM   | 01         | 
| MODE_VEL   | 10         | 
| MODE_PID   | 11         | 

**Please Note:** The driver's fault flags will always be on if the motor bus voltage is below 12V.

#### SYSTEM STATUS

The `system_status` field is an **8-bit status register** used to report the state of the embedded system,
with key bits indicating critical conditions such as a successful **EEPROM initialization** `b7=0`,
an EEPROM write failure `b0=1`, or the necessity of a system restart `b6=1`.

| Bit | Name                       | Description                                   |
|-----|----------------------------|-----------------------------------------------|
| b7  | 	EPROM_INIT_OK = 0         | If 1, EEPROM has failed during initialization |        
| b6  | 	RESTART_REQUIRED = 1      | System Restart required                       |
| b0  | 	EEPROM_WRITE_WRITE_OK = 0 | If 1, EEPROM write operation failed           |

### 🔨 Services

`rosrider_interfaces/srv/PidCtl.srv`

This service is used to **dynamically adjust the Proportional-Integral-Derivative (PID) controller gains**
for one of the robot's motor control loops at runtime.

| Field | Type    | Description                                                                |
|:------|:--------|----------------------------------------------------------------------------|
| kp    | float32 | The new **Proportional (P)** gain value to set.                            |
| ki    | float32 | The new **Integral (I)** gain value to set.                                |
| kd    | float32 | The new **Derivative (D)** gain value to set.                              |
| i     | uint8   | ID of the PID loop to be configured (0 for Left Wheel, 1 for Right Wheel). |
| result| uint8   | Response code                                                              |

`rosrider_interfaces/srv/SysCtl.srv`

| Field | Type    | Description                                               |
|:------|:--------|-----------------------------------------------------------|
| cmd	| uint8	  | The system command ID specifying the desired system action.|
| result| uint8   | Response code                                             |

This service is used to send high-level, single-byte system commands to control the overall state or
mode of the robot's embedded system.

| SYS COMMAND                  | Code | Description                        |
|:-----------------------------|------|------------------------------------|
| SYSCTL_CODE_RESET            | 0x01 | System Hard Reset                  |
| SYSCTL_CODE_SOFTRESET        | 0x02 | System Soft Reset                  |
| SYSCTL_CODE_ENCODER_RESET    | 0x04 | System Encoder Reset               |
| SYSCTL_CODE_HIBERNATE        | 0x05 | System Hibernate Sleep             |
| SYSCTL_CODE_PRINT_PARAMETERS | 0x06 | Print parameters on serial out     |
| SYSCTL_CODE_PRINT_STATUS     | 0x07 | Print status on serial out         |
| SYSCTL_CODE_RECORD_PID       | 0x08 | Record PID values in flash         |
| SYSCTL_CODE_PRINT_KPID       | 0x09 | Print PID values on serial out     |
| SYSCTL_CODE_DIR_LEFT_FWD     | 0x30 | Left motor forward override        |
| SYSCTL_CODE_DIR_LEFT_REV     | 0x31 | Left motor reverse override        |
| SYSCTL_CODE_DIR_RIGHT_FWD    | 0x32 | Right motor forward override       | 
| SYSCTL_CODE_DIR_RIGHT_REV    | 0x33 | Right motor reverse override       |
| SYSCTL_CODE_DIR_BOTH_FWD     | 0x34 | Motor directions forward override  |
| SYSCTL_CODE_DIR_BOTH_REV     | 0x35 | Motor directions reverse override  |
| SYSCTL_CODE_MODE1_ON         | 0x40 | Drivers MODE1 enabled              |
| SYSCTL_CODE_MODE1_OFF        | 0x41 | Drivers MODE1 off                  |
| SYSCTL_CODE_MODE2_ON         | 0x42 | Drivers MODE2 enabled (servo mode) |
| SYSCTL_CODE_MODE2_OFF        | 0x43 | Drivers MODE2 off                  |
| SYSCTL_CODE_AUX_ON           | 0x50 | 5V Aux power supply on             |
| SYSCTL_CODE_AUX_OFF          | 0x51 | 5V Aux power supply off            |
| SYSCTL_CODE_FACTORY_DEFAULTS | 0x99 | Factory Defaults. Requires reset   |
| SYSCTL_CODE_PRINT_RTC        | 0xAA | Print RTC time on serial out       |

These system control services provide the capability to **override critical settings** at the hardware level.
While they can be used to issue commands to **recover the robot** from a difficult or unresponsive state, **users must exercise extreme caution**.
Certain commands will result in the immediate **loss of connection** with the driver,
potentially requiring a **system restart and/or direct physical intervention (e.g., pressing a reset button)** on the robot.

### 📖 Documentation

For complete and comprehensive guides on all aspects of the ROSRider project, please refer to the dedicated documentation site: [https://docs.acada.dev/rosrider_doc](https://docs.acada.dev/rosrider_doc)

---
#### ACADA Robotics ● [https://acada.dev](https://acada.dev)  
[![ACADA Robotics](https://docs.acada.dev/rosrider_doc/images/logo.svg)](https://acada.dev)