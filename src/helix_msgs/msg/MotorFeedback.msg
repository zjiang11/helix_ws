uint32 timestamp          # Timestamp in milliseconds since rover startup
uint8 motor_id            # Node ID of motor controller
float32 battery_voltage   # Battery voltage in Volts
float32 motor_current     # Motor current in Amps
int16 motor_power         # Motor command (-1000 to 1000)
int16 motor_rpm           # Motor speed in RPM
float64 motor_controller_temp  # Motor controller temperature (Celsius)
uint8 fault_flag          # Fault flag, encoded as a bit field
uint16 status_flag