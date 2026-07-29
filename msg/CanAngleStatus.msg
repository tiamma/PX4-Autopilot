uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# timestamp of the CAN feedback sample
float32 angle_setpoint		# current (slew-limited) setpoint [rad]
float32 angle			# actual angle from the device [rad]
bool enabled			# device enable output state
bool communication_ok		# feedback received within timeout
uint16 device_error		# device-reported error code
uint32 rx_count			# number of valid feedback frames received
uint32 tx_count			# number of command frames transmitted
uint32 error_count		# number of checksum / protocol errors
