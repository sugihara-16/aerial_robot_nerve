builtin_interfaces/Time stamp

# Link status and the module selected from the latest valid Plexus frame.
bool link_active
uint8 module_count
uint8 module_index
uint32 last_received_cycle
uint32 plexus_timestamp_ms
uint32 last_valid_frame_age_ms

# Cumulative Spinal-side SPI diagnostics since boot.
uint32 attempted_transfers
uint32 completed_transfers
uint32 valid_frames
uint32 invalid_frames
uint32 semantic_errors
uint32 cycle_mismatches
uint32 dma_start_errors
uint32 timeouts
uint32 peripheral_errors

# One ModuleState from the nine-module SPI payload. module_index is round-robin.
float32[3] acceleration
float32[3] angular_velocity
float32[3] magnetic_field
float32[4] quaternion
float32[8] joint_position
float32[8] joint_velocity
float32[8] joint_torque
float32 battery_voltage
