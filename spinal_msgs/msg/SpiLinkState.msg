builtin_interfaces/Time stamp

# Link status and the module selected from the latest valid Plexus frame.
bool link_active
uint32 local_node_id
uint32 plexus_node_id
bool node_identity_valid
uint8 module_count
uint8 module_index
uint8 active_rs485_node_count
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
uint32 node_identity_mismatches

# Decoded RS-485 network-test state. Valid remote nodes occupy module slots
# 1..8. Slots are assigned by Plexus_1's node table, not by physical port.
bool rs485_test_valid
bool remote_imu_valid
uint32 remote_imu_timestamp_ms
bool remote_joint_valid
uint32 remote_joint_timestamp_ms
uint8 remote_joint_count
int16[8] remote_joint_position_raw
int16[8] remote_joint_load_raw
# Zero-based slot in Plexus_1's remote-node table.
uint8 rs485_node_slot
# Plexus_1 ingress port for the complete route to this node.
uint8 rs485_port_index
uint8 rs485_physical_port
uint32 rs485_transaction_id
uint32 rs485_input_value
uint32 rs485_responder_node_id
uint32 rs485_result_value
# Immediate parent of the responder and the physical port on that parent.
uint32 rs485_parent_node_id
uint8 rs485_parent_physical_port
# Physical port used by the responder itself to reach its immediate parent.
uint8 rs485_upstream_physical_port
uint8 rs485_hop_count

# One active remote-node ModuleState selected round-robin. For a valid remote
# node these fields contain that node's Spinal estimator output. If none is
# active, module_index is 0 and the ordinary virtual diagnostic state is used.
float32[3] acceleration
float32[3] angular_velocity
float32[3] magnetic_field
float32[4] quaternion
# For compatibility and rqt_plot convenience, valid remote joint samples are
# mirrored here as float values. Position and torque contain raw servo position
# and raw load respectively; velocity is zero because it is not measured.
float32[8] joint_position
float32[8] joint_velocity
float32[8] joint_torque
float32 battery_voltage
