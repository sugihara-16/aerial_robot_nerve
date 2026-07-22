builtin_interfaces/Time stamp

# Link status and the module selected from the latest valid Plexus frame.
bool link_active
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

# Decoded RS-485 network-test state. Valid remote nodes occupy module slots
# 1..8. Slots are assigned by Plexus_1's node table, not by physical port.
bool rs485_test_valid
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

# One active remote-node ModuleState selected round-robin. If none is active,
# module_index is 0 and the ordinary virtual diagnostic state is published.
float32[3] acceleration
float32[3] angular_velocity
float32[3] magnetic_field
float32[4] quaternion
float32[8] joint_position
float32[8] joint_velocity
float32[8] joint_torque
float32 battery_voltage
