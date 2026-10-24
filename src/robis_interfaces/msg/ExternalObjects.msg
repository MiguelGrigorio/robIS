# Spatial observations supplied by a calibrated environment service.
# Each Detection3D.header.stamp is its ORIGINAL observation time, not a heartbeat.
std_msgs/Header header
string source_id
string environment_id
# SHA-256 identifying the navigation map to which positions are aligned.
string map_checksum
# Maximum age of observations; receiver also applies its own upper bound.
float64 validity_sec
robis_interfaces/ExternalObject[] objects
