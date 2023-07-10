#segment errors at (tcp) protocol level
int64 retransmits
int64 badsegments

#errors in udp transmission
int64 udperrors

#system level (MAC layer) errors
int64 tx_retires
int64 rx_dropped


#interface level (NIC statistics) errors
int64 nic_tx_errors
int64 nic_rx_errors
int64 nic_tx_dropped
int64 nic_rx_dropped
