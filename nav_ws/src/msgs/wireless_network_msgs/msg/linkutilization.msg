Header header
#TCP related information on link utilization for a given (NIC) interface
int64 tcp_tx_segments
int64 tcp_rx_segments
float64 tcp_tx_segmentrate
float64 tcp_rx_segmentrate

#UDP related information on link utilization for a given (NIC) interface
int64 udp_tx_datagrams
int64 udp_rx_datagrams
float64 udp_tx_datagramrate
float64 udp_rx_datagramrate

#Total (IP: TCP + UDP) link utilization for a given (NIC) interface
int64 total_tx_packets
int64 total_tx_bytes
int64 total_rx_packets
int64 total_rx_bytes
float64 total_tx_mbps
float64 total_rx_mbps
