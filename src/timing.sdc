create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
# Asynchronous keys/encoder inputs are handled by existing synchronizers.
# ADC input/output delay must be checked against the actual board and 3PA1030.
