#!/bin/bash
# Convert nerve map into LTLMoP regions format and copy it into place
# Bring up a vim editor so you can manually move the calibration matrix in

python utils/convert_map.py ../Robot/subtle_world/topo_maps/nerve.map > convert_map_temp
mv ../Robot/subtle_world/topo_maps/nerve.regions ../LTLMoP/src/examples/gumbotest/
vi -o convert_map_temp ../LTLMoP/src/examples/gumbotest/configs/ros.config
rm convert_map_temp
