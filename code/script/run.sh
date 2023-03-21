#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <single_file>"
    exit 1
fi

single_file=$1

road_type=penghu

#gdb --args 
./bin/roadmarking ${single_file} ${single_file}_out ./model_pool/models_${road_type}_example/ ./config/parameter_${road_type}_example.txt
