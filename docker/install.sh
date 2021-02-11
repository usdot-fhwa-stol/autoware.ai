#!/bin/bash
source /home/carma/.base-image/init-env.sh
autoware_src="/home/carma/autoware.ai"
cd ${autoware_src}
./autoware/ros/carma_autoware_build -a ${autoware_src}
