#!/bin/bash

source ~/env.sh

if [ "$FRT_ROBOT_ID" = "ferenc" ]; then
    ./ferenc
else
    ./viktor
fi
