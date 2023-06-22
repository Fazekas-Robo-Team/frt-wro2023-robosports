#!/bin/bash

echo "Sourcing env..."
source ~/env.sh

if [ "$FRT_ROBOT_ID" = "ferenc" ]; then
    echo "Starting ferenc..."
    chmod +x ~/bin/ferenc
    ~/bin/ferenc
else
    echo "Starting viktor..."
    chmod +x ~/bin/viktor
    ~/bin/viktor
fi
