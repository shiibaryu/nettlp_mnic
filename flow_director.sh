#!/bin/bash
ETH=enp3s0f0
SADDR=192.168.10.1
DADDR=192.168.10.3
ethtool --features enp3s0f0 ntuple off
ethtool --features enp3s0f0 ntuple on
for x in `seq 0 15`; "do"
        idx=$(( $x % 16 ))
        PORT=$(( 12288 + $idx ))
        cmd="sudo ethtool --config-ntuple enp3s0f0 flow-type udp4 \
                src-ip ${SADDR} dst-ip ${DADDR} \
                src-port $PORT dst-port $PORT action $idx"
        echo $cmd
        $cmd
"done"
ethtool --show-ntuple enp3s0f0 
