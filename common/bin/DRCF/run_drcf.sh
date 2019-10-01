#!/bin/bash

echo "Run Emulator of the Doosan Robot Controller"

#echo "Total Param = $#, PROG: $0, param1 =$1, param1 =$2"
#$1 = server port : 12345 
#$2 = Robot model      : m0609, m0617, m1013, m1509   

echo "$(dirname "$0")"
echo "$(server_port "$1")"
echo "$(robot model "$2")"

cd "$(dirname "$0")" 

if [ `getconf LONG_BIT` = "64" ]
then
    echo "ARCH: 64-bit"
    echo "$0"
    echo "$1"
    echo "$2"
    ./DRCF64 $0 $1 
else
    echo "ARCH: 32-bit"
    ./DRCF32 $0 $1
fi
