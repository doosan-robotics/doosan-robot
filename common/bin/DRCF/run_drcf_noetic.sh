#!/bin/bash

echo "Run Emulator of the Doosan Robot Controller (for noetic)"

#echo "Total Param = $#, PROG: $0, param1 =$1, param1 =$2"
#$1 = server port : 12345 
#$2 = Robot model      : m0609, m0617, m1013, m1509   

echo "dirname:" "$0" 
echo "server_port:" "$1" 
echo "robot model:" "$2" 


cd "$(dirname "$0")" 

./DRCF64_noetic $1 $2