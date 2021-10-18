#!/bin/bash

while true; do 
	foo=$(cat /proc/meminfo | grep MemAvailable | awk '{print $2, $3}')
	rostopic pub /memfree std_msgs/String "$foo" --once
	sleep 1
done
