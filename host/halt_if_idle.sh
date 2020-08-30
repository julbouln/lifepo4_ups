#!/bin/bash

idle=$(/usr/local/bin/idle)
timeout=300

if [ "$idle" -gt "$timeout" ];then
	/sbin/halt
fi