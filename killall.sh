#!/bin/bash
echo "Killing [p]ointcloud_publisher Process IDs..."
ps -ef | grep [p]ointcloud_publisher | awk {print'$2'}
ps -ef | grep [p]ointcloud_publisher | awk {print'$2'} | xargs kill -9
echo "-------------------------------"
echo "Processess Killed."