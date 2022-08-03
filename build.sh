#!/bin/bash

mkdir -p build && cd build 

skip="false"
debug="false"
while getopts 'sd' OPTION; do
	case "$OPTION" in
	  s) skip="true" ;;
	  d) debug="true";;
	esac
done

flags="Release"
if [[ $debug == true ]];
then flags = "Debug"
fi

if [[ $skip == false ]];
then cmake .. -DCMAKE_BUILD_TYPE=Release
fi
	
sudo make
