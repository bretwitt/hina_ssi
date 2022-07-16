#!/bin/bash

mkdir -p build && cd build 

skip="false"
while getopts 's' OPTION; do
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
then cmake .. -DCMAKE_BUILD_TYPE={$flags}
fi
	
sudo make
