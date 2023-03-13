#!/bin/bash

mkdir -p build && cd build 

skip="false"
while getopts 's' OPTION; do
	case "$OPTION" in
	  s) skip="true" ;;
	esac
done


if [[ $skip == false ]];
then cmake .. -DCMAKE_BUILD_TYPE=Release
fi

sudo make
