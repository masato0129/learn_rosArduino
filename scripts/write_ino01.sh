#!/bin/bash

cd `dirname $0`
echo "`pwd` \n"
cd ..
cd platformio/ino1
echo "`pwd` \n"
platformio run

