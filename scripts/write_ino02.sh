#!/bin/bash
cd `dirname $0`
echo "`pwd` \n"
cd ..
echo "`pwd` \n"
cd platformio/ino02
echo "`pwd` \n"
platformio run

