#!/bin/bash

rsync --delete -a -v ./install/ cmeyers@flexo.robotgrave.com:bender/ros2/install/
