#!/bin/bash

rsync --delete -a -v ./install/ cmeyers@flexo.robotgrave.com:install/
