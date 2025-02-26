#!/bin/sh
gongxun_v=$(pwd)/..
echo "export gongxun_v=\"$gongxun_v\"" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:\$gongxun_v" >> ~/.bashrc