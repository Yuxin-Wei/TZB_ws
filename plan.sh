#!/bin/bash
# set -x
# set -e

echo "[START] Fast-LIO2"

# 通过本脚本文件路径来获取项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"



gnome-terminal -- bash -c "source ${PROJECT_DIR}/devel/setup.bash;roslaunch ego_planner bridge_search.launch"  && sleep 1;

gnome-terminal -- bash -c "source ${PROJECT_DIR}/devel/setup.bash;roslaunch px4ctrlpx4 vins_to_mavros.launch"  && sleep 1;

