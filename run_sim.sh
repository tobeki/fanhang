#!/usr/bin/env bash
# =============================================================================
# Manual Return 完整仿真启动脚本
# -----------------------------------------------------------------------------
# 作用：
#   1) 把本次仿真的所有日志集中保存到项目根目录的 logs/ 文件夹；
#   2) 终端输出（SUMMARY / ERROR / 节点 screen 输出）同时 tee 到
#      logs/console_<时间戳>.log；
#   3) ROS 内部日志（roslaunch-*.log 以及每个节点的 .log）重定向到
#      logs/<run_id>/ 目录。
#
# 用法：
#   cd ~/fanhang
#   bash run_sim.sh
#   bash run_sim.sh scenario:=turn90        # 传入 launch 参数（如场景标签）
#
# 触发返航（另开一个终端）：
#   rosservice call /manual_return/trigger "{}"
# =============================================================================
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$WS_DIR/logs"
mkdir -p "$LOG_DIR"

# 把 ROS 日志根目录从默认的 ~/.ros/log 改到项目 logs 目录，
# 这样每次 roslaunch 都会在 logs/ 下新建一个 <run_id>/ 子目录。
export ROS_LOG_DIR="$LOG_DIR"

source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
CONSOLE_LOG="$LOG_DIR/console_$STAMP.log"

echo "======================================================================"
echo " ROS_LOG_DIR  = $ROS_LOG_DIR"
echo " 节点日志目录 = $ROS_LOG_DIR/<run_id>/"
echo " 终端输出日志 = $CONSOLE_LOG"
echo "======================================================================"

# 终端输出同时写到屏幕和日志文件；"$@" 透传额外 launch 参数（如 scenario:=turn90）
roslaunch manual_return_planner manual_return_mid360.launch "$@" 2>&1 | tee "$CONSOLE_LOG"
