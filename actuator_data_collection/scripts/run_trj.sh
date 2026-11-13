#!/usr/bin/env bash
# 通过 ROS 2 服务设置轨迹插件的参数，确认后启动插件。
# 前提：repl、xbot2-core、xbot2_gui_server 已经在运行，homing 已经完成。
#
# 用法：
#   ./run_trj.sh <插件名> [参数=值 ...]
# 例子：
#   ./run_trj.sh trj        j_motor/amplitude=0.3 trj_out=Position log_file=/tmp/trj_A030
#   ./run_trj.sh trj_chirp  j_motor/amplitude=0.3 log_file=/tmp/chirp_A030
#   ./run_trj.sh trj_policy amplitude=0.1 seed=1.0 log_file=/tmp/policy_A010_seed1
#   DRY_RUN=1 ./run_trj.sh trj_chirp j_motor/amplitude=0.3   # 只打印命令，不执行
#
# 参数名不带插件前缀，脚本会自动加上 /<插件名>/。
# 服务名和类型可用环境变量覆盖：XBOT_PARAM_SERVICE、XBOT_PARAM_SRV_TYPE。
set -euo pipefail

SERVICE=${XBOT_PARAM_SERVICE:-/xbotcore/set_parameter_values}
SRV_TYPE=${XBOT_PARAM_SRV_TYPE:-xbot_msgs/srv/SetString}
DRY_RUN=${DRY_RUN:-0}

if [[ $# -lt 1 ]]; then
    sed -n '2,15p' "$0"
    exit 1
fi
plugin=$1
shift

# 拼出 {request: {/插件/参数: 值, ...}}，和手动调用时的格式相同。
entries=()
for kv in "$@"; do
    if [[ $kv != *=* ]]; then
        echo "参数格式应为 名字=值：$kv" >&2
        exit 1
    fi
    key=${kv%%=*}
    value=${kv#*=}
    # MatLogger2 把文件名里最后一个点之后当扩展名，只接受 .mat 或无扩展名。
    if [[ $key == log_file ]]; then
        base=${value##*/}
        if [[ $base == *.* && $base != *.mat ]]; then
            echo "log_file 的文件名不能带点（例如 A0.1 写成 A010）：$value" >&2
            exit 1
        fi
    fi
    entries+=("/$plugin/$key: $value")
done

run() {
    echo "+ $*"
    if [[ $DRY_RUN != 1 ]]; then
        "$@"
    fi
}

if [[ $DRY_RUN != 1 ]] && ! ros2 service list | grep -qx "$SERVICE"; then
    echo "找不到服务 $SERVICE。当前与参数相关的服务：" >&2
    ros2 service list | grep -i param >&2 || true
    echo "确认名字后用 XBOT_PARAM_SERVICE=<服务名> 重新运行。" >&2
    exit 1
fi

if [[ ${#entries[@]} -gt 0 ]]; then
    payload="{request: {$(IFS=,; echo "${entries[*]}" | sed 's/,/, /g')}}"
    run ros2 service call "$SERVICE" "$SRV_TYPE" "$payload"
else
    echo "没有给参数，使用插件当前的参数值。"
fi

# 启动前再确认一次：插件一启动电机就会动。
read -r -p "确认 homing 已完成，且 $plugin 以外的轨迹插件都已关闭。启动 $plugin？[y/N] " answer
if [[ $answer == y || $answer == Y ]]; then
    run xbot2 plugin switch "$plugin" 1
    echo "已启动。中途停止：xbot2 plugin switch $plugin 0"
else
    echo "没有启动。"
fi
