#!/usr/bin/env bash
# 通过 ROS 2 服务设置轨迹插件的参数，确认后启动插件，并在日志旁写一份实验条件文件。
# 前提：repl、xbot2-core、xbot2_gui_server 已经在运行，homing 已经完成。
#
# 用法：
#   ACTUATOR_ID=<编号> LOAD_MASS=<kg> LOAD_RADIUS=<m> ./run_trj.sh <插件名> [参数=值 ...]
# 例子：
#   export ACTUATOR_ID=A0368 LOAD_MASS=2.0 LOAD_RADIUS=0.30 KP=80 KD=8.5
#   ./run_trj.sh trj        j_motor/amplitude=0.3 trj_out=Position log_file=/tmp/trj_A030
#   ./run_trj.sh trj_chirp  j_motor/amplitude=0.3 log_file=/tmp/chirp_A030
#   ./run_trj.sh trj_policy amplitude=0.1 seed=1.0 log_file=/tmp/policy_A010_seed1
#   DRY_RUN=1 ./run_trj.sh trj_chirp j_motor/amplitude=0.3 log_file=/tmp/x   # 只打印，不执行
#
# 参数名不带插件前缀，脚本会自动加上 /<插件名>/。
# 实验条件（写进 <log_file>_meta.yaml，转换数据时自动读取）：
#   必填 ACTUATOR_ID、LOAD_MASS（空载填 0）、LOAD_RADIUS（空载填 0）；
#   选填 KP、KD（驱动器位置环增益）、AMBIENT_TEMP（环境温度 °C，温升实验要填）、NOTE（备注，如 "cold start"）。
#   不想写条件文件时设 META=0。
# 服务名和类型可用环境变量覆盖：XBOT_PARAM_SERVICE、XBOT_PARAM_SRV_TYPE。
set -euo pipefail

SERVICE=${XBOT_PARAM_SERVICE:-/xbotcore/set_parameter_values}
SRV_TYPE=${XBOT_PARAM_SRV_TYPE:-xbot_msgs/srv/SetString}
DRY_RUN=${DRY_RUN:-0}
META=${META:-1}

if [[ $# -lt 1 ]]; then
    sed -n '2,20p' "$0"
    exit 1
fi
plugin=$1
shift

# 拼出 {request: {/插件/参数: 值, ...}}，和手动调用时的格式相同。
entries=()
log_file=""
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
        log_file=${value%.mat}
    fi
    entries+=("/$plugin/$key: $value")
done

# 实验条件：在动电机之前检查，缺了就不启动。
if [[ $META != 0 ]]; then
    number='^[0-9]+([.][0-9]+)?$'
    if [[ -z ${ACTUATOR_ID:-} ]] || ! [[ ${LOAD_MASS:-} =~ $number ]] || ! [[ ${LOAD_RADIUS:-} =~ $number ]]; then
        echo "请设置 ACTUATOR_ID、LOAD_MASS（kg，空载填 0）、LOAD_RADIUS（m，空载填 0），或者 META=0 跳过。" >&2
        exit 1
    fi
    for gain in KP KD AMBIENT_TEMP; do
        if [[ -n ${!gain:-} ]] && ! [[ ${!gain} =~ $number ]]; then
            echo "$gain 应为非负数：${!gain}" >&2
            exit 1
        fi
    done
    if [[ -z $log_file ]]; then
        echo "请给 log_file=...：条件文件写在日志旁边，名字要对得上。" >&2
        exit 1
    fi
    meta_file="${log_file}_meta.yaml"
    if [[ -e $meta_file ]]; then
        echo "$meta_file 已存在：每次实验请用新的 log_file，避免条件文件对错日志。" >&2
        exit 1
    fi
fi

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

# YAML 双引号字符串
quote() {
    local s=${1//\\/\\\\}
    printf '"%s"' "${s//\"/\\\"}"
}

write_meta() {
    local repo commit branch dirty
    repo=$(cd "$(dirname "$0")/../.." && pwd)
    commit=$(git -C "$repo" rev-parse HEAD 2>/dev/null || echo unknown)
    branch=$(git -C "$repo" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
    dirty=$([[ -n $(git -C "$repo" status --porcelain 2>/dev/null) ]] && echo true || echo false)
    {
        echo "plugin: $(quote "$plugin")"
        echo "log_file: $(quote "$log_file")"
        echo "started: $(quote "$(date -Iseconds)")"
        echo "host: $(quote "$(hostname)")"
        echo "actuator_id: $(quote "$ACTUATOR_ID")"
        echo "load_mass_kg: $LOAD_MASS"
        echo "load_radius_m: $LOAD_RADIUS"
        echo "drive_kp: ${KP:-null}"
        echo "drive_kd: ${KD:-null}"
        echo "ambient_temp_c: ${AMBIENT_TEMP:-null}"
        echo "note: $(quote "${NOTE:-}")"
        echo "config_file: $(quote "$repo/actuator_data_collection/config/xbot2_config.yaml")"
        echo "git_branch: $(quote "$branch")"
        echo "git_commit: $(quote "$commit")"
        echo "git_dirty: $dirty"
        echo "overrides:"
        for e in "${entries[@]}"; do
            echo "  $(quote "${e%%: *}"): $(quote "${e#*: }")"
        done
    }
}

# 启动前再确认一次：插件一启动电机就会动。
read -r -p "确认 homing 已完成，且 $plugin 以外的轨迹插件都已关闭。启动 $plugin？[y/N] " answer
if [[ $answer == y || $answer == Y ]]; then
    run xbot2 plugin switch "$plugin" 1
    if [[ $META != 0 ]]; then
        if [[ $DRY_RUN == 1 ]]; then
            echo "+ 写入 $meta_file："
            write_meta
        else
            write_meta > "$meta_file"
            echo "实验条件已写入 $meta_file（和 .mat 一起拷走）"
        fi
    fi
    echo "已启动。中途停止：xbot2 plugin switch $plugin 0"
else
    echo "没有启动。"
fi
