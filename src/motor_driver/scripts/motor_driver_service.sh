#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: motor_driver_service.sh {enable|disable|reset} [timeout_seconds]

Examples:
  motor_driver_service.sh enable
  motor_driver_service.sh disable 5
  motor_driver_service.sh reset
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
    usage >&2
    exit 2
fi

operation="$1"
timeout_seconds="${2:-5}"

case "$operation" in
    enable)
        service_name="/motor_driver/enable"
        service_type="motor_msgs/srv/EnableMotor"
        ;;
    disable)
        service_name="/motor_driver/disable"
        service_type="motor_msgs/srv/DisableMotor"
        ;;
    reset)
        service_name="/motor_driver/reset"
        service_type="motor_msgs/srv/ResetMotor"
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

if ! [[ "$timeout_seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   (( $(awk "BEGIN { print ($timeout_seconds <= 0) }") )); then
    echo "timeout_seconds must be greater than zero" >&2
    exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command not found; source the ROS 2 environment first" >&2
    exit 127
fi

echo "Calling ${service_name}..."
timeout --foreground "${timeout_seconds}s" \
    ros2 service call "$service_name" "$service_type" "{}"
