#!/bin/bash

KLIPPER_PATH="${HOME}/klipper"
REPO_PATH="${HOME}/accelerometer-endstop"
EXTENSIONS="accelerometer_endstop"
green=$(echo -en "\e[92m")
red=$(echo -en "\e[91m")
cyan=$(echo -en "\e[96m")
white=$(echo -en "\e[39m")

set -eu
export LC_ALL=C


function preflight_checks {
    if [ "$EUID" -eq 0 ]; then
        echo "[PRE-CHECK] This script must not be run as root!"
        exit -1
    fi

    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F 'klipper.service')" ]; then
        printf "[PRE-CHECK] Klipper service found! Continuing...\n\n"
    else
        echo "[ERROR] Klipper service not found, please install Klipper first!"
        exit -1
    fi
}

function uninstall_extension {
    local yn
    while true; do
        read -p "${cyan}Do you really want to uninstall Accelerometer-Endstop? (Y/n):${white} " yn
        case "${yn}" in
          Y|y|Yes|yes)
            for extension in ${EXTENSIONS}; do
              if [ -L "${KLIPPER_PATH}/klippy/extras/${extension}.py" ]; then
                  rm -f "${KLIPPER_PATH}/klippy/extras/${extension}.py"
              fi
            done
            break;;
          N|n|No|no|"")
            exit 0;;
          *)
            echo "${red}Invalid Input!";;
        esac
    done
}

function restart_klipper {
    echo "[POST-UNINSTALL] Restarting Klipper..."
    sudo systemctl restart klipper
}

preflight_checks
uninstall_extension
restart_klipper
