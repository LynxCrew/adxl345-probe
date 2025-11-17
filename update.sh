#!/bin/bash

REPO_PATH="${HOME}/accelerometer-endstop"

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

function update_repo {
    cd ${REPO_PATH}
    git fetch origin
    if [ `git rev-list HEAD...origin/main --count` != 0 ]; then
        echo "[UPDATE] Updating Accelerometer-Endstop repository..."
        if git pull origin; then
            printf "[UPDATE] Download complete!\n\n"
        else
            echo "[ERROR] Download of Accelerometer-Endstop git repository failed!"
            exit -1
        fi
    else
        echo "[UPDATE] Repo already up to date."
        exit 0
    fi
}

function install {
    chmod +x ${REPO_PATH}/install.sh
    chmod +x ${REPO_PATH}/update.sh
    chmod +x ${REPO_PATH}/uninstall.sh
    bash ${REPO_PATH}/install.sh
}

preflight_checks
update_repo
install
