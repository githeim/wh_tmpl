#!/bin/bash
##
# @file wh_tui.sh
# @brief wh_tmpl Textual TUI 프론트엔드 실행 스크립트
#
# @details 스크립트 위치를 기준으로 frontend/ 디렉토리의
#          wh_tui.py 를 실행한다.
#

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
FRONTEND_DIR="$SCRIPT_DIR/frontend"

python3 "$FRONTEND_DIR/wh_tui.py"
