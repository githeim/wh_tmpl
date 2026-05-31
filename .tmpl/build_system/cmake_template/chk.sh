#!/bin/bash
set -u
echo 지속적으로 프로젝트 상태를 감시하고 빌드를 검증한다
cd "$(dirname "$0")"

run_check() {
  clear
  echo "[chk] build start"
  if ! ./bd.sh; then
    echo "[chk] build failed"
    return 1
  fi

  echo
  echo "[chk] test start"
  if ! ./tt.sh; then
    echo "[chk] test failed"
    return 1
  fi

  echo
  echo "[chk] OK"
  return 0
}

if command -v entr >/dev/null 2>&1; then
  echo "[chk] watch mode with entr"
  printf '%s\n' \
    CMakeLists.txt \
    chk.sh \
    bd.sh \
    tt.sh \
    $(find include libsrc src test cmake -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.c' -o -name '*.cmake' \) | sort) \
    | entr -c sh -c './bd.sh && ./tt.sh'
else
  echo "[chk] entr not found. fallback polling mode"
  echo "[chk] install recommendation: sudo apt install entr"

  last_sig=""
  while true; do
    cur_sig=$( {
      printf '%s\n' CMakeLists.txt chk.sh bd.sh tt.sh
      find include libsrc src test cmake -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.c' -o -name '*.cmake' \) | sort
    } | xargs -r stat -c '%Y %n' 2>/dev/null | md5sum | cut -d' ' -f1 )

    if [ "$cur_sig" != "$last_sig" ]; then
      last_sig="$cur_sig"
      run_check
    fi
    sleep 1
  done
fi
