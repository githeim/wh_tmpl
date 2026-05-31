#!/bin/bash
set -u

cd "$(dirname "$0")"

# 소스 파일 목록 해시 (이름만, 추가/삭제 감지용)
get_filelist_sig() {
  find include libsrc src test cmake -type f \
    \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \
       -o -name '*.c' -o -name '*.cmake' \) \
    2>/dev/null | sort | md5sum | cut -d' ' -f1
}

SIG_FILE="build/.filelist_sig"
cur_sig=$(get_filelist_sig)
prev_sig=""
[ -f "$SIG_FILE" ] && prev_sig=$(cat "$SIG_FILE")

if [ "$cur_sig" != "$prev_sig" ]; then
  echo "[bd] file list changed, running cmake reconfigure..."
  cmake -GNinja -B build -S . || exit 1
  echo "$cur_sig" > "$SIG_FILE"
fi

echo Build
cd build && ninja -v -j"$(nproc)" "$@"
