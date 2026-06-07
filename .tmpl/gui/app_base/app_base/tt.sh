#!/bin/bash
# 실시간 시각화 모드: 실제 창이 열리고 마우스가 실제로 움직이며 클릭된다.
# SDL_WarpMouseGlobal 로 커서를 이동 → ImGui GlobalMouseState 로 hover 판정
cd build/Debug ; TEST_VISUAL=1 ./test_WW_ProjectName_WW-WW_MajorVer_WW.WW_MinorVer_WW.out
