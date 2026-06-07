#!/bin/bash
# 헤드리스 모드로 테스트 실행 (ImGui GlobalMouseState 간섭 방지)
echo Test Headless
cd build/Debug ; SDL_VIDEODRIVER=offscreen ./test_WW_ProjectName_WW-WW_MajorVer_WW.WW_MinorVer_WW.out
