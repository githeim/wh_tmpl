# wh_tmpl TUI 수정 이력

---

TUI frontend에 대한 수정 이력을 적는다.

## 2026-05-10 10:14

### [수정] 디렉토리 선택 모달 Tab 순환
- Tab 순환: 디렉토리 트리 → 선택 → 취소 → 디렉토리 트리

### [수정] u/c 키 스크린 레벨 바인딩으로 변경
- `BINDINGS` 에 `priority=True` 로 등록하여 버튼 포커스 상태에서도 동작
- `action_go_up`, `action_create_dir` 메서드로 분리

### [개선] 코드 정리
- `import tempfile` 중복 제거 (함수 내부 → 파일 상단으로 이미 이동)
- `_focus_panel` 상태 변수 제거 (불필요, `self.focused` 로 직접 판단)
- `on_tmpl_selected` 제거 (`on_tmpl_highlighted` 와 동작 중복)
- `NewDirInputScreen` 아래 불필요한 빈 줄 정리

---

## 2026-05-10 09:33

### [추가] 디렉토리 선택 모달에서 c 키로 새 디렉토리 생성
- `c` 키 입력 시 디렉토리 이름 입력 팝업(`NewDirInputScreen`) 출현
- 입력 필드 기본값은 현재 선택된 템플릿 이름으로 설정
- 생성 후 DirectoryTree 자동 갱신
- 안내 텍스트에 `[ c ] 새 디렉토리 생성` 추가
- 버그 수정: `ModalScreen` 내 `push_screen` → `self.app.push_screen` 으로 변경

---

## 2026-05-10 09:23

### [수정] 앱 타이틀 변경
- `WHTuiApp.TITLE` 을 "Windheim's Template Engine" 으로 설정

### [수정] 템플릿 목록 알파벳 순 정렬
- `_load_tmpl_list` 에서 목록 로드 후 프로젝트명 기준 오름차순 정렬

### [추가] 디렉토리 선택 모달 u 키 안내 텍스트
- 모달 타이틀 아래 `\[ u ] 상위 디렉토리로 이동` 안내 문구 추가
- Textual 마크업 충돌로 `[` 를 `\[` 로 이스케이프 처리

---

## 2026-05-10 08:45

### [수정] 프론트엔드 / 코어 완전 분리
- `_apply_template` 에서 wh_tmpl 내부 함수 직접 호출 방식 제거
- `wh_tmpl.py set <prj_name> <target_dir> <marker.yaml>` subprocess 실행 방식으로 변경
- 사용자 입력 마커값으로 임시 yaml 파일 생성 후 전달, 실행 후 자동 삭제

### [수정] 디렉토리 선택 모달에서 u 키로 상위 디렉토리 이동
- `DirectorySelectScreen` 에 `on_key` 핸들러 추가
- `u` 키 입력 시 현재 트리 루트의 상위 디렉토리로 이동
- `/` (루트) 에서는 더 이상 올라가지 않음

---

## 2026-05-10 08:06

### [추가] Textual 기반 TUI 초기 구현
- `frontend/wh_tui.py` 생성
  - Textual 프레임워크 기반 TUI 메인 애플리케이션 구현
  - 좌측 템플릿 목록 패널 (ListView)
  - 우측 상단 설명 패널 (Static, 읽기 전용)
  - 우측 중단 마커 입력 패널 (Input 위젯 동적 생성)
  - 우측 하단 확인/취소 버튼 패널
  - 확인 버튼 클릭 시 DirectoryTree 기반 디렉토리 선택 모달 팝업
  - Tab 키로 패널 포커스 순환 (템플릿 목록 → 마커 → 확인/취소 → 템플릿 목록)
  - 모든 함수에 Doxygen 포맷 한글 주석 적용
- `wh_tui.sh` 생성
  - 실행 스크립트, `frontend/wh_tui.py` 를 실행

### [추가] 커서 이동 시 우측 패널 즉시 업데이트
- `ListView.Highlighted` 이벤트 핸들러 추가
- 방향키로 템플릿 목록을 이동하면 설명 및 마커 패널이 즉시 업데이트됨
- 기존 `ListView.Selected` (엔터 키) 핸들러는 유지

### [수정] 디렉토리 선택 모달 초기 경로
- `DirectoryTree` 루트 경로를 `/home` 에서 `os.getcwd()` 로 변경
- `wh_tui.sh` 실행 시점의 작업 디렉토리에서 탐색 시작

### [수정] Tab 키 포커스 이동 버그 수정
- Tab 키를 한 번 눌렀을 때 두 번 이동하는 버그 수정
- 원인: Textual 기본 Tab 동작과 커스텀 바인딩 충돌
- 해결: BINDINGS 에 `priority=True` 추가하여 커스텀 바인딩 우선 처리
