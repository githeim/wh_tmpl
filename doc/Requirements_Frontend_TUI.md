# wh_tmpl TUI Requirements

## 개요

기존 urwid 기반 `cli_menu.py` 를 Textual 기반 TUI로 교체한다.

---

## 배경

wh_tmpl은 템플릿 프로젝트를 복사하고, 파일 내용 및 파일명에 포함된
치환 마커(`WW_..._WW` 형식)를 사용자가 지정한 값으로 치환하는 템플릿 엔진이다.

치환 마커 예시:
- `WW_ProjectName_WW`
- `WW_Author_WW`

마커는 파일 내용과 파일명 양쪽에 모두 적용된다.

---

## 레이아웃

```
+------------------+---------------------------+
|                  |  Description              |
|  Template List   |  (우측 상단)              |
|                  +---------------------------+
|                  |  Markers                  |
|                  |  WW_ProjectName_WW: [    ]|
|                  |  WW_Author_WW     : [    ]|
|                  +---------------------------+
|                  |  [Confirm]    [Cancel]    |
+------------------+---------------------------+
```

### 패널 구성

| 패널 | 위치 | 역할 |
|------|------|------|
| Template List | 좌측 | 템플릿 목록 표시 및 선택 |
| Description | 우측 상단 | 선택된 템플릿의 설명 출력 (읽기 전용) |
| Markers | 우측 중단 | 치환 마커 목록 및 입력 필드 |
| Confirm/Cancel | 우측 하단 | 실행/취소 버튼 |

---

## 기능 요구사항

### REQ-01 템플릿 선택
- 좌측 패널에 템플릿 목록을 표시한다.
- 템플릿을 선택하면 우측 패널들이 자동으로 업데이트된다.

### REQ-02 Description 출력
- 선택된 템플릿의 description을 우측 상단 패널에 표시한다.
- 읽기 전용이며 Tab 순환 대상에서 제외된다.

### REQ-03 Markers 입력
- 선택된 템플릿에 포함된 치환 마커(`WW_..._WW`)를 우측 중단 패널에 나열한다.
- 각 마커 옆에 입력 필드를 제공하여 치환될 값을 입력할 수 있다.

### REQ-04 Tab 포커스 순환
- Tab 키로 패널 포커스를 순환한다.
- 순서: Template List → Markers → Confirm/Cancel → Template List
- Description 패널은 순환에서 제외한다.

### REQ-05 Confirm 동작
- Confirm 버튼을 누르면 디렉토리 선택 팝업을 띄운다.
- 팝업은 Textual의 `DirectoryTree` 위젯을 `ModalScreen` 안에 구현한다.
- 디렉토리를 선택하면 해당 경로에 템플릿을 생성한다.
- 마커는 입력된 값으로 치환된다.

### REQ-06 Cancel 동작
- Cancel 버튼을 누르면 입력 내용을 초기화하고 초기 상태로 돌아간다.

---

### REQ-07 디렉토리 구조
- TUI 구현 코드는 `frontend/` 디렉토리 안에 위치한다.
- 프로젝트 루트에 `wh_tui.sh` 실행 스크립트를 만든다.
- `wh_tui.sh` 는 `frontend/` 안의 진입점 코드를 실행한다.

### REQ-08 코드 주석
- 모든 구현 함수에 Doxygen 포맷 주석을 달아야 한다.
- 최소 `@brief`, 파라미터(`@param`), 반환값(`@return`) 항목을 포함한다.

---

## 기술 스택

- 언어: Python 3
- TUI 프레임워크: Textual
- 코어 로직: 기존 `wh_tmpl.py` 재사용
