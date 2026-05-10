#!/usr/bin/python3
##
# @file wh_tui.py
# @brief wh_tmpl 템플릿 엔진의 Textual 기반 TUI 프론트엔드
#

import sys
import os
import subprocess
import tempfile

# 부모 디렉토리에서 wh_tmpl 임포트 허용
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
import wh_tmpl

from textual.app import App, ComposeResult
from textual.widgets import (
    Header, Footer, ListView, ListItem, Label,
    Input, Button, Static, DirectoryTree
)
from textual.containers import Horizontal, Vertical, ScrollableContainer
from textual.screen import ModalScreen
from textual.binding import Binding
from textual import on


##
# @brief DirectoryTree 위젯을 이용한 디렉토리 선택 모달 화면
#
class DirectorySelectScreen(ModalScreen):

    BINDINGS = [
        Binding("u", "go_up", "상위 디렉토리", show=False, priority=True),
        Binding("c", "create_dir", "새 디렉토리", show=False, priority=True),
    ]

    ##
    # @brief 모달 초기화 — 선택된 템플릿 이름 저장
    #
    # @param prj_name[IN] 선택된 템플릿 프로젝트 이름
    #
    # @return None
    def __init__(self, prj_name: str = ""):
        super().__init__()
        self._prj_name = prj_name

    ##
    # @brief 모달 레이아웃 구성 (DirectoryTree 포함)
    #        DirectoryTree 의 루트는 앱 실행 시점의 작업 디렉토리로 설정
    #
    # @return ComposeResult
    def compose(self) -> ComposeResult:
        with Vertical(id="dir-modal"):
            yield Label("대상 디렉토리 선택", id="dir-modal-title")
            yield Label("\\[ u ] 상위 디렉토리   \\[ c ] 새 디렉토리 생성", id="dir-modal-hint")
            yield DirectoryTree(os.getcwd(), id="dir-tree")
            with Horizontal(id="dir-modal-buttons"):
                yield Button("선택", id="btn-dir-select", variant="primary")
                yield Button("취소", id="btn-dir-cancel", variant="default")

    ##
    # @brief 트리에서 지정된 경로의 노드로 포커스 이동
    #
    # @param tree[IN]     DirectoryTree 위젯
    # @param target[IN]   포커스할 디렉토리의 전체 경로
    #
    # @return None
    def _focus_new_dir(self, tree: DirectoryTree, target: str) -> None:
        for node in tree.root.children:
            if node.data and str(node.data.path) == target:
                tree.move_cursor(node)
                break

    ##
    # @brief 선택 버튼 클릭 시 선택된 경로를 반환하며 모달 종료
    #
    # @param event Button.Pressed 이벤트
    #
    # @return None
    @on(Button.Pressed, "#btn-dir-select")
    def on_select(self, event: Button.Pressed) -> None:
        tree = self.query_one("#dir-tree", DirectoryTree)
        if tree.cursor_node and tree.cursor_node.data:
            path = str(tree.cursor_node.data.path)
            self.dismiss(path)
        else:
            self.dismiss(None)

    ##
    # @brief 취소 버튼 클릭 시 None을 반환하며 모달 종료
    #
    # @param event Button.Pressed 이벤트
    #
    # @return None
    @on(Button.Pressed, "#btn-dir-cancel")
    def on_cancel(self, event: Button.Pressed) -> None:
        self.dismiss(None)

    ##
    # @brief u 액션 — 상위 디렉토리로 이동
    #
    # @return None
    def action_go_up(self) -> None:
        tree = self.query_one("#dir-tree", DirectoryTree)
        current = str(tree.path)
        parent = os.path.dirname(current)
        if parent != current:  # 루트(/) 에서는 더 이상 올라가지 않음
            tree.path = parent
        tree.focus()

    ##
    # @brief c 액션 — 새 디렉토리 생성 모달 열기
    #
    # @return None
    def action_create_dir(self) -> None:
        tree = self.query_one("#dir-tree", DirectoryTree)
        if tree.cursor_node and tree.cursor_node.data:
            base_path = str(tree.cursor_node.data.path)
            if not os.path.isdir(base_path):
                base_path = os.path.dirname(base_path)
        else:
            base_path = str(tree.path)
        self.app.push_screen(NewDirInputScreen(self._prj_name), lambda name: self._create_dir(base_path, name))

    ##
    # @brief Tab 키로 디렉토리 트리 → 선택 → 취소 → 디렉토리 트리 순환
    #        u/c 키는 어디서든 동작하도록 스크린 레벨 바인딩으로 처리
    #
    # @param event Key 이벤트
    #
    # @return None
    def on_key(self, event) -> None:
        if event.key == "tab":
            event.prevent_default()
            event.stop()
            focused = self.focused
            if isinstance(focused, DirectoryTree):
                self.query_one("#btn-dir-select", Button).focus()
            elif focused == self.query_one("#btn-dir-select", Button):
                self.query_one("#btn-dir-cancel", Button).focus()
            else:
                self.query_one("#dir-tree", DirectoryTree).focus()
            return

    ##
    # @brief 새 디렉토리를 생성하고 트리를 갱신하며 생성된 디렉토리로 포커스 이동
    #
    # @param base_path[IN] 새 디렉토리를 생성할 부모 경로
    # @param name[IN]      생성할 디렉토리 이름, None 이면 취소
    #
    # @return None
    def _create_dir(self, base_path: str, name) -> None:
        if not name:
            return
        new_path = os.path.join(base_path, name)
        try:
            os.makedirs(new_path, exist_ok=True)
            # 생성된 디렉토리의 부모를 루트로 설정하고 갱신 후 서브 디렉토리 펼침
            tree = self.query_one("#dir-tree", DirectoryTree)
            tree.path = base_path

            def _expand_and_focus():
                tree.root.expand()
                # 갱신 후 생성된 디렉토리 노드로 포커스 이동
                self.app.call_after_refresh(lambda: self._focus_new_dir(tree, new_path))

            self.app.call_after_refresh(_expand_and_focus)
            self.notify(f"디렉토리 생성: {new_path}", severity="information")
        except Exception as e:
            self.notify(f"디렉토리 생성 실패: {e}", severity="error")


##
# @brief 새 디렉토리 이름을 입력받는 모달 화면
#
class NewDirInputScreen(ModalScreen):
    ##
    # @brief 모달 초기화 — 기본 디렉토리 이름 설정
    #
    # @param default_name[IN] 입력 필드의 기본값 (선택된 템플릿 이름)
    #
    # @return None
    def __init__(self, default_name: str = ""):
        super().__init__()
        self._default_name = default_name

    ##
    # @brief 모달 레이아웃 구성 (이름 입력 필드 포함)
    #
    # @return ComposeResult
    def compose(self) -> ComposeResult:
        with Vertical(id="newdir-modal"):
            yield Label("새 디렉토리 이름 입력", id="newdir-title")
            yield Input(value=self._default_name, placeholder="디렉토리 이름", id="newdir-input")
            with Horizontal(id="newdir-buttons"):
                yield Button("생성", id="btn-newdir-ok", variant="primary")
                yield Button("취소", id="btn-newdir-cancel", variant="default")

    ##
    # @brief 생성 버튼 클릭 시 입력값을 반환하며 모달 종료
    #
    # @param event Button.Pressed 이벤트
    #
    # @return None
    @on(Button.Pressed, "#btn-newdir-ok")
    def on_ok(self, event: Button.Pressed) -> None:
        name = self.query_one("#newdir-input", Input).value.strip()
        self.dismiss(name if name else None)

    ##
    # @brief 취소 버튼 클릭 시 None을 반환하며 모달 종료
    #
    # @param event Button.Pressed 이벤트
    #
    # @return None
    @on(Button.Pressed, "#btn-newdir-cancel")
    def on_cancel(self, event: Button.Pressed) -> None:
        self.dismiss(None)

    ##
    # @brief Enter 키로 생성 확정, Escape 키로 취소
    #
    # @param event Key 이벤트
    #
    # @return None
    def on_key(self, event) -> None:
        if event.key == "enter":
            name = self.query_one("#newdir-input", Input).value.strip()
            self.dismiss(name if name else None)
        elif event.key == "escape":
            self.dismiss(None)
class WHTuiApp(App):

    TITLE = "Windheim's Template Engine"

    CSS = """
    #main-layout {
        height: 1fr;
    }

    #left-panel {
        width: 30%;
        border: solid $accent;
        padding: 0 1;
    }

    #left-title {
        background: $accent;
        color: $text;
        padding: 0 1;
        margin-bottom: 1;
    }

    #right-panel {
        width: 70%;
    }

    #desc-panel {
        height: 25%;
        border: solid $accent;
        padding: 0 1;
    }

    #desc-title {
        background: $accent;
        color: $text;
        padding: 0 1;
        margin-bottom: 1;
    }

    #desc-text {
        color: $text-muted;
    }

    #marker-panel {
        height: 1fr;
        border: solid $accent;
        padding: 0 1;
    }

    #marker-title {
        background: $accent;
        color: $text;
        padding: 0 1;
        margin-bottom: 1;
    }

    #btn-panel {
        height: 5;
        border: solid $accent;
        align: center middle;
        padding: 0 1;
    }

    .marker-row {
        height: 3;
        margin-bottom: 1;
    }

    .marker-label {
        width: 30;
        padding-top: 1;
    }

    .marker-input {
        width: 1fr;
    }

    #btn-confirm {
        margin: 0 2;
    }

    /* 디렉토리 선택 모달 */
    #dir-modal {
        background: $surface;
        border: solid $accent;
        width: 70%;
        height: 70%;
        padding: 1 2;
    }

    #dir-modal-title {
        background: $accent;
        color: $text;
        padding: 0 1;
        margin-bottom: 1;
    }

    #dir-modal-hint {
        color: $text-muted;
        margin-bottom: 1;
    }

    #dir-tree {
        height: 1fr;
        border: solid $panel;
    }

    #dir-modal-buttons {
        height: 3;
        align: center middle;
        margin-top: 1;
    }

    /* 새 디렉토리 생성 모달 */
    #newdir-modal {
        background: $surface;
        border: solid $accent;
        width: 50%;
        height: auto;
        padding: 1 2;
    }

    #newdir-title {
        background: $accent;
        color: $text;
        padding: 0 1;
        margin-bottom: 1;
    }

    #newdir-buttons {
        height: 3;
        align: center middle;
        margin-top: 1;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "종료", show=True),
    ]

    ##
    # @brief 앱 상태 변수 초기화
    #
    # @return None
    def __init__(self):
        super().__init__()
        self._prj_list = []
        self._selected_prj = None

    ##
    # @brief 메인 레이아웃 구성
    #
    # @return ComposeResult
    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal(id="main-layout"):
            # 좌측 패널: 템플릿 목록
            with Vertical(id="left-panel"):
                yield Label("Template List", id="left-title")
                yield ListView(id="tmpl-list")

            # 우측 패널
            with Vertical(id="right-panel"):
                # 설명 패널
                with Vertical(id="desc-panel"):
                    yield Label("Description", id="desc-title")
                    yield Static("", id="desc-text")

                # 마커 패널
                with Vertical(id="marker-panel"):
                    yield Label("Markers", id="marker-title")
                    yield ScrollableContainer(id="marker-container")

                # 확인 / 취소 버튼 패널
                with Horizontal(id="btn-panel"):
                    yield Button("확인", id="btn-confirm", variant="primary")

        yield Footer()

    ##
    # @brief 앱 마운트 시 템플릿 목록 로드
    #
    # @return None
    def on_mount(self) -> None:
        self._load_tmpl_list()

    ##
    # @brief wh_tmpl에서 템플릿 프로젝트 목록을 불러와 ListView에 출력
    #
    # @return None
    def _load_tmpl_list(self) -> None:
        (ret, prj_list) = wh_tmpl.Get_Tmpl_Prj_List()
        if not ret:
            return
        self._prj_list = prj_list
        self._prj_list.sort(key=lambda x: x[0])
        lv = self.query_one("#tmpl-list", ListView)
        lv.clear()
        for [prj_name, prj_desc, prj_path, ref_prj] in prj_list:
            lv.append(ListItem(Label(prj_name), id="prj-" + prj_name))

    ##
    # @brief ListView에서 커서 이동 시 우측 패널 즉시 업데이트 (방향키)
    #
    # @param event 하이라이트된 항목 정보를 담은 ListView.Highlighted 이벤트
    #
    # @return None
    @on(ListView.Highlighted, "#tmpl-list")
    def on_tmpl_highlighted(self, event: ListView.Highlighted) -> None:
        if event.item is None:
            return
        item_id = event.item.id  # "prj-<name>" 형식
        if item_id and item_id.startswith("prj-"):
            prj_name = item_id[4:]
            self._select_project(prj_name)

    ##
    # @brief 선택된 프로젝트에 맞게 설명 및 마커 패널 업데이트
    #
    # @param prj_name[IN] 선택된 템플릿 프로젝트 이름
    #
    # @return None
    def _select_project(self, prj_name: str) -> None:
        self._selected_prj = prj_name

        # 프로젝트 컨텍스트 조회
        (ret, prj_ctx) = wh_tmpl.Find_Prj_Ctx(prj_name, self._prj_list)
        if not ret:
            return
        [name, desc, path, ref_prj] = prj_ctx

        # 설명 업데이트
        self.query_one("#desc-text", Static).update(desc)

        # tmpl_prj.yaml에서 마커 로드
        prj_file = os.path.join(path, "tmpl_prj.yaml")
        markers = self._load_markers_from_file(prj_file)

        # 마커 컨테이너 재구성
        container = self.query_one("#marker-container", ScrollableContainer)
        container.remove_children()
        for (marker, default_val, desc_str) in markers:
            row = Horizontal(classes="marker-row")
            row.compose_add_child(Label(marker, classes="marker-label"))
            inp = Input(value=default_val, placeholder=desc_str,
                        classes="marker-input", id="marker-" + marker)
            row.compose_add_child(inp)
            container.mount(row)

    ##
    # @brief tmpl_prj.yaml 파일에서 마커 목록 파싱
    #
    # @param prj_file[IN] tmpl_prj.yaml 파일의 전체 경로
    #
    # @return (마커명, 기본 치환값, 설명) 튜플의 리스트
    def _load_markers_from_file(self, prj_file: str) -> list:
        result = []
        if not os.path.isfile(prj_file):
            return result
        with open(prj_file, 'r') as f:
            ctx = yaml.load(f, Loader=yaml.FullLoader)
        marker_list = ctx.get('MarkerList', {})
        if marker_list is None:
            return result
        for marker_key, marker_val in marker_list.items():
            desc_str = marker_val.get('Desc', '')
            replace  = marker_val.get('Replace', '')
            result.append((marker_key, replace, desc_str))
        return result

    ##
    # @brief UI에서 현재 마커 입력값 수집
    #
    # @return (마커, 치환값) 튜플의 리스트
    def _collect_marker_values(self) -> list:
        result = []
        container = self.query_one("#marker-container", ScrollableContainer)
        for inp in container.query(Input):
            marker = inp.id.replace("marker-", "", 1)
            result.append((marker, inp.value))
        return result

    ##
    # @brief 확인 버튼 클릭 시 디렉토리 선택 모달 열기
    #
    # @param event Button.Pressed 이벤트
    #
    # @return None
    @on(Button.Pressed, "#btn-confirm")
    def on_confirm(self, event: Button.Pressed) -> None:
        if self._selected_prj is None:
            self.notify("템플릿을 먼저 선택하세요.", severity="warning")
            return
        self.push_screen(DirectorySelectScreen(self._selected_prj), self._on_dir_selected)

    ##
    # @brief 디렉토리 선택 모달 종료 후 콜백 처리
    #
    # @param path[IN] 선택된 디렉토리 경로, 취소 시 None
    #
    # @return None
    def _on_dir_selected(self, path) -> None:
        if path is None:
            return
        marker_values = self._collect_marker_values()
        self._apply_template(self._selected_prj, path, marker_values)

    ##
    # @brief 마커 치환 값을 적용하여 선택된 디렉토리에 템플릿 생성
    #        사용자 입력값으로 임시 yaml 파일을 생성한 후
    #        wh_tmpl.py set 명령을 subprocess 로 실행
    #
    # @param prj_name[IN]      템플릿 프로젝트 이름
    # @param target_dir[IN]    출력 대상 디렉토리 경로
    # @param marker_values[IN] (마커, 치환값) 튜플의 리스트
    #
    # @return None
    def _apply_template(self, prj_name: str, target_dir: str, marker_values: list) -> None:
        wh_tmpl_script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "wh_tmpl.py"
        )

        # 사용자 입력값으로 임시 marker yaml 생성
        marker_dict = {}
        for (marker, value) in marker_values:
            marker_dict[marker] = {'Replace': value}

        tmp_yaml = None
        try:
            # 임시 yaml 파일 생성
            with tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', delete=False
            ) as f:
                tmp_yaml = f.name
                yaml.dump({'MarkerList': marker_dict}, f, allow_unicode=True)

            cmd = f"python3 {wh_tmpl_script} set {prj_name} {target_dir} {tmp_yaml}"
            result = subprocess.run(
                cmd, shell=True, universal_newlines=True,
                env=os.environ.copy(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT
            )
            if result.returncode == 0:
                self.notify(f"'{prj_name}' 템플릿이 {target_dir} 에 생성되었습니다.", severity="information")
            else:
                self.notify(f"생성 실패:\n{result.stdout}", severity="error")
        except Exception as e:
            self.notify(f"오류 발생: {e}", severity="error")
        finally:
            # 임시 yaml 파일 삭제
            if tmp_yaml and os.path.exists(tmp_yaml):
                os.remove(tmp_yaml)

    ##
    # @brief Tab 키 입력 시 패널 포커스를 순환 이동
    #        순서: 템플릿 목록 → 마커 → 확인 → 템플릿 목록
    #        마커 패널 내부에서는 위/아래 방향키로 이동
    #
    # @param event Key 이벤트
    #
    # @return None
    def on_key(self, event) -> None:
        focused = self.focused

        # 마커 Input 에서 위/아래 방향키로 마커 간 이동
        if isinstance(focused, Input) and event.key in ("up", "down"):
            container = self.query_one("#marker-container", ScrollableContainer)
            inputs = list(container.query(Input))
            if not inputs:
                return
            idx = inputs.index(focused) if focused in inputs else -1
            if event.key == "down" and idx < len(inputs) - 1:
                inputs[idx + 1].focus()
            elif event.key == "up" and idx > 0:
                inputs[idx - 1].focus()
            event.prevent_default()
            return

        if event.key != "tab":
            return

        # Template List 에서 Tab → 마커 패널로
        if isinstance(focused, ListView):
            event.prevent_default()
            container = self.query_one("#marker-container", ScrollableContainer)
            inputs = container.query(Input)
            if inputs:
                inputs.first().focus()
            else:
                self.query_one("#btn-confirm", Button).focus()
            return

        # 마커 Input 에서 Tab → 바로 Confirm 버튼으로
        if isinstance(focused, Input):
            event.prevent_default()
            self.query_one("#btn-confirm", Button).focus()
            return

        # 버튼 패널에서 Tab → Template List 로
        if isinstance(focused, Button):
            event.prevent_default()
            self.query_one("#tmpl-list", ListView).focus()
            return


##
# @brief TUI 애플리케이션 진입점
#
# @return None
def main():
    app = WHTuiApp()
    app.run()


if __name__ == "__main__":
    main()
