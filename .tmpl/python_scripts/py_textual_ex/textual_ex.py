#!/bin/python3

from textual.app import App, ComposeResult
from textual.widgets import Header, Footer

from textual.widgets import Button, Footer, Header, Static
from textual.widgets import Label, ListItem, ListView, Log,RichLog,DirectoryTree

Options ={ 
    "touch A":    "create file A",
    "rm A":       "delete file A",
    "touch B":    "create file B",
    "rm A":       "delete file A",
    "cat ls > C": "create file C and write directory info to file C",
    "rm C":       "delete file C"
          } 

# 일반 ListItem으로는 레이블을 확인할 수 없어서 wrapper 클래스 추가
class LabelItem(ListItem):
    def __init__(self, label: str) -> None:
        super().__init__()
        self.label = label

    def compose( self ) -> ComposeResult:
        yield Label(self.label)

class Evt_List(Static) :
  def compose (self) -> ComposeResult:

    Items =[]
    ListViewInstance = ListView()
    # Options 에 있는 메시지들을 선택지로 등록한다
    for option in Options :
      ListViewInstance.append(LabelItem(option))

    yield ListViewInstance


class WW_ProjectName_WW_App(App):
  """A Textual app example """

  BINDINGS = [
      ("d", "toggle_dark", "Toggle dark mode"),
      ("q", "quit", "quit")
      ]
  CSS_PATH = "layout.tcss"
  def compose(self) -> ComposeResult:
    """Create child widgets for the app."""
    # Panel01 : top -    right  Label 
    Panel01=Label(id="Panel01")
    Panel01.styles.border=("tall", "white")
    # Panel02 : bottom - left   RichLog
    Panel02=RichLog(id="Panel02")
    Panel02.styles.border=("tall", "red")
    Panel02.write("This panel is RichLog")
    # Panel03 : bottom - right  DirectoryTree
    Panel03=Button("Button Example",id="Panel03")
    Panel03.styles.border=("tall", "green")

    yield Header()
    yield Evt_List()
    yield Panel01
    yield Panel02
    yield Panel03
    yield Footer()

    
  def action_toggle_dark(self) -> None:
    """An action to toggle dark mode."""
    self.dark = not self.dark

  # callback for highlighted item
  def on_list_view_highlighted(self, event: ListView.Selected):
    print (event.item.label)
    SelectedItem = event.item.label
    # Get description of command
    self.query_one("#Panel01",Label).update(Options[SelectedItem])

  # callback for selected item
  def on_list_view_selected(self, event: ListView.Selected):
    print("===================")
    print (event.item.label)
    print("===================")


if __name__ == "__main__":
  app = WW_ProjectName_WW_App()
  app.run()

