#!/bin/python3
import subprocess

from textual.app import App, ComposeResult
from textual.widgets import Header, Footer

from textual.widgets import Button, Footer, Header, Static
from textual.widgets import Label, ListItem, ListView, Log,RichLog,DirectoryTree
from textual.screen import ModalScreen
from textual.containers import Grid
from textual import events 

Options ={ 
    "touch A":    "create file A",
    "rm A":       "delete file A",
    "touch B":    "create file B",
    "rm B":       "delete file A",
    "ls > C": "create file C and write directory info to file C",
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

# Popup for confirm 
class ConfirmScreen(ModalScreen[int]):
  """Screen with a dialog to quit."""
  ConfirmText="Confirm?"
  def compose(self) -> ComposeResult:
    yield Grid(
        Label(self.ConfirmText, id="question"),
        Button("Yes", variant="error", id="Yes"),
        Button("No", variant="primary", id="No"),
        id="dialog"
    )

  def on_button_pressed(self, event: Button.Pressed) -> None:
    print("chk--------------------")
    if   event.button.id == "Yes":
      self.dismiss(0)
    elif event.button.id == "No" : 
      self.dismiss(1)
  def on_key(self, event: events.Key) -> None:
    print ("Key : ") 
    print (event.key) 
    if ( event.key == 'left' ) : 
      self._move_focus(-1)
    elif (event.key == 'right' ) :
      self._move_focus(1)


class Textual_EX_App(App):
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
    def Chk_Confirm(Confirm: int) -> None:
      print ("Confirm : ",Confirm)
      if (Confirm == 0) :
        # execute the item
        subprocess.run(event.item.label, shell=True, universal_newlines=True,
          stdout=subprocess.PIPE,
          stderr=subprocess.STDOUT)
        
    Confirm= ConfirmScreen()
    # Question for confirm - you can edit the question here
    Confirm.ConfirmText="Execute this command?\n"+"$ "+event.item.label
    self.push_screen(Confirm,Chk_Confirm)



if __name__ == "__main__":
  app = Textual_EX_App()
  app.run()

