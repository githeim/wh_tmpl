#!/bin/python3
import subprocess

from textual.app import App, ComposeResult
from textual.widgets import Header, Footer

from textual.widgets import Button, Footer, Header, Static, Input
from textual.widgets import Label, ListItem, ListView, Log,RichLog,DirectoryTree
from textual.screen import ModalScreen
from textual.containers import Grid
from textual import events 
import wh_tmpl
import os

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
    def __init__(self, label: []) -> None:
        super().__init__()
        self.label = label

    def compose( self ) -> ComposeResult:
        yield Label(self.label)

class PrjList(Static) :
  def compose (self) -> ComposeResult:
    Items =[]
    ListViewInstance = ListView()
    # _mapPrj 에 있는 메시지들을 선택지로 등록한다
    for option in self._mapPrj :
      ListViewInstance.append(LabelItem(option))
    yield ListViewInstance

class PrjCustomizing(Static) :
  def compose (self) -> ComposeResult:
    # Default install path
    default_install_path = os.getcwd()+'/test'
    yield Label("Install Directory")
    yield Input(placeholder="Target Directory to set up",
                value=default_install_path,
                validate_on=["changed"]
                )
    yield Input()
    yield Input()
    yield Input()
#    yield Input(
#            placeholder="Enter a number...",
#            validators=[
#                Number(minimum=1, maximum=100),  
#                Function(is_even, "Value is not even."),  
#                Palindrome(),  
#            ],
#        )

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


class WH_Tmpl(App):
  """A Textual app example """

  BINDINGS = [
      ("q", "quit", "quit")
      ]
  CSS_PATH = "layout.tcss"
  def compose(self) -> ComposeResult:
    """Create child widgets for the app."""

    # Get Template prj list
    (ret,listPrj) = wh_tmpl.Get_Tmpl_Prj_List()
    # sort the templates by the name
    listPrj.sort(key=lambda x:x[0])
    mapPrj={}
    # change prject list type to map from list
    for item in listPrj :
      # item --> [template project name, Description ,dir path]
      mapPrj[item[0]] = [item[1],item[2]]
    self._mapPrj=mapPrj;
    panelPrjLIst = PrjList()
    panelPrjLIst._mapPrj={}
    panelPrjLIst._mapPrj=self._mapPrj


    # panelPrjDescription : top-right  Label 
    panelPrjDescription=Label(id="panelPrjDescription")
    panelPrjDescription.styles.border=("tall", "white")

    # panelPrjCustomizing : bottom - left   
    panelPrjCustomizing = PrjCustomizing()
    panelPrjCustomizing._mapPrj=self._mapPrj

    # Panel03 : bottom - right  DirectoryTree
    Panel03=Button("Button Example",id="Panel03")
    Panel03.styles.border=("tall", "green")

    yield Header()
    yield panelPrjLIst
    yield panelPrjDescription
    yield panelPrjCustomizing
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
    self.query_one("#panelPrjDescription",Label).update(
        self._mapPrj[SelectedItem][0])

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
  app = WH_Tmpl()
  app.run()

