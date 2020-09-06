#!/usr/bin/python3
import subprocess
import os
import sys
import unittest
import urwid
import wh_tmpl

choices = u'Chapman Cleese Gilliam Idle Jones Palin'.split()


class CMenuFrame :
  ##
  # @brief Setup Menu Frame
  #
  # @param strTitle[IN] Title of the menu
  # @param menu_ctx[IN] Menu item context
  #                     example)                 
  #                     [                  
  #                       ['Start'  , 'Start program'],
  #                       ['Option' , 'setup options'],
  #                       ['Exit'   , 'exit program'],
  #                     ]
  # @return 
  def __init__(self,strTitle, Menu_Ctx, strInstallationPath):
    self.m_strTitle           = strTitle
    self.m_Menu_Ctx           = Menu_Ctx
    self.m_strInstallationPath =strInstallationPath

    # :x: menu ctx list's item offset
    self.m_iOffset_Tmpl_Title = 0
    self.m_iOffset_Tmpl_Info  = 1 

    # Main Palette
    self.m_Palette = [
        (None,  'light gray', 'black'),
        ('heading', 'black', 'light gray'),
        ('line', 'black', 'light gray'),
        ('options', 'dark gray', 'black'),
        ('target path', 'white', 'dark blue'),
        ('focus heading', 'white', 'dark red'),
        ('focus line', 'white', 'dark blue'),
        ('focus options', 'black', 'light gray'),
        ('selected', 'white', 'dark blue')]
    

    # info text of the selected item
    self.m_txtInfoText =urwid.Text(('selected',u"""Selected Item's info"""))
    
    # Create Main Loop
    placeholder = urwid.SolidFill('.')
    self.m_MainLoop = urwid.MainLoop(placeholder,self.m_Palette,
                                     unhandled_input = self.CB_Exit_Key)
    # Create Selection Menu
    self.m_Menu = self.Create_Menu (self.m_strTitle, self.m_Menu_Ctx,
                                    self.CB_Choice,  self.CB_Item_Change)
    # Create the main frame
    self.m_MainFrame = self.Create_Frame(self.m_Menu,self.m_txtInfoText)

    self.m_MainLoop.widget = self.m_MainFrame

  def CB_Selection_Done(self,button):
    self.m_bComplete = True
    raise urwid.ExitMainLoop()

  def CB_Exit_Key(self, key):
    if key in ('q', 'Q'):
      self.m_bComplete=False
      raise urwid.ExitMainLoop()

  def CB_Choice(self, button, choice):
    response = urwid.Text([u'template : ', choice, u'\n'
                           u'target path : ',self.m_strInstallationPath,u'\n'
                          ])
    btn_Yes = urwid.Button(u'Yes')
    btn_No = urwid.Button(u'No')
    urwid.connect_signal(btn_Yes, 'click', self.CB_Selection_Done)
    urwid.connect_signal(btn_No, 'click', self.CB_BacktoMain)
    self.m_MainLoop.widget = urwid.Filler(
              urwid.Pile([response,
                    urwid.AttrMap(btn_Yes, None, focus_map='reversed'),
                    urwid.AttrMap(btn_No, None, focus_map='reversed')
                    ]))

  def CB_BacktoMain(self,button):
    self.m_MainLoop.widget = self.m_MainFrame

  def CB_Item_Change(self):
    # :x: to sync the selected item idx 
    # :x: 1 ; menu title, 2 ; divider, 3; target path edit text
    menu_item_base_idx = 3
    index = self.m_Menu.get_focus()[1] - menu_item_base_idx

    self.m_iIndexSelected = index
    self.m_txtInfoText.set_text(
                               self.m_Menu_Ctx[index][self.m_iOffset_Tmpl_Info])
    self.m_Menu_Ctx[self.m_iOffset_Tmpl_Info]

  def CB_On_Target_Path_Change(self,edit, new_edit_text):
    self.m_strInstallationPath = new_edit_text

  def Create_Menu(self, strTitle, MenuItem, 
                  callback_Choice, callback_Item_Change):
    body = [urwid.Text(strTitle), urwid.Divider()]
    # the path that is applied the template
    target_path = urwid.Edit(
                         caption=('target path', u"Target Path\n"),
                         edit_text=self.m_strInstallationPath
                         )
    urwid.connect_signal(target_path,'change',self.CB_On_Target_Path_Change)
    body.append(target_path)
    for item in MenuItem:
      strMenuTitle = item[self.m_iOffset_Tmpl_Title]
      button = urwid.Button(strMenuTitle)
      urwid.connect_signal(button, 'click', callback_Choice, strMenuTitle)
      body.append(urwid.AttrMap(button, None, focus_map='streak'))
  
    walker = urwid.SimpleFocusListWalker(body)
    urwid.connect_signal(walker,"modified", callback_Item_Change)
  
    body.append(urwid.Text(strTitle))
    return urwid.ListBox(walker)

  def Create_Frame(self, menu,info_text):
    return urwid.Frame(body=menu,footer=info_text)


  def Run(self):
    bRet = True;
    iIdx = 0
    ret = self.m_MainLoop.run()
    bRet = self.m_bComplete
    iIdx = self.m_iIndexSelected
    strInstallationPath = self.m_strInstallationPath
    strSelectedItem = self.m_Menu_Ctx[iIdx][self.m_iOffset_Tmpl_Title]

    Output = (iIdx,strSelectedItem,strInstallationPath);
    return (bRet,Output)

def Create_Menu_Ctx_from_wh_tmpl(wh_tmpl_ctx):
  Offset_Tmpl_Title = 0
  Offset_Tmpl_Info =  1 

  menu_ctx =[]
  for item in wh_tmpl_ctx:
    menu_ctx.append(
          # Menu Item Title         # Menu Item Info
        [ item[Offset_Tmpl_Title], item[Offset_Tmpl_Info] ])
  return menu_ctx

def Create_wh_tmpl_cmd(ctx):
  strSelectedTemplate = ctx[1]
  strInstallationPath = ctx[2]
  strCmd = 'wh_tmpl.py set '+strSelectedTemplate+' '+strInstallationPath
  print('execution command : ' + strCmd)
  return strCmd


def main() :
  default_install_path = os.getcwd()+'/test'

  (ret,prj_list) = wh_tmpl.Get_Tmpl_Prj_List()
  # sort the templates by the name
  prj_list.sort(key=lambda x:x[0])

  menu_ctx = Create_Menu_Ctx_from_wh_tmpl(prj_list)
  strTitle = """Windheim's template engine \nChoose the template"""

  MainMenu = CMenuFrame(strTitle,menu_ctx,default_install_path)
  MainMenu.m_strInstallationPath = default_install_path
  (ret,Output) = MainMenu.Run()
  if (ret == True) :
    strJobCmd = Create_wh_tmpl_cmd(Output)
    print ("Command Line ; "+ strJobCmd)
    local_env = os.environ.copy()
    result = subprocess.run(strJobCmd, shell=True, universal_newlines=True,
        env=local_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    print (result.stdout)

# Write Unit Test Here

class general_unit_test(unittest.TestCase):
  def setUp(self):
    print("setup")

  def tearDown(self):
    print("teardown")

  def test_ex00(self):
    self.assertTrue(True)

if __name__ == '__main__':
  exit( main())

