#!/usr/bin/python3
import os
import sys
import unittest
import enum

def Get_Parent_Dir_Path():
      return os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
sys.path.append(Get_Parent_Dir_Path())
from wh_tmpl import * 


# Write Unit Test Here
class general_unit_test(unittest.TestCase):
  def setUp(self):
    """setup"""
  
  def tearDown(self):
    """teardown"""
 
  def test_FindPrjFile(self):
    pathSet =Find_PrjFiles(
            prj_search_path='./TestFix00',tmpl_file_name='tmpl_prj.yaml') 
    
    testFix = {'./TestFix00/Prj00/tmpl_prj.yaml', 
       './TestFix00/Category01/Prj05/tmpl_prj.yaml', 
       './TestFix00/Category01/Prj06/tmpl_prj.yaml', 
       './TestFix00/Category00/Prj02/tmpl_prj.yaml',
       './TestFix00/Category00/Prj04/tmpl_prj.yaml',
       './TestFix00/Category00/Prj03/tmpl_prj.yaml', 
       './TestFix00/Prj01/tmpl_prj.yaml'}
    self.assertEqual(pathSet,testFix)
            

  def test_Get_Tmpl_Prj_List(self):
    # :x: 
    (ret,prj_list) = Get_Tmpl_Prj_List(
            prj_search_path='./TestFix00') 

  def test_Verify_Tmpl_prj_file(self):
    (ret,log) = Verify_Tmpl_prj_file('./TestFix00/Prj01/tmpl_prj.yaml')

    self.assertTrue(ret)
    # test exceptions - check the required fields
    exceptionList = [
            './ExceptionTestFix00_field_chk/PrjFieldChk00/tmpl_prj.yaml',
            './ExceptionTestFix00_field_chk/PrjFieldChk01/tmpl_prj.yaml',
            './ExceptionTestFix00_field_chk/PrjFieldChk02/tmpl_prj.yaml',
            ]
    for item in exceptionList:
      (ret,log) =Verify_Tmpl_prj_file( item )
      self.assertFalse( ret )
      #print (log)
            

  def test_Chk_Duplicated_Prj(self):
    # :x: duplicated case
    (ret,prj_list) = Get_Tmpl_Prj_List(
            prj_search_path='./ExceptionTestFix01_duplication') 
    (ret,log) =Chk_Duplicated_Prj(prj_list)
    self.assertTrue(ret)

    ## :x: not duplicated case - normal case
    (ret,prj_list) = Get_Tmpl_Prj_List(
            prj_search_path='./TestFix00') 
    (ret,log) =Chk_Duplicated_Prj(prj_list)
    self.assertFalse(ret)
    self.assertEqual(log,"")


  def test_Get_Prj_List_Report(self):
    (ret,log) = Get_Prj_List_Report('./TestFix00')
    self.assertTrue(ret)

  def test_Copy_Tmpl_Prj_file(self):
    ret =False
    target_file_path =None
    (ret,target_file_path) = Copy_Tmpl_Prj_file(
            "Prj06",prj_search_path ='./TestFix00',target_path ='./')
    self.assertTrue(ret)
    # remove copied file
    if ( ret == True) :
      os.remove(target_file_path)

  def test_Get_Markers(self):
    prj_file ='./TestFix01_tmpl_setup/basic_cpp_main/tmpl_prj.yaml'
    (ret,marker_replace_list) = Get_Markers(prj_file)
    self.assertEqual(marker_replace_list ,
         [
          ['WW_ProjectName_WW', 'Basic Test'], 
          ['WW_PrintLetter_WW', 'Hello World']
         ]
            )
  def test_Set_Tmpl_Prj(self):
    # test wrong case ;Create 'basic_cpp_main' template test
    (ret,log) = Set_Tmpl_Prj(target_prj_name = 'basic_cpp_main', 
                 target_dir = './tmp_workspace', 
                 prj_file = None, 
                 prj_search_path = './.tmpl')
    # This should be False , because of wrong prj_seach_path             
    self.assertFalse(ret)
    if ( ret == True) :
      os.rmdir('./tmp_workspace')

    #Create 'basic_cpp_main' template test
    (ret,log) = Set_Tmpl_Prj(target_prj_name = 'basic_cpp_main', 
                 target_dir = './tmp_workspace', 
                 prj_file = None, 
                 prj_search_path = './TestFix01_tmpl_setup')
    if ( ret == True) :
      shutil.rmtree('./tmp_workspace')

if __name__ == '__main__':
  unittest.main() 


