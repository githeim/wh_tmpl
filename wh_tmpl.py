#!/usr/bin/python3
import os
import sys
import unittest
import yaml
import shutil
import distutils.core

def Get_Default_SearchPath():
  return os.path.dirname(os.path.abspath(__file__))+"/.tmpl"

##
# @brief Find the set of template project files in the path
#
# @param prj_search_path[IN]        path to seach
# @param tmpl_file_name[IN]  template project file name
#
# @return set of template project file path
def Find_PrjFiles(
        prj_search_path = Get_Default_SearchPath(),
        tmpl_file_name= 'tmpl_prj.yaml') :
  pathSet = set()
  for (path, dir, files) in os.walk(prj_search_path):
    for filename in files:
      if filename == tmpl_file_name:
        pathSet.add("%s/%s" % (path, filename))
  return pathSet

##
# @brief Check the required field in template project file(*.yaml)
#
# @param path[IN] path to check file
#
# @return (ret,log) 
def Verify_Tmpl_prj_file(path):
  ret = True
  log = []
  chk_field_list = [
          "ProjectName", 
          "Description",
          "MarkerList",
          ]
  with open (path,'r') as fs:
    ctx = yaml.load(fs, Loader=yaml.FullLoader)
    for chkfield in chk_field_list:
      if ( not chkfield in ctx) :
        ret = False
        log.append ("Verify_Tmpl_prj_file() : Cannot find field ;[" + chkfield 
                + "] in file ["+path+"]")
    # :x: if MarkerList has its items, 
    # :x: each items should have Desc & Replace field
    # TODO ; check MarkerList's item
  return (ret,log)

   

##
# @brief get template projects list and its contexts
#
# @return [ [template project name, Description ,dir path],[... ] ]
def Get_Tmpl_Prj_List(prj_search_path = Get_Default_SearchPath(),
                      tmpl_file_name= 'tmpl_prj.yaml'):
  ret = True
  pathSet = Find_PrjFiles(prj_search_path       = prj_search_path ,
                          tmpl_file_name = tmpl_file_name)
  prj_list = []
  for prj_file in pathSet :
    # verify Project File
    (ret,log) = Verify_Tmpl_prj_file(prj_file)
    if (ret == False ):
      print (log)
      continue
    with open (prj_file,'r') as fs:
      ctx = yaml.load(fs, Loader=yaml.FullLoader)
      path = os.path.dirname(prj_file)
      item = []
      item.append(ctx["ProjectName"])
      item.append(ctx["Description"])
      item.append(path)
      if ( 'Reference_Prj' in ctx ) :
        item.append(ctx["Reference_Prj"])
      else : 
        item.append([])

      prj_list.append(item)

  return (ret,prj_list)

##
# @brief check the duplicated project and its context
#
# @param prj_list[IN] project list to find duplication
#
# @return (ret,log) if any duplications, ret = True 
def Chk_Duplicated_Prj(prj_list):
  ret = False
  log = "Duplicated Projects \n"
  prj_dictionary = { }
  for [ProjectName,Desc,Path,ref_prj] in prj_list:
    if ProjectName in prj_dictionary:
        prj_dictionary[ProjectName].append([Desc,Path])
    else :
        prj_dictionary[ProjectName] = [  [Desc,Path] ]
  
  for item in prj_dictionary:
    duplications = len(prj_dictionary[item])
    if duplications > 1 :
      ret = True
      log+= "\nThe Project [%s] is duplicated : There are %d duplications\n" \
              % ( item, duplications )
      for [desc, path] in prj_dictionary[item]  :
        log+= "Description  ; [%s]\nProject Path ; [%s]\n" \
                % (desc,path)
  if ret == False :
    log =""
  return (ret,log)
 
def Get_Prj_List_Report(
        prj_search_path = Get_Default_SearchPath(),
        tmpl_file_name= 'tmpl_prj.yaml') :
  ret = True
  log = ""
  cnt = 0;
  (ret,prj_list) = Get_Tmpl_Prj_List(prj_search_path,tmpl_file_name)
  (Duplret,Dupllog) = Chk_Duplicated_Prj(prj_list)
  if Duplret == True :
    log+=Dupllog
    ret = False
    return (ret,log)
  log+="\nTemplate Project List\n"

  for [prj_name,prj_desc,prj_path,ref_prj] in prj_list:
      log+="\n"
      log+="Project Name : %s\n" % prj_name
      log+="Description  : %s\n" % prj_desc
      log+="Path         : %s\n" % prj_path
      log+="Reference Prj: %s\n" % ref_prj
      log+="\n"
      cnt+=1
  log+="Total %d Project(s) available\n" % cnt
  return (ret,log)

##
# @brief replace files by template prj file
#
# @param replace_target_path[IN]
# @param tmpl_prj_file_path[IN]
#
# @return 
def Replace_Markers(
        replace_target_path,
        marker_replace_list
        ) :
#with open("Stud.txt", "rt") as fin:
#    with open("out.txt", "wt") as fout:
#        for line in fin:
#            fout.write(line.replace('A', 'Orange'))

  # :x: Get files from replace_target_path
  for (path, dir, files) in os.walk(replace_target_path):
    for file in files :
      with open (path+"/"+file,'r') as fin :
        with open(path+"/"+file+"-_tmp_-", "wt",errors='ignore') as fout:
          print("Creating & Replacing :"+path+"/"+file)
          for line in fin:
            for (marker,replace) in marker_replace_list:
              #print ("marker ; %s, %s"% (marker,replace))
              #print (line.replace(marker,replace))
              line = line.replace(marker,replace)
            fout.write(line)
      shutil.copystat(path+"/"+file,path+"/"+file+"-_tmp_-")
      shutil.move(path+"/"+file+"-_tmp_-",path+"/"+file)
  return True


##
# @brief Find specific project context from project list
#
# @param target_prj_name[IN]
# @param prj_list[IN]
#
# @return 
def Find_Prj_Ctx(target_prj_name,prj_list):
  prj_ctx = None
  (Duplret,Dupllog) = Chk_Duplicated_Prj(prj_list)
  if Duplret == True :
    return (False,None)

  for [prj_name,prj_desc,prj_path,ref_prj] in prj_list:
    if (target_prj_name == prj_name) :
      prj_ctx = [prj_name,prj_desc,prj_path,ref_prj]

  if (prj_ctx == None):
    # :x: the case of cannot find target project
    return (False,[None,None,None,None])

  return (True,prj_ctx)

##
# @brief Copy specific template's project file to target path
#
# @param target_prj_name[IN]
# @param prj_search_path[IN]
# @param target_path[IN]
#
# @return (ret,target_file_path) True/False and copied file's path
def Copy_Tmpl_Prj_file(
        target_prj_name,
        prj_search_path =Get_Default_SearchPath(),
        target_path='./') :
  ret = False
  target_file_path =None
  (ret,prj_list) = Get_Tmpl_Prj_List(prj_search_path)
  
  (ret,[prj_name,prj_desc,prj_path,ref_prj]) = Find_Prj_Ctx(target_prj_name,prj_list)
  if (ret == True) :
    target_file_path = target_path+target_prj_name+".tmpl.yaml"
    prj_file_path = prj_path+'/'+'tmpl_prj.yaml'
    shutil.copy2(prj_file_path,target_file_path)
    ret = True

  return (ret,target_file_path)

def Get_Markers(prj_file):
  ret = True
  marker_replace_list = []

  with open (prj_file,'r') as fs:
    ctx = yaml.load(fs, Loader=yaml.FullLoader)
    if (not 'MarkerList' in ctx) :
      return (false,None)
    MarkerList = ctx['MarkerList']
    for marker in MarkerList:
      #print('marker ; %s , replace ; %s' % (marker,ctx['MarkerList'][marker]['Replace']))
      marker_replace_list.append(
              [marker,ctx['MarkerList'][marker]['Replace']])
  return (True,marker_replace_list)

 
##
# @brief Set the project to target directory
#
# @param target_prj_name[IN] the project to set up
# @param target_dir[IN]      directory to set
# @param prj_file[IN]        project file to appy
# @param prj_search_path[IN] project search path (default ./tmpl ) 
#
# @return 
def Set_Tmpl_Prj(target_prj_name, target_dir, prj_file=None,
       prj_search_path = Get_Default_SearchPath()):
  ret = False
  log = ''
  # :x: Get Tmpl prj list
  (ret,prj_list) = Get_Tmpl_Prj_List(prj_search_path)
  if (ret == False) :
    return (ret,log)
  if (len(prj_list) == 0) :
    log+='Err: Set_Tmpl_Prj(); Cannot find any project on [%s]' \
            % prj_search_path
    print(log)
    return (False,log)
  # :x: Get project entity
  (ret,[prj_name,prj_desc,prj_path,ref_prj])  = \
                                        Find_Prj_Ctx(target_prj_name,prj_list)
  if (ret == False) :
    log+= \
       "Err: Can't find project [%s]'s context! can't find project [%s]"% \
       (target_prj_name,target_prj_name)
    print (log)
    return (False,log)
  # :x: Get Marker file ; default is tmpl_prj.yaml
  if (prj_file == None) :
    prj_file = prj_path+"/tmpl_prj.yaml"

  # :x: Apply reference projects
  for prj in ref_prj:
    print ( "setup Reference Project ; [%s] "% (prj))
    print ( "Project file; [%s] "% (prj_file))
    Set_Tmpl_Prj(prj,target_dir,prj_file,prj_search_path)
    print ( "Reference Project ; [%s] is Done \n"% (prj))
  if (ret == False) :
    return (ret,log)
  # :x: Copy template to target dir
  distutils.dir_util.copy_tree(prj_path,target_dir)
  shutil.copymode(prj_path,target_dir)
  # :x: Remove Template Prj file(tmpl_prj.yaml)
  os.remove(target_dir+"/tmpl_prj.yaml")

  # :x: Get markers
  (ret,marker_replace_list) = Get_Markers(prj_file)

  # :x: Replace according to markers
  Replace_Markers(target_dir,marker_replace_list)


  return (ret,log)

def PrintHelp():
  exe_filename = os.path.basename(sys.argv[0])
  print("\n")
  print("How to Use ; \n")
  print("* Print out template projects list  ; ")
  print("$ "+exe_filename+" list")
  print("\n")
  print("* Get marker file from template project  ; ")
  print("$ "+exe_filename+" get Template_Prj_Name")
  print("ex) $ "+exe_filename+" get hellowolrd ")
  print("\n")
  print("* Set template project to target path ; ")
  print("   Markers are replaced to default value ")
  print("$ "+exe_filename+" set Template_Prj_Name Target_Dir")
  print("ex) $ "+exe_filename+" set hellowolrd ../workspace")
  print("\n")
  print("* Set template project to target path with marker file(*.yaml); ")
  print("   Markers are replaced to the values in marker file")
  print("$ "+exe_filename+" set Template_Prj_Name Target_Dir marker.yaml")
  print("ex) $ "+exe_filename+" set hellowolrd ../workspace ./tmpl_helloworld.yaml")

def main(argv) :
  ret = False
  if argv[1] == 'list' :
    (ret,log) =Get_Prj_List_Report()
    if (ret == True) :
      print(log)
  elif len(sys.argv) == 3 and argv[1] == 'get':
    prj_name = argv[2]
    (ret,target_file_path) = Copy_Tmpl_Prj_file(prj_name)
    if (ret == True) :
      print('\nThe file %s is copied\n' % target_file_path)
  elif len(sys.argv) == 4 and argv[1] == 'set':
    target_prj_name = argv[2]
    target_dir = argv[3]
    (ret,log) = Set_Tmpl_Prj(target_prj_name,target_dir)
  elif len(sys.argv) == 5 and argv[1] == 'set':
    target_prj_name = argv[2]
    target_dir = argv[3]
    prj_file = argv[4]
    (ret,log) = Set_Tmpl_Prj(target_prj_name,target_dir,prj_file)




  if (ret == True ) :
    return 0
  else :
    PrintHelp()
    return 1


if __name__ == '__main__':
  if ( len(sys.argv) == 1 ):
    PrintHelp()
    exit(-1)
  exit( main(sys.argv))

