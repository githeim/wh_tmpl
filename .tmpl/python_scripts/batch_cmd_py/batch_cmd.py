#!/usr/bin/python3
import subprocess
import os
import sys
import unittest
import enum
import datetime

def Get_Parent_Dir():
  return os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
def Get_Current_Dir():
  return os.path.abspath(os.path.dirname(__file__))
# :x: example
def Get_Working_Dir():
  return Get_Current_Dir()+"/testdir"


def JobList():
    return [
        # working directory       # job_cmd

        [ Get_Current_Dir(),  'mkdir ./testdir' ],
        [ Get_Current_Dir(),  'touch ./testdir/fileA' ],
        [ Get_Current_Dir(),  'touch ./testdir/fileB' ],
        [ Get_Current_Dir(),  'touch ./testdir/fileC' ],

        [ Get_Working_Dir(),  'ls -l' ],

        [ Get_Working_Dir(),  'rm file*' ],

        [ Get_Current_Dir(),  'rm -rf testdir' ],


        # test cases/runners
    ]

class JobOffset():
    dir =  0        # directory
    job_cmd =  1  # job command



def DoJobList( JobList ):
  StepCnt = 0
  CurrentDir = os.getcwd()
  ret = True
  # The value For printing report
  strJobReport ="Job Report\n"
  strJobLogReport = "Log Report\n"

  for JobItem in JobList:
    work_dir = JobItem[JobOffset.dir]
    strJobCmd = JobItem[JobOffset.job_cmd]
    StepCnt = StepCnt+1

    strJobLogReport= strJobLogReport+"\n"+\
      "============================================\n"+\
      "Step["+str(StepCnt)+"] Work Dir ["+work_dir+"]\n"+\
      "job cmd  ["+str(strJobCmd)+"]\n"+\
      "============================================\n"

    print("Job #"+str(StepCnt))
    print("check Working Directory ; "+work_dir)
    if os.path.isdir(work_dir) == False:
      print("No working directory ["+work_dir+"]")
      strJobLogReport= strJobLogReport+"No working directory ["+work_dir+"]\n"
      strJobSuccess = "Fail"
      ret = False
    else :
      # :x: Change working directory
      os.chdir(work_dir)
      print("execution job command ; "+strJobCmd)
      print("processing")
      # Get local systems environment variable
      # without this value, it can't get the environment values
      # ex) $PATH $HOME...
      local_env = os.environ.copy()
      output=subprocess.run(strJobCmd, shell=True, universal_newlines=True,
               env=local_env,
               stdout=subprocess.PIPE,
               stderr=subprocess.STDOUT)
      print("processing Done")

      strJobLogReport= strJobLogReport+output.stdout+"\n"
      if ( output.returncode !=0 ):
        print("Job Error ; on ["+str(work_dir)+"]");
        strJobSuccess = "Fail"
        ret = False
      else:
        print("Job Done  ; ["+str(work_dir)+"]");
        strJobSuccess = "Success"
    strJobReport= strJobReport +\
                    "Step[%04d] ;[%7s],[%s],[%s]\n" %(StepCnt,strJobSuccess,work_dir,strJobCmd)
    # :x: back to original working directory
    os.chdir(CurrentDir)
  return (ret,strJobReport,strJobLogReport)



def main() :

  ret = DoJobList(JobList())

  bSuccess = ret[0]
  strJobReport = ret[1]
  strJobLogReport = ret[2]
  print("\n\n\n==============\n"+strJobReport+"\n")
  strJobLogFileName ='job_log_'+datetime.datetime.now().strftime('%m_%d_%H_%M_%S')+'.txt'

  f = open ("./"+strJobLogFileName,'w')
  f.write(strJobLogReport)
  f.close()

  print ("Job log file ; "+strJobLogFileName+"\n")
  if (bSuccess != True):
      print("Job Error Check Report ; "+strJobLogFileName)
      return -1
  print("All Job success")

  return 0

# Write Unit Test Here

class general_unit_test(unittest.TestCase):
  def setUp(self):
    print("setup")

  def tearDown(self):
    print("teardown")

  def test_vundle_install(self):
    self.assertTrue(True)



if __name__ == '__main__':
  print ("chk")
  exit( main())

