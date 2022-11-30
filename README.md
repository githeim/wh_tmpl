windheim's template Engine
=============================

This is the general template engine for programming. This template engine is de-
signed for C/C++, Python Codings. You can add your template in the directory 
'./.tmpl' and apply the codes where you want. 

# install

```
$ git clone https://github.com/githeim/wh_tmpl.git
$ cd wh_tmpl
$ ./install_wh_tmpl.sh 
```



# How to use

How to Use ;

* Print out template projects list  ;
./wh_tmpl.py list


* Get marker file from template project  ;
./wh_tmpl.py get Template_Prj_Name
ex) ./wh_tmpl.py get hellowolrd


* Set template project to target path ;
   Markers are replaced to default value
./wh_tmpl.py set Template_Prj_Name Target_Dir
ex) ./wh_tmpl.py set hellowolrd ../workspace


* Set template project to target path with marker file(*.yaml);
   Markers are replaced to the values in marker file
./wh_tmpl.py set Template_Prj_Name Target_Dir marker.yaml
ex) ./wh_tmpl.py set hellowolrd ../workspace ./tmpl_helloworld.yaml



