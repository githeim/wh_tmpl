SWRS for wh tmpl
======

windheim template Engine

# SWRS

* '.tmpl' 디렉토리에서 템플릿들의 리스트를 얻어낼 수 있어야 한다
* 템플릿 프로젝트를 판별하는 조건; 해당 디렉토리 tmpl_prj.yaml파일이 있으면 된다.
* 템플릿 프로젝트가 있는 디렉토리 및에 중첩으로 템플릿 프로젝트가 존재할 수 없다.
* tmpl_prj.yaml의 문법, MarkerList 밑의 marker는 WW_,_WW 라는 prefix/postfix 포함

ProjectName: "basic_cpp_main"
  Description: "기본적인 cpp main 함수 구현 템플릿"

MarkerList:
  WW_ProjectName_WW: 
    Desc: "프로젝트 명을 기입"
    Replace: "Basic Test"
  WW_PrintLetter_WW: 
    Desc: "출력할 문자열"
    Replace: "Hello World"

* tmpl engine은 템플릿 프로젝트 디렉토리 아래 src 디렉토리 밑에 있는 내용을 모두 타겟 디렉토리에 복사
  * 복사된 파일 내에 있는 marker는 tmpl_prj.yaml에 있는 MarkerList 밑에 있는 marker로 치환

* .tmpl 밑에 중복된 프로젝트(ProjectName이 같은 프로젝트)가 있으면 예외를 발생시키고 중복된 프로젝트 명을 표시한다
  * 중복된 프로젝트 명 / Description / 파일 경로 리스트 출력


# Use case

용례
$ wh_tmpl.py list
--> 사용할 수 있는 템플릿 프로젝트들의 이름과 description을 출력한다
--> 중복이 있는 템플릿 프로젝트가 있으면 그 내용을 출력한다

$ wh_tmpl.py get '템플릿프로젝트명' 
--> 템플릿 프로젝트의 Maker Yaml 파일을 가져온다

$ wh_tmpl.py set '템플릿프로젝트명' '타겟디렉토리' 
--> 타겟 디렉토리에 템플릿 생성 ; 치환 마커 값은 프로젝트의 디폴트 값으로 치환

$ wh_tmpl.py set '템플릿프로젝트명' '타겟디렉토리' 'Marker yaml 파일'
--> 타겟 디렉토리에 템플릿 생성 ; 치환 마커 값은 'Marker yaml 파일' 값으로 치환


