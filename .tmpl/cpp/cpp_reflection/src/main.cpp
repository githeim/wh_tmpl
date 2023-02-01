#include <stdio.h>
#include <iostream>
#include <functional>
#include <map>
#include <memory>

std::shared_ptr<std::map<std::string,std::function<int(std::string)>>> g_pReflect;

__attribute__ ((constructor)) void InitReflect() {
  printf("\033[1;32m[%s][%d] :x: Init map  \033[m\n",__FUNCTION__,__LINE__);
  g_pReflect = 
    std::make_shared<std::map<std::string,std::function<int(std::string)>>>();
}

std::map<std::string,std::function<int(std::string)>> g_mapReflect = {};

/**
 * @brief Get function from the function name that is registered
 *
 * @param FunctionName[IN] function name to get
 * @param Func[OUT] the function instance
 *
 * @return success 0, when it cannot find registered function 1
 */
int GetReflectFunction(std::string FunctionName,
    std::function<int(std::string)> &Func) 
{
  std::map<std::string,std::function<int(std::string)>>& Reflect =(*g_pReflect);
  if (Reflect.find(FunctionName) == Reflect.end()) {
    printf("\033[1;31m[%s][%d]"
        " :x: Error ; the function name [%s] was not registered "
        " \033[m\n",__FUNCTION__,__LINE__,FunctionName.c_str());
    return 1;
  }
  Func = Reflect[FunctionName];
  return 0;
}

// The following example code shows how to use attribute 'constructor'
int Func_A(std::string strInput) {
  printf("\033[1;32m[%s][%d] :x: Check [%s] \033[m\n",
      __FUNCTION__,__LINE__,strInput.c_str());
  return 0;
}

__attribute__ ((constructor)) void _REGISTER_Func_A();
void _REGISTER_Func_A() {
  printf("\033[1;32m[%s][%d] :x: Register By Manual  \033[m\n",
      __FUNCTION__,__LINE__);
  (*g_pReflect)["Func_A"] = Func_A;

}

// Register macro function for the reflection
#define REGISTER_FUNC(func) \
  __attribute__ ((constructor)) void _REGISTER_##func(); \
  void _REGISTER_##func() {\
    printf("\033[1;32m[%s][%d] :x: Register %s \033[m\n",__FUNCTION__,__LINE__,\
                                                     #func );\
  (*g_pReflect)[#func] = func; \
  }



int main(int argc, char *argv[]) {
  printf("\033[1;33m[%s][%d] :x: WW_ProjectName_WW \033[m\n",
      __FUNCTION__,__LINE__);

  printf("WW_PrintLetter_WW : c++ reflection with macro example\n");


  std::function<int (std::string)> func;
  // Call functions with string function name
  if (!GetReflectFunction("Func_A",func)) {
    func("Call_1");
  }
  if (!GetReflectFunction("Func_B",func)) {
    func("Call_2");
  }

  if (!GetReflectFunction("Func_C",func)) {
    func("Call_3");
  }

  // The function "Func_D" is not registered --> error case
  if (!GetReflectFunction("Func_D",func)) {
    func("Call_3");
  }

  return 0;
}


// Register functions by macro function

int Func_B(std::string strInput) {
  printf("\033[1;32m[%s][%d] :x: Check [%s] \033[m\n",
      __FUNCTION__,__LINE__,strInput.c_str());
  return 0;
}
REGISTER_FUNC(Func_B)


int Func_C(std::string strInput) {
  printf("\033[1;32m[%s][%d] :x: Check [%s] \033[m\n",
      __FUNCTION__,__LINE__,strInput.c_str());
  return 0;
}
REGISTER_FUNC(Func_C)
