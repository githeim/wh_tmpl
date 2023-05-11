#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <chrono>

#include <nlohmann/json.hpp>
#include <string>
#include <fstream>
std::string strJson = "";
static const char* g_JSON_EXAMPLE = R"(                      
{
  "squadName": "Super hero squad",
  "homeTown": "Metro City",
  "formed": 2016,
  "secretBase": "Super tower",
  "active": true,
  "members": [
    {
      "name": "Molecule Man",
      "age": 29,
      "secretIdentity": "Dan Jukes",
      "powers": [
        "Radiation resistance",
        "Turning tiny"
      ]
    },
    {
      "name": "Madame Uppercut",
      "age": 39,
      "secretIdentity": "Jane Wilson",
      "powers": [
        "Million tonne punch",
        "Damage resistance",
        "Superhuman reflexes"
      ]
    }
  ]
}
          
)";                                                            


int main(int argc, char *argv[]) {
  printf("\033[1;33m[%s][%d] :x: WW_PrintLetter_WW \033[m\n",
      __FUNCTION__,__LINE__);

#if 0 
  // File
std::ifstream InputFile("resource/json_example.json");
  nlohmann::json read_instance;
  InputFile >> read_instance;
#else
  // Variable
  (void)g_JSON_EXAMPLE;
  auto read_instance = nlohmann::json::parse(g_JSON_EXAMPLE);
#endif

  read_instance["homeTown"];
  printf("\033[1;33m[%s][%d] :x: homeTown =[%s] \033[m\n",__FUNCTION__,__LINE__,
                          read_instance["homeTown"].get<std::string>().c_str());
  printf("\033[1;33m[%s][%d] :x: formed =[%d] \033[m\n",__FUNCTION__,__LINE__,
                          read_instance["formed"].get<int>());

  for (auto member : read_instance["members"]) {
    printf("\033[1;32m[%s][%d] :x: member name : %s \033[m\n",
        __FUNCTION__,__LINE__,member["name"].get<std::string>().c_str());
    printf("\033[1;32m[%s][%d] :x: age : %d \033[m\n",
        __FUNCTION__,__LINE__,member["age"].get<int>());
  }
  
  // Add new Item
  read_instance["BaseAddress"] ="secret road 103";
  printf("\033[1;33m[%s][%d] :x: BaseAddress =[%s] \033[m\n",
      __FUNCTION__,__LINE__,
      read_instance["BaseAddress"].get<std::string>().c_str());

  nlohmann::json new_item;
  new_item["name"]="Miss Headbutt";
  new_item["age"]="22";
  new_item["secretIdentity"]="Chris Bald";

  read_instance["members"].push_back(new_item);

  std::string strDump = read_instance.dump(1);

  printf("\033[1;36m[%s][%d] :x: Dump \n%s \033[m\n",__FUNCTION__,__LINE__,
      strDump.c_str());

  return 0;
}
