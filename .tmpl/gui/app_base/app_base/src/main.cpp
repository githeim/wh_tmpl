#include "Log_Util.h"
#include <stdio.h>
#include "main_entry.h"

/**
 * @brief 애플리케이션의 실제 진입점이다.
 *
 * glog 초기화 후 main_entry()로 제어를 넘긴다.
 *
 * @param[in] argc 명령행 인자 개수
 * @param[in] argv 명령행 인자 목록
 * @return 애플리케이션 종료 코드
 */
int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  LIY("Project app_base\n");
  LIY("app_base\n");

  return main_entry();
}
