#include "App.h"




/**
 * @brief App 객체를 생성하고 시작 및 대기 절차를 수행한다.
 *
 * @return 시작 실패 시 1, 정상 종료 시 App::Wait() 반환값
 */
int main_entry() {
  App app;

  if (app.Start() != 0) {
    return 1;
  }

  return app.Wait();
}
