#pragma once

#include <memory>

/**
 * @brief SDL 기반 앱 프레임워크의 최상위 제어 클래스.
 *
 * 애플리케이션 시작, 대기, 종료 흐름을 단순한 인터페이스로 감싼다.
 */
class App {
public:
  /**
   * @brief 애플리케이션 구현 객체와 런타임 컨텍스트를 준비한다.
   */
  App();
  /**
   * @brief 애플리케이션 객체를 정리한다.
   */
  ~App();

  /**
   * @brief 메인 루프 실행을 시작한다.
   *
   * @return 성공 시 0, 실패 시 0 이외의 값
   */
  int Start();
  /**
   * @brief 실행 중인 메인 루프 스레드가 종료될 때까지 대기한다.
   *
   * @return 대기 처리 결과 코드
   */
  int Wait();
  /**
   * @brief 실행 중인 메인 루프를 중지한다.
   *
   * @return 중지 처리 결과 코드
   */
  int Stop();

private:
  class Impl;
  std::unique_ptr<Impl> m_pImpl;
};
