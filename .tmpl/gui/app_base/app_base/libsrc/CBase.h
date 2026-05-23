#pragma once

#include <functional>
#include <thread>
#include <unordered_map>
#include <memory>
#include "entt/entt.hpp"
#include <atomic>


/**
 * @brief 프레임 제어를 사용할 주체들을 명시하는 열거형이다.
 */
typedef enum {
  MAIN_APP,                    ///< 메인 애플리케이션 루프
  SCENE_CHECKER,               ///< 씬 상태 체커
  NUM_OF_FRAME_CONTROLLER      ///< 프레임 제어기 개수 (센티널)
} FPS_Idx_T;


/**
 * @brief 프레임 제어용 기준 시각 저장소를 초기화한다.
 */
void Init_FPS_Ctrl();
/**
 * @brief 지정한 프레임 제어 채널의 실행 주기를 맞춘다.
 *
 * @param[in]  Idx 제어 대상 프레임 인덱스
 * @param[in]  dbInterval_SEC 목표 프레임 간격(초)
 * @param[out] dbTimeDiff_SEC 실제 경과 시간(초)
 * @return 실제 계산된 FPS
 */
double FPS_Ctrl(const FPS_Idx_T &Idx, const double dbInterval_SEC, 
                double &dbTimeDiff_SEC);

/**
 * @brief 루프 실행 시점을 구분하는 열거형이다.
 */
typedef enum  {
  PRE_LOOP_PROC  = 0x01,  ///< 루프 시작 전에 실행하는 프로시저
  LOOP_PROC      = 0x02,  ///< 루프 진행 중에 실행하는 프로시저
  POST_LOOP_PROC = 0x04,  ///< 루프 종료 후에 실행하는 프로시저
} ProcType_T;
/**
 * @brief 루프 구간별로 실행될 콜백 함수의 타입이다.
 */
typedef std::function<int(entt::registry&)> LoopProc_T;


/**
 * @brief 실행 시점(ProcType_T)과 콜백 함수(LoopProc_T)를 매핑하는 타입이다.
 */
typedef std::unordered_map<ProcType_T,LoopProc_T> Proc_Map_T;

/**
 * @brief 루프 실행 스레드와 전/중/후처리 콜백을 관리하는 베이스 클래스.
 */
class CBase {
public:
  /**
   * @brief 기본 실행 상태를 초기화한다.
   */
  CBase();
  /**
   * @brief 실행 중인 스레드가 남아 있으면 정리한 뒤 소멸한다.
   */
  ~CBase();

  /**
   * @brief 등록된 프로시저 맵을 사용해 메인 루프 스레드를 시작한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @param[in]     mapProc 전/중/후처리 프로시저 맵
   * @return 성공 시 0, 실패 시 0 이외의 값
   */
  int Start(entt::registry& ECS,Proc_Map_T& mapProc);
  /**
   * @brief 내부 실행 스레드가 종료될 때까지 대기한다.
   *
   * @return 대기 처리 결과 코드
   */
  int Wait();
  /**
   * @brief 내부 루프를 중지하고 실행 스레드를 join 한다.
   *
   * @return 중지 처리 결과 코드
   */
  int Stop();

  /**
   * @brief 추가 초기화 확장을 위한 인터페이스 자리표시자다.
   *
   * @return 초기화 결과 코드
   */
  int Init();
  std::atomic<bool> m_bLoopTrigger {false};
  /**
   * @brief 루프 지속 여부 플래그를 조회한다.
   *
   * @return true 이면 루프 계속, false 이면 루프 중지
   */
  bool Get_bLoopTrigger(){ return m_bLoopTrigger.load();};
  /**
   * @brief 루프 지속 여부 플래그를 갱신한다.
   *
   * @param[in] bVal 설정할 루프 지속 여부
   */
  void Set_bLoopTrigger(bool bVal){ m_bLoopTrigger.store(bVal);};

  /**
   * @brief 등록된 전/중/후처리 프로시저를 순서대로 실행한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   * @return 실행 결과 코드
   */
  int OnExecute(entt::registry& ECS);
 
  std::shared_ptr<std::thread> m_thrMain = nullptr;
private:
  LoopProc_T m_procPreLoop;   ///< 루프 시작 전 실행 프로시저
  LoopProc_T m_procLoop;      ///< 메인 루프 프로시저 (필수)
  LoopProc_T m_procPostLoop;  ///< 루프 종료 후 실행 프로시저

};
