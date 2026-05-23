#include <chrono>
#include <unistd.h>
#include "CBase.h"
#include "Log_Util.h"

std::chrono::time_point<std::chrono::steady_clock>
    g_mapFrameTime[NUM_OF_FRAME_CONTROLLER];

/**
 * @brief 프레임 제어용 기준 시간을 현재 시각으로 초기화한다.
 */
void Init_FPS_Ctrl() {
  auto now = std::chrono::steady_clock::now();
  for (int i = 0; i < NUM_OF_FRAME_CONTROLLER; i++) {
    FPS_Idx_T idx = static_cast<FPS_Idx_T>(i);
    g_mapFrameTime[idx] = now;
  }
  return ;
}
/**
 * @brief 지정한 프레임 제어 슬롯의 목표 주기를 맞춘다.
 *
 * 이전 프레임 시각과 현재 시각을 비교하여 남은 시간만큼 sleep 하고,
 * 실제 경과 시간으로부터 FPS 값을 계산한다.
 *
 * @param[in]  Idx 제어 대상 프레임 슬롯 인덱스
 * @param[in]  dbInterval_SEC 목표 프레임 간격(초)
 * @param[out] dbTimeDiff_SEC 실제 경과 시간(초)
 * @return 실제 계산된 FPS
 */
double FPS_Ctrl(const FPS_Idx_T &Idx, const double dbInterval_SEC, 
                double &dbTimeDiff_SEC)
{
  // Idx, dbInterval 입력 오류 방지
  if ((Idx < 0) || (Idx >= NUM_OF_FRAME_CONTROLLER) ||
      (dbInterval_SEC <= 0.0)) {
    dbTimeDiff_SEC = 0.0;
    return 0.0;
  }

  auto now = std::chrono::steady_clock::now();

  // 이전 프레임 시작 시간과 비교해서 남은 시간만큼 sleep
  std::chrono::duration<double> Frame_diff_SEC = now - g_mapFrameTime[Idx];
  if (Frame_diff_SEC.count() < dbInterval_SEC) {
    usleep( (dbInterval_SEC - Frame_diff_SEC.count()) * 1000000 );
  }
  auto after_sleep = std::chrono::steady_clock::now();

  // 실제 경과 시간 측정
  std::chrono::duration<double> Actual_diff_SEC = after_sleep - g_mapFrameTime[Idx];
  dbTimeDiff_SEC = Actual_diff_SEC.count();
  g_mapFrameTime[Idx] = after_sleep;

  if (dbTimeDiff_SEC <= 0.0) return 0.0;
  return (1.0/dbTimeDiff_SEC);
}

/**
 * @brief CBase 객체를 생성하고 기본 로그를 남긴다.
 */
CBase::CBase() {
  LIG("Base Construct");
}
/**
 * @brief 실행 스레드가 남아 있으면 자동으로 정리한 뒤 소멸한다.
 */
CBase::~CBase() {
  if (m_thrMain != nullptr) {
    LIY("Warning: Stop() was not called before destruction. CBase::Stop() is called automatically");
    Stop();
  }
  LIG("Base Destruct");
}


/**
 * @brief 프로시저 맵을 바탕으로 실행 스레드를 시작한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @param[in]     mapProc 전/중/후처리 콜백 맵
 * @return 성공 시 0, 실패 시 1
 */
int CBase::Start(entt::registry& ECS,Proc_Map_T& mapProc) {
  if (Get_bLoopTrigger() == true) {
    LIR("Error: class instance thread is already Start");
    return 1;
  }
  if (mapProc.find(PRE_LOOP_PROC) != mapProc.end()) {
    m_procPreLoop = mapProc[PRE_LOOP_PROC];
  } else {
    m_procPreLoop = nullptr;
  }
  if (mapProc.find(POST_LOOP_PROC) != mapProc.end()) {
    m_procPostLoop = mapProc[POST_LOOP_PROC];
  }
   else {
    m_procPostLoop = nullptr;
  }

  if (mapProc.find(LOOP_PROC) != mapProc.end()) {
    m_procLoop = mapProc[LOOP_PROC];
  } else{
    LFR("Fatal : Loop procedure is mandatory");
    return 1;
  }
  Set_bLoopTrigger(true);
  if (m_thrMain == nullptr) {
    m_thrMain = std::make_shared<std::thread>(&CBase::OnExecute,this,
                                           std::ref(ECS));
  }
  else{
    LIR("Error: class instance thread variable is not null");
    Set_bLoopTrigger(false);
    return 1;
  }

  return 0;
}

/**
 * @brief 등록된 전처리, 메인 루프, 후처리 콜백을 차례대로 실행한다.
 *
 * @param[in,out] ECS ECS 레지스트리
 * @return 항상 0
 */
int CBase::OnExecute(entt::registry& ECS) {
  // Pre Loop Procedure Call
  if (m_procPreLoop != nullptr)
    m_procPreLoop(ECS);

  while( Get_bLoopTrigger()) {
    if ( m_procLoop(ECS) ) {
      // Stop the loop
      Set_bLoopTrigger(false);
    }
  } // end while( Get_bLoopTrigger())

  // Post Loop Procedure Call
  if (m_procPostLoop != nullptr)
    m_procPostLoop(ECS);

  return 0;
}

/**
 * @brief 루프 중지 플래그를 내리고 실행 스레드를 join 한다.
 *
 * @return 항상 0
 */
int CBase::Stop() {
  if (Get_bLoopTrigger() == false) {
    LIR("class instance is already Stop");
    if (m_thrMain != nullptr && m_thrMain->joinable()) {
      m_thrMain->join();
      m_thrMain = nullptr;
    }
    return 0;
  }
  Set_bLoopTrigger(false);
  if (m_thrMain != nullptr && m_thrMain->joinable()) {
    m_thrMain->join();
  }
  m_thrMain=nullptr;

  return 0;
}

/**
 * @brief 실행 스레드가 종료될 때까지 대기한다.
 *
 * @return 항상 0
 */
int CBase::Wait() {
  if (m_thrMain != nullptr && m_thrMain->joinable()) {
    m_thrMain->join();
  }
  m_thrMain = nullptr;
  return 0;
}
