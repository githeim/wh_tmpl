#ifndef _LOG_UTIL_H_
#define _LOG_UTIL_H_
#include <glog/logging.h>

/// @defgroup AnsiColors ANSI 색상 코드
/// 터미널 출력에 색을 입히기 위한 ANSI escape 코드 매크로.
/// @{
#define ANSI_YELLOW  "\033[1;33m" ///< 밝은 노란색
#define ANSI_RED     "\033[1;31m" ///< 밝은 빨간색
#define ANSI_GREEN   "\033[1;32m" ///< 밝은 초록색
#define ANSI_CYAN    "\033[1;36m" ///< 밝은 청록색
#define ANSI_RESET   "\033[m"     ///< 색상 초기화
/// @}

/// @defgroup LogBase 기본 로그 매크로
/// 함수명을 자동으로 포함하는 glog 래퍼 매크로.
/// @{
#define LI(Val) LOG(INFO)   <<" "<<__FUNCTION__<<"() "<<Val; ///< INFO 로그
#define LW(Val) LOG(WARNING)<<" "<<__FUNCTION__<<"() "<<Val; ///< WARNING 로그
#define LE(Val) LOG(ERROR)  <<" "<<__FUNCTION__<<"() "<<Val; ///< ERROR 로그
#define LF(Val) LOG(FATAL)  <<" "<<__FUNCTION__<<"() "<<Val; ///< FATAL 로그
/// @}

/// @defgroup LogInfo 색상 INFO 로그 매크로
/// INFO 레벨에 ANSI 색상을 입히는 단축 매크로.
/// @{
#define LIY(Val) LI(ANSI_YELLOW<< Val << ANSI_RESET) ///< INFO 노란색
#define LIR(Val) LI(ANSI_RED << Val << ANSI_RESET)   ///< INFO 빨간색
#define LIG(Val) LI(ANSI_GREEN << Val << ANSI_RESET) ///< INFO 초록색
#define LIB(Val) LI(ANSI_CYAN << Val << ANSI_RESET)  ///< INFO 청록색
/// @}

/// @defgroup LogWarn 색상 WARNING 로그 매크로
/// WARNING 레벨에 ANSI 색상을 입히는 단축 매크로.
/// @{
#define LWY(Val) LW(ANSI_YELLOW << Val << ANSI_RESET) ///< WARNING 노란색
#define LWR(Val) LW(ANSI_RED << Val << ANSI_RESET)    ///< WARNING 빨간색
#define LWG(Val) LW(ANSI_GREEN << Val << ANSI_RESET)  ///< WARNING 초록색
#define LWB(Val) LW(ANSI_CYAN << Val << ANSI_RESET)   ///< WARNING 청록색
/// @}

/// @defgroup LogError 색상 ERROR 로그 매크로
/// ERROR 레벨에 ANSI 색상을 입히는 단축 매크로.
/// @{
#define LEY(Val) LE(ANSI_YELLOW << Val << ANSI_RESET) ///< ERROR 노란색
#define LER(Val) LE(ANSI_RED << Val << ANSI_RESET)    ///< ERROR 빨간색
#define LEG(Val) LE(ANSI_GREEN << Val << ANSI_RESET)  ///< ERROR 초록색
#define LEB(Val) LE(ANSI_CYAN << Val << ANSI_RESET)   ///< ERROR 청록색
/// @}

/// @defgroup LogFatal 색상 FATAL 로그 매크로
/// FATAL 레벨에 ANSI 색상을 입히는 단축 매크로.
/// @{
#define LFY(Val) LF(ANSI_YELLOW << Val << ANSI_RESET) ///< FATAL 노란색
#define LFR(Val) LF(ANSI_RED << Val << ANSI_RESET)    ///< FATAL 빨간색
#define LFG(Val) LF(ANSI_GREEN << Val << ANSI_RESET)  ///< FATAL 초록색
#define LFB(Val) LF(ANSI_CYAN << Val << ANSI_RESET)   ///< FATAL 청록색
/// @}

#endif /* ifndef _LOG_UTIL_H_ */
