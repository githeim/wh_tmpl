#pragma once

#include <string>

/**
 * @brief 앱 전역 누적 게임 상태. 씬 간에 유지되는 데이터를 담는다.
 *
 * 이 구조체는 게임 시작~종료까지 계속 누적된다.
 * entt ctx에 등록해 어느 씬에서든 접근 가능하다.
 *
 * 사용 예:
 * - emplace:  ECS.ctx().emplace<GameState>();
 * - 읽기:     auto& gs = ECS.ctx().get<GameState>();
 * - 수정:     ECS.ctx().get<GameState>().gold += 100;
 */
struct GameState {
  std::string playerName  = "Player";  ///< 플레이어 이름
  int         level       = 1;         ///< 현재 레벨 (씬 진행도 기준)
  int         score       = 0;         ///< 게임 전체 누적 점수
  int         gold        = 0;         ///< 보유 골드 (재화 예시)
  float       playTimeSec = 0.0f;      ///< 총 플레이 시간(초)
  int         playerLives = 3;         ///< 남은 목숨
};
