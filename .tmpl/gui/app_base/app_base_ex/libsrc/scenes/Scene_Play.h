#pragma once
#include "entt/entt.hpp"

/**
 * @brief Play 씬 구현 네임스페이스.
 *
 * 상단 1/3은 ImGui HUD, 하단 2/3는 SDL Renderer 게임 드로잉 영역으로 구성된다.
 * Play 씬 상태는 ECS 컴포넌트와 GameState ctx를 통해 관리한다.
 *
 * OnUpdate 훅이 등록된 유일한 씬으로, 매 프레임 삼각형 회전/원 맥박/점수 갱신을 수행한다.
 */
namespace Scene::Play {

  /**
   * @brief Play 씬 진입 시 호출된다.
   *
   * 공통 진입 카운터를 갱신하고 Play 씬 전용 entity 및 CommandBuffer를 초기화한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnEnter(entt::registry &ECS, float dt);

  /**
   * @brief Play 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnExit(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 Play 씬을 렌더링한다.
   *
   * SDL Renderer로 게임 드로잉(삼각형/원/사각형)을 먼저 수행한 뒤,
   * 상단 영역에 ImGui HUD(점수, 선택 시나리오, Game Over 버튼)를 그린다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnRender(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 Play 씬 게임 로직을 갱신한다.
   *
   * 삼각형 회전 각도 증가, 원 반지름 맥박 변동, 60프레임마다 점수 +1을 수행한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnUpdate(entt::registry &ECS, float dt);
}
