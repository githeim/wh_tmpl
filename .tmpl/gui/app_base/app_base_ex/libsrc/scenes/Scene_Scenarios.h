#pragma once
#include "entt/entt.hpp"

/**
 * @brief Scenarios 씬 구현 네임스페이스.
 *
 * 시나리오 1~4 중 하나를 선택하여 Play 씬으로 전이한다.
 * 선택값은 PlayArgs 로 SceneTransitionRequest에 담겨 전달된다.
 */
namespace Scene::Scenarios {

  /**
   * @brief Scenarios 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnEnter(entt::registry &ECS, float dt);

  /**
   * @brief Scenarios 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnExit(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 Scenarios 씬 UI를 렌더링한다.
   *
   * 시나리오 선택 버튼 4개와 Main Menu 복귀 버튼을 제공한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnRender(entt::registry &ECS, float dt);
}
