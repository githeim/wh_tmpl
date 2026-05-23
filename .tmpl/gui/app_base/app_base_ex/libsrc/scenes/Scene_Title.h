#pragma once
#include "entt/entt.hpp"

/**
 * @brief Title 씬 구현 네임스페이스.
 *
 * 앱 최초 진입 씬으로, Preparing 없이 즉시 표시된다.
 * Main Menu 씬으로의 전이 버튼을 제공한다.
 */
namespace Scene::Title {

  /**
   * @brief Title 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnEnter(entt::registry &ECS, float dt);

  /**
   * @brief Title 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnExit(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 Title 씬 UI를 렌더링한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnRender(entt::registry &ECS, float dt);
}
