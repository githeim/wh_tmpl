#pragma once
#include "entt/entt.hpp"

/**
 * @brief End 씬 구현 네임스페이스.
 *
 * 게임 종료 후 표시되는 화면이다.
 * Main Menu로 복귀하는 버튼을 제공한다.
 */
namespace Scene::End {

  /**
   * @brief End 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnEnter(entt::registry &ECS, float dt);

  /**
   * @brief End 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnExit(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 End 씬 UI를 렌더링한다.
   *
   * Main Menu 복귀 버튼을 제공한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnRender(entt::registry &ECS, float dt);
}
