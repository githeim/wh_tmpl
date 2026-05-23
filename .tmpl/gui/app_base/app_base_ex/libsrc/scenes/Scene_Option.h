#pragma once
#include "entt/entt.hpp"

/**
 * @brief Option 씬 구현 네임스페이스.
 *
 * 옵션 화면 예제 씬이다. 실제 옵션 항목 대신
 * 씬 전이 구조 시연에 집중한다.
 */
namespace Scene::Option {

  /**
   * @brief Option 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnEnter(entt::registry &ECS, float dt);

  /**
   * @brief Option 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnExit(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 Option 씬 UI를 렌더링한다.
   *
   * Main Menu 복귀 버튼을 제공한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnRender(entt::registry &ECS, float dt);
}
