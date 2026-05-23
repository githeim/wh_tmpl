#pragma once
#include "entt/entt.hpp"

/**
 * @brief Main Menu 씬 구현 네임스페이스.
 *
 * 시나리오 진입, 옵션, 종료 세 가지 경로를 제공한다.
 * 종료는 Popup 확인 후 AppQuitRequest를 enqueue 한다.
 */
namespace Scene::MainMenu {

  /**
   * @brief Main Menu 씬 진입 시 호출된다. 공통 진입 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnEnter(entt::registry &ECS, float dt);

  /**
   * @brief Main Menu 씬 이탈 시 호출된다. 공통 이탈 카운터를 갱신한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnExit(entt::registry &ECS, float dt);

  /**
   * @brief 매 프레임 Main Menu 씬 UI를 렌더링한다.
   *
   * 시나리오/옵션/종료 버튼과 종료 확인 팝업을 포함한다.
   *
   * @param[in,out] ECS ECS 레지스트리
   */
  void OnRender(entt::registry &ECS, float dt);
}
