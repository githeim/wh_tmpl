#pragma once
#include "entt/entt.hpp"

namespace Scene::GameOver {
  void OnEnter(entt::registry &ECS, float dt);
  void OnExit(entt::registry &ECS, float dt);
  void OnRender(entt::registry &ECS, float dt);
}
