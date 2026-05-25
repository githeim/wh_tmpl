#pragma once
#include "entt/entt.hpp"

namespace Scene::Game {
  void OnEnter(entt::registry &ECS, float dt);
  void OnExit(entt::registry &ECS, float dt);
  void OnRender(entt::registry &ECS, float dt);
  void OnUpdate(entt::registry &ECS, float dt);
}
