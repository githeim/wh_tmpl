#pragma once

#include "entt/entt.hpp"
#include "SceneDef.h"

void BeginFullscreenUi(const char *windowName);
void EndFullscreenUi();
void OnGenericSceneEnter(entt::registry &ECS, SceneId scene, float dt = 0.0f);
void OnGenericSceneExit(entt::registry &ECS, SceneId scene, float dt = 0.0f);
