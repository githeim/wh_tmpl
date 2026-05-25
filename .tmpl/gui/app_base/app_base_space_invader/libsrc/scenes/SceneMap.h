#pragma once

#include <functional>
#include <map>

#include <entt/entt.hpp>

/**
 * @brief 애플리케이션 내의 씬(Scene) 식별자를 정의하는 열거형이다.
 */
enum class SceneId {
  Title,     ///< 타이틀 화면
  Game,      ///< 게임 화면
  GameOver,  ///< 게임 오버 화면
};

/**
 * @brief 씬 식별자를 화면 표시용 문자열로 변환한다.
 */
inline const char *ToString(const SceneId scene) {
  switch (scene) {
    case SceneId::Title:    return "Title";
    case SceneId::Game:     return "Game";
    case SceneId::GameOver: return "GameOver";
  }
  return "Unknown";
}

using SceneHook = std::function<void(entt::registry &, float)>;

struct SceneDefinition {
  SceneHook onEnter;
  SceneHook onExit;
  SceneHook onRender;
  SceneHook onUpdate;  ///< nullptr 이면 스킵
};

std::map<SceneId, SceneDefinition> GetSceneMap();
