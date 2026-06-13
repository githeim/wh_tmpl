#pragma once

#include <functional>
#include <map>
#include <queue>

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

/**
 * @brief 씬 로딩/정리 태스크 타입.
 *
 * OnEnter/OnExit 에서 ECS.ctx().emplace<LoadTaskQueue>() 로 등록하면
 * CApp 메인 루프가 매 프레임 1개씩 메인 스레드에서 실행한다.
 * SDL 렌더러 조작(CreateTexture 등)이 가능하고, 태스크 사이마다
 * RenderTransitionScreen 이 호출되어 로딩 화면이 갱신된다.
 */
using LoadTask      = std::function<void(entt::registry &)>;
using LoadTaskQueue = std::queue<LoadTask>;

/**
 * @brief 씬 하나에 연결된 생명주기 훅 집합이다.
 *
 * onEnter/onExit: ECS.ctx() 에 LoadTaskQueue 를 등록하는 역할만 수행한다.
 *   실제 작업은 CApp UpdateTransition 이 메인 스레드에서 1개씩 꺼내 실행한다.
 * onUpdate 는 nullptr 이면 스킵.
 */
struct SceneDefinition {
  SceneHook onEnter;   ///< 태스크 큐 등록 (실제 작업은 ECS.ctx()<LoadTaskQueue> 에서)
  SceneHook onExit;    ///< 태스크 큐 등록 (실제 작업은 ECS.ctx()<LoadTaskQueue> 에서)
  SceneHook onRender;
  SceneHook onUpdate;  ///< nullptr 이면 스킵
};

std::map<SceneId, SceneDefinition> GetSceneMap();
