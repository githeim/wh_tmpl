#pragma once

#include <functional>
#include <map>
#include <queue>

#include <entt/entt.hpp>

/**
 * @brief 애플리케이션 내의 씬(Scene) 식별자를 정의하는 열거형이다.
 *
 * 새 씬을 추가할 때:
 *  1. 이 열거형에 항목 추가
 *  2. ToString() 에 case 추가
 *  3. scenes/Scene_Xxx.h/.cpp 생성
 *  4. SceneMap.cpp 의 GetSceneMap() 에 항목 등록
 */
enum class SceneId {
  Title,      ///< 타이틀 화면
  MainMenu,   ///< 메인 메뉴 화면
  Scenarios,  ///< 시나리오 선택 화면
  Option,     ///< 옵션 설정 화면
  Play,       ///< 플레이(게임) 화면
  End,        ///< 종료 화면
};

/**
 * @brief 씬 식별자를 화면 표시용 문자열로 변환한다.
 */
inline const char *ToString(const SceneId scene) {
  switch (scene) {
    case SceneId::Title:     return "Title";
    case SceneId::MainMenu:  return "Main Menu";
    case SceneId::Scenarios: return "Scenarios";
    case SceneId::Option:    return "Option";
    case SceneId::Play:      return "Play";
    case SceneId::End:       return "End";
  }
  return "Unknown";
}

/**
 * @brief 씬 훅 함수 타입. EnTT 레지스트리를 받아 동작을 수행한다.
 */
using SceneHook = std::function<void(entt::registry &, float)>;

/**
 * @brief 씬 로딩/정리 태스크 타입.
 *
 * OnEnter/OnExit 에서 태스크 큐에 등록하면 CApp 메인 루프가
 * 매 프레임 1개씩 메인 스레드에서 실행한다.
 * SDL 렌더러 조작(CreateTexture 등)이 가능하고, 태스크 사이마다
 * RenderTransitionScreen 이 호출되어 로딩 화면이 갱신된다.
 */
using LoadTask      = std::function<void(entt::registry &)>;
using LoadTaskQueue = std::queue<LoadTask>;

/**
 * @brief 씬 하나에 연결된 생명주기 훅 집합이다.
 *
 * onEnter/onExit: 태스크 큐에 LoadTask 를 등록하는 역할만 수행한다.
 *   실제 작업은 enterTasks/exitTasks 큐에서 메인 스레드가 1개씩 꺼내 실행한다.
 * onUpdate 는 선택적이며 nullptr 이면 해당 씬에서 게임 로직 갱신이 수행되지 않는다.
 */
struct SceneDefinition {
  SceneHook      onEnter;    ///< 태스크 큐 등록 (실제 작업은 ECS.ctx()<LoadTaskQueue> 에서)
  SceneHook      onExit;     ///< 태스크 큐 등록 (실제 작업은 ECS.ctx()<LoadTaskQueue> 에서)
  SceneHook      onRender;   ///< 매 프레임 렌더링 호출 (필수)
  SceneHook      onUpdate;   ///< 매 프레임 게임 로직 갱신. nullptr 이면 스킵.
};

/**
 * @brief 모든 씬의 생명주기 훅을 SceneDefinition 맵으로 구성하여 반환한다.
 *
 * 새 씬을 추가할 때 SceneMap.cpp 에 항목을 등록한다.
 *
 * @return SceneId → SceneDefinition 맵
 */
std::map<SceneId, SceneDefinition> GetSceneMap();
