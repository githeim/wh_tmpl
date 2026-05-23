#include "SceneMap.h"

#include "Scene_Title.h"
#include "Scene_MainMenu.h"
#include "Scene_Scenarios.h"
#include "Scene_Option.h"
#include "Scene_Play.h"
#include "Scene_End.h"

/**
 * @brief 모든 씬의 생명주기 훅을 구성한 맵을 반환한다.
 *
 * 새 씬을 추가할 때 절차:
 *  1. SceneDef.h 의 SceneId enum 에 항목 추가
 *  2. libsrc/scenes/Scene_Xxx.h/.cpp 생성 (네임스페이스 Scene::Xxx)
 *  3. 이 파일에 #include 추가 후 맵 항목 등록
 *  4. onUpdate 불필요한 씬은 마지막 인자를 nullptr 로 전달
 *
 * @return SceneId → SceneDefinition 맵
 */
std::map<SceneId, SceneDefinition> GetSceneMap() {
  return {
      {SceneId::Title,
       {Scene::Title::OnEnter, Scene::Title::OnExit, Scene::Title::OnRender, nullptr}},
      {SceneId::MainMenu,
       {Scene::MainMenu::OnEnter, Scene::MainMenu::OnExit, Scene::MainMenu::OnRender, nullptr}},
      {SceneId::Scenarios,
       {Scene::Scenarios::OnEnter, Scene::Scenarios::OnExit, Scene::Scenarios::OnRender, nullptr}},
      {SceneId::Option,
       {Scene::Option::OnEnter, Scene::Option::OnExit, Scene::Option::OnRender, nullptr}},
      {SceneId::Play,
       {Scene::Play::OnEnter, Scene::Play::OnExit, Scene::Play::OnRender, Scene::Play::OnUpdate}},
      {SceneId::End,
       {Scene::End::OnEnter, Scene::End::OnExit, Scene::End::OnRender, nullptr}},
  };
}
