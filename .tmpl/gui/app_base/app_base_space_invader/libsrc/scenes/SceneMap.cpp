#include "SceneMap.h"

#include "Scene_Title.h"
#include "Scene_Game.h"
#include "Scene_GameOver.h"

std::map<SceneId, SceneDefinition> GetSceneMap() {
  return {
      {SceneId::Title,
       {Scene::Title::OnEnter, Scene::Title::OnExit, Scene::Title::OnRender, nullptr}},
      {SceneId::Game,
       {Scene::Game::OnEnter, Scene::Game::OnExit, Scene::Game::OnRender, Scene::Game::OnUpdate}},
      {SceneId::GameOver,
       {Scene::GameOver::OnEnter, Scene::GameOver::OnExit, Scene::GameOver::OnRender, nullptr}},
  };
}
