#pragma once

#include "entt/entt.hpp"
#include "SceneDef.h"

/**
 * @brief 화면 전체를 덮는 ImGui 창을 시작한다.
 *
 * @param[in] windowName ImGui 창 식별 이름
 */
void BeginFullscreenUi(const char *windowName);

/**
 * @brief 전체 화면 ImGui 창을 종료하고 관련 스타일 스택을 복원한다.
 */
void EndFullscreenUi();

/**
 * @brief 각 씬 공통 헤더와 라이프사이클 정보를 렌더링한다.
 *
 * @param[in] ECS      ECS 레지스트리
 * @param[in] scene    현재 씬
 * @param[in] heading  씬 제목
 * @param[in] subtitle 씬 설명 문구
 */
void RenderSceneChrome(entt::registry &ECS, SceneId scene,
                       const char *heading, const char *subtitle, float dt = 0.0f);

/**
 * @brief 공통 씬 진입 처리. enter 횟수와 라이프사이클 메시지를 갱신한다.
 *
 * @param[in,out] ECS   ECS 레지스트리
 * @param[in]     scene 진입한 씬
 * @param[in] dt  프레임 경과 시간 (초, 미사용, 확장 예약)
 */
void OnGenericSceneEnter(entt::registry &ECS, SceneId scene, float dt = 0.0f);

/**
 * @brief 공통 씬 이탈 처리. exit 횟수와 라이프사이클 메시지를 갱신한다.
 *
 * @param[in,out] ECS   ECS 레지스트리
 * @param[in]     scene 이탈한 씬
 * @param[in] dt  프레임 경과 시간 (초, 미사용, 확장 예약)
 */
void OnGenericSceneExit(entt::registry &ECS, SceneId scene, float dt = 0.0f);
