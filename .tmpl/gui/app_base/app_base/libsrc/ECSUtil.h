#pragma once

#include <functional>
#include <vector>
#include <variant>

#include <entt/entt.hpp>

// =============================================================================
// 공통 커맨드 구조체
// =============================================================================

/**
 * @brief entity 를 Flush 시점에 destroy 하는 커맨드이다.
 *
 * Flush 전에 이미 invalid 한 entity 는 안전하게 스킵한다.
 */
struct CmdDestroyEntity {
  entt::entity e;
};

/**
 * @brief 새 entity 를 생성하고 setup 람다에서 컴포넌트를 부착하는 커맨드이다.
 *
 * setup 은 임의 컴포넌트 조합을 부착해야 하므로 std::function 을 사용한다.
 * 이 커맨드만 힙 할당이 발생할 수 있다.
 */
struct CmdCreateEntity {
  std::function<void(entt::registry &, entt::entity)> setup;
};

/**
 * @brief ctx 수정 등 entity 없는 임의 로직을 Flush 시 실행하는 커맨드이다.
 *
 * 컴포넌트 타입이 고정되지 않는 경우(ctx 수정 등)에 사용한다.
 * std::function 을 포함하므로 힙 할당이 발생할 수 있다.
 */
struct CmdSubmit {
  std::function<void(entt::registry &)> fn;
};

// Apply — 공통 커맨드 실행 (CommandBuffer::Flush 에서 ADL 로 호출)
inline void Apply(entt::registry &reg, const CmdDestroyEntity &c) {
  if (reg.valid(c.e)) reg.destroy(c.e);
}
inline void Apply(entt::registry &reg, const CmdCreateEntity &c) {
  c.setup(reg, reg.create());
}
inline void Apply(entt::registry &reg, const CmdSubmit &c) {
  c.fn(reg);
}

// =============================================================================
// 씬별 컴포넌트 커맨드 패턴
//
// Update / Add / Remove 는 컴포넌트 타입이 컴파일 타임에 고정되어야 하므로
// 씬 파일 anonymous namespace 에서 아래 패턴으로 정의한다.
//
// ── Update (컴포넌트 값 갱신) ──────────────────────────────────────────────
//   struct CmdUpdateFoo { entt::entity e; Foo value; };
//   inline void Apply(entt::registry &reg, const CmdUpdateFoo &c) {
//     if (reg.valid(c.e) && reg.all_of<Foo>(c.e))
//       reg.get<Foo>(c.e) = c.value;
//   }
//
// ── Add (기존 entity 에 컴포넌트 추가) ────────────────────────────────────
//   struct CmdAddFoo { entt::entity e; Foo value; };
//   inline void Apply(entt::registry &reg, const CmdAddFoo &c) {
//     if (reg.valid(c.e)) reg.emplace_or_replace<Foo>(c.e, c.value);
//   }
//
// ── Remove (기존 entity 에서 컴포넌트 제거) ───────────────────────────────
//   struct CmdRemoveFoo { entt::entity e; };
//   inline void Apply(entt::registry &reg, const CmdRemoveFoo &c) {
//     if (reg.valid(c.e) && reg.all_of<Foo>(c.e)) reg.erase<Foo>(c.e);
//   }
//
// 정의 후 씬의 using XxxCmd = std::variant<..., CmdUpdateFoo, CmdAddFoo, CmdRemoveFoo>
// 에 추가하면 Flush 시 자동으로 dispatch 된다.
// =============================================================================

// =============================================================================
// CommandBuffer<Variant>
// =============================================================================

/**
 * @brief ECS 업데이트 턴 내 변경 사항을 모아 Flush 시 일괄 반영하는 커맨드 버퍼이다.
 *
 * Variant 에 등록된 커맨드 타입만 수용한다. 컴포넌트 값을 구조체로 직접 저장하므로
 * std::function 람다 방식 대비 힙 할당이 없다 (CmdCreateEntity/CmdSubmit 제외).
 *
 * ### 씬별 사용법
 * 1. 씬 파일 anonymous namespace 에서 컴포넌트별 Cmd 구조체와 Apply 함수 정의
 * 2. `using XxxCmd = std::variant<CmdDestroyEntity, CmdCreateEntity, CmdSubmit, CmdUpdateXxx, ...>`
 * 3. `using XxxCmdBuffer = CommandBuffer<XxxCmd>`
 * 4. OnEnter: `ECS.ctx().emplace<XxxCmdBuffer>()`
 * 5. System: `ECS.ctx().get<XxxCmdBuffer>().Add(CmdUpdateXxx{e, newValue})`
 * 6. OnUpdate 마지막: `ECS.ctx().get<XxxCmdBuffer>().Flush(ECS)`
 * 7. OnExit: `ECS.ctx().erase<XxxCmdBuffer>()`
 *
 * ### 새 컴포넌트 추가 시
 * 1. Cmd 구조체 추가  (e.g. `struct CmdUpdateFoo { entt::entity e; Foo value; };`)
 * 2. Apply 함수 추가 (e.g. `void Apply(entt::registry &, const CmdUpdateFoo &c) {...}`)
 * 3. Variant 목록에 추가
 *
 * ### 병렬화 확장
 * System 별로 CommandBuffer 를 분리해두고 메인 스레드에서 순서대로 Flush 하면
 * race condition 없이 병렬 System 실행이 가능하다.
 *
 * @tparam Variant 이 버퍼가 수용하는 커맨드 타입들의 std::variant
 */
template<typename Variant>
struct CommandBuffer {
  /**
   * @brief 커맨드를 큐에 추가한다.
   *
   * @param cmd 추가할 커맨드 (Variant 에 등록된 타입이어야 함)
   */
  void Add(Variant cmd) {
    m_commands.push_back(std::move(cmd));
  }

  /**
   * @brief 이 턴에 쌓인 모든 커맨드를 순서대로 실행하고 큐를 비운다.
   *
   * 각 커맨드 타입에 대응하는 Apply(reg, cmd) 를 ADL 로 호출한다.
   * OnUpdate 마지막에 1회 호출한다.
   *
   * @param reg 대상 ECS 레지스트리
   */
  void Flush(entt::registry &reg) {
    for (auto &cmd : m_commands) {
      std::visit([&reg](auto &c) { Apply(reg, c); }, cmd);
    }
    m_commands.clear();
  }

  /** @brief 현재 대기 중인 커맨드 수를 반환한다. (디버그/테스트용) */
  [[nodiscard]] std::size_t Size() const { return m_commands.size(); }

private:
  std::vector<Variant> m_commands;  ///< 대기 중인 커맨드 목록
};
