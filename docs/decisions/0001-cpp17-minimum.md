# ADR-0001 — C++17 as minimum standard

## Status

Accepted

## Context

C++ has evolved rapidly: C++11/14/17/20/23. We need to pick a baseline:

- **C++17**: stable, fully supported on all modern compilers (gcc 7+, clang 5+).
  Includes structured bindings, `std::optional`, `std::variant`, parallel
  algorithms, etc.
- **C++20**: adds concepts, ranges, coroutines — but compiler support is still
  uneven (especially on embedded toolchains).
- **C++23**: too recent for stable cross-platform use.

## Decision

We target **C++17 as the minimum standard** for all C++ code in FieldNav.
Specific modules may opt-in to C++20 features (e.g. `<concepts>` in filters)
behind a feature detection macro.

## Consequences

- ✅ All modern compilers we care about (gcc 11+, clang 14+) support C++17 fully.
- ✅ Embedded toolchains (arm-none-eabi-gcc 12+) support C++17.
- ✅ ROS 2 Humble/Jazzy require C++17.
- ⚠️ We forgo concepts in Phase 1-2 (would have been nice for the EKF template
   constraints in Phase 3). Mitigation: use `static_assert` + `enable_if`.
