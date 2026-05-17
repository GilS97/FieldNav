# ADR-0002 — Eigen for linear algebra

## Status

Accepted (effective from Phase 2)

## Context

FieldNav needs intensive matrix operations for its filters (EKF, UKF, PF).
Options considered:

1. **Custom library** (max control, learning value, performance unknown).
2. **Eigen 3** (mature, performant, de-facto standard in robotics).
3. **Armadillo** (nice syntax, less common in robotics).
4. **Blaze**, **xtensor**, others.

## Decision

Use **Eigen 3.4+** as the linear algebra backend, starting from Phase 2.
Phase 1's `c_legacy/` keeps a hand-rolled C implementation purely for educational
purposes.

## Consequences

- ✅ Optimal performance (SIMD vectorized).
- ✅ Native interop with ROS 2 (`tf2_eigen`) and most robotics libraries.
- ✅ Header-only — easy to vendor or fetch.
- ⚠️ Templates are heavy at compile time. Mitigation: explicit instantiation
   in `.cpp` files for common types.
- ⚠️ Error messages on misuse can be cryptic. Mitigation: type aliases in
   `fieldnav::core` to hide raw Eigen types from public API.
