# Architecture

## Layered view

FieldNav is organized in six layers, each independently usable:

```
Layer 6 — Visualization, cloud, paper
Layer 5 — ROS 2 integration (rclcpp lifecycle nodes)
Layer 4 — Applications and pipelines
Layer 3 — Algorithms (filters, planning)
Layer 2 — C++ core types
Layer 1 — Embedded firmware (STM32)
```

## Module dependency graph

```
c_legacy ──┐
           │ (educational only, not used by upper layers)
           ▼
        (none)

core ◄── filters ◄── apps ◄── ros2_localization_node
                        ▲
                        └── embedded (via serialized messages)
```

## Design choices

See ADRs in `docs/decisions/`:
- ADR-0001 — C++17 as minimum standard
- ADR-0002 — Eigen for linear algebra (Phase 2 onwards)
- ADR-0003 — Apache-2.0 license

## Public API conventions

- Namespace: `fieldnav::<module>` (`fieldnav::core`, `fieldnav::filters`, ...)
- Include path: `<fieldnav/<module>/<header>.hpp>`
- Naming: classes `CamelCase`, functions `camelBack`, variables `lower_case`,
  private members suffixed with `_`.
- All public functions are `[[nodiscard]]` when they return a value.
- Functions that don't throw are marked `noexcept`.
