# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-06-XX

### Added

- Initial project structure (`c_legacy/`, `core/`, `filters/`, …).
- C matrix library `c_legacy/` with `Matrix`, `matrix_create`, `matrix_destroy`,
  `matrix_get`, `matrix_set`, `matrix_mul`, `matrix_transpose`.
- C++ core skeleton `core/`: `fieldnav::core::Quaternion`,
  `fieldnav::core::Pose3D`, version header, time aliases.
- CMake build system with presets (debug, release, asan).
- GitHub Actions CI: build matrix (gcc + clang, Debug + Release) and
  sanitizers (ASan + UBSan).
- Clang-format and clang-tidy configurations.
- Apache-2.0 license, README, CONTRIBUTING, CHANGELOG.

[Unreleased]: https://github.com/YOUR-USER/fieldnav/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/YOUR-USER/fieldnav/releases/tag/v0.1.0
