# FieldNav

> Open-source modular library for sensor fusion and localization in outdoor mobile robotics.

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)
![Status](https://img.shields.io/badge/status-Phase%201%20Bootstrap-orange.svg)

FieldNav is a C++ stack for the localization and navigation of mobile robots in
unstructured outdoor environments (fields, vineyards, orchards). It covers the
full vertical from STM32 firmware to ROS 2 nodes.

---

## 🚧 Current status

**Phase 1 — Bootstrap** (June 2026). The project foundation is in place:
build system, CI/CD, C matrix library for educational purposes, and skeleton
of the C++ core library.

See [docs/roadmap.md](docs/roadmap.md) for the full plan.

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Layer 6 - Visualization, Cloud & Paper                 │
├─────────────────────────────────────────────────────────┤
│  Layer 5 - ROS 2 Integration                            │
│  fieldnav_msgs  •  fieldnav_localization_node           │
├─────────────────────────────────────────────────────────┤
│  Layer 4 - Applications & Pipelines                     │
├─────────────────────────────────────────────────────────┤
│  Layer 3 - Algorithms                                   │
│  fieldnav_filters : EKF, UKF, Particle Filter           │
├─────────────────────────────────────────────────────────┤
│  Layer 2 - C++ Core Library                             │
│  fieldnav_core : Pose, Transform, Quaternion, Time      │
├─────────────────────────────────────────────────────────┤
│  Layer 1 - Embedded firmware (STM32 + FreeRTOS)         │
└─────────────────────────────────────────────────────────┘
```

Detailed architecture: [docs/architecture.md](docs/architecture.md)

---

## 🚀 Quick start

### Prerequisites

- Linux (Ubuntu 22.04+) or WSL2 on Windows
- CMake ≥ 3.20
- C++17 compiler (gcc 11+ or clang 14+)
- Ninja (recommended)

```bash
sudo apt-get install build-essential cmake ninja-build
```

### Build and test

```bash
git clone https://github.com/YOUR-USER/fieldnav.git
cd fieldnav

# Configure (Debug build)
cmake --preset debug

# Build
cmake --build build/debug

# Run all tests
ctest --test-dir build/debug --output-on-failure
```

### Build with sanitizers (recommended during development)

```bash
cmake --preset asan
cmake --build build/asan
ctest --test-dir build/asan --output-on-failure
```

### Available CMake presets

| Preset    | Compiler | Build type | Sanitizers     | Purpose                |
|-----------|----------|------------|----------------|------------------------|
| `debug`   | gcc      | Debug      | none           | Day-to-day debugging   |
| `release` | gcc      | Release    | none           | Performance testing    |
| `asan`    | clang    | Debug      | ASan + UBSan   | Catching memory bugs   |

---

## 📁 Project structure

```
fieldnav/
├── c_legacy/        Phase 1: dynamic matrix library in C (educational)
├── core/            Phase 2: C++17 core types (Pose, Quaternion, ...)
├── filters/         Phase 3: EKF, UKF, Particle Filter (stub)
├── ros2_ws/         Phase 4: ROS 2 nodes (stub)
├── embedded/        Phase 5: STM32 firmware (stub)
├── cmake/           CMake helper modules
├── docs/            Architecture, ADRs, tutorials
└── .github/         CI/CD workflows
```

---

## 🗺️ Roadmap

- [x] **v0.1** Phase 1 — Bootstrap (June 2026)
- [ ] **v0.2** Phase 2 — C++ core types (Sep 2026)
- [ ] **v0.3** Phase 3 — Filters (Jan 2027)
- [ ] **v0.4** Phase 4 — ROS 2 integration (Apr 2027)
- [ ] **v0.5** Phase 5 — Embedded firmware (May 2027)
- [ ] **v1.0** Phase 6 — M2 paper release (Jun 2027)

---

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for code style, branching strategy,
and PR process.

---

## 📜 License

Apache License 2.0 — see [LICENSE](LICENSE).
