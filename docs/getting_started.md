# Getting started

## 1. Install prerequisites (Ubuntu / WSL2)

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build clang clang-tidy git
```

## 2. Clone

```bash
git clone https://github.com/YOUR-USER/fieldnav.git
cd fieldnav
```

## 3. Configure with a preset

```bash
cmake --preset debug
```

This creates `build/debug/` with a Ninja generator.

## 4. Build

```bash
cmake --build build/debug
```

## 5. Run the tests

```bash
ctest --test-dir build/debug --output-on-failure
```

## 6. Try the sanitizers

```bash
cmake --preset asan
cmake --build build/asan
ctest --test-dir build/asan --output-on-failure
```

To see ASan in action, edit `c_legacy/src/matrix.c` and introduce a
deliberate out-of-bounds write, rebuild, and watch the test fail with a
detailed stack trace.

## 7. Format the code before committing

```bash
find c_legacy core -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \
    | xargs clang-format -i
```

## Next steps

- Read `docs/architecture.md`.
- Pick an item from the Phase 1 checklist in your project documents.
- Open a feature branch: `git checkout -b feature/add-transform-class`.
- Make changes, run tests locally, push, open a PR.
