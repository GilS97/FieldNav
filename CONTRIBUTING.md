# Contributing to FieldNav

Thanks for your interest. Even though this is a personal project for
educational purposes, it follows professional standards.

## Code style

- C/C++ code is auto-formatted with `clang-format` (style file at the root).
- Configure your editor to format on save.
- Run `find . -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' | xargs clang-format -i`
  before pushing.

## Branching

- `main`: stable. All merges go through PR with CI green.
- `feature/<short-name>`: feature branches, max 2 weeks.
- `fix/<short-name>`: bug fixes.
- Branches are deleted after merge.

## Commit messages

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
feat(core): add Pose3D compose() operator
fix(c_legacy): correct off-by-one in matrix_mul
test(core): add unit tests for Quaternion normalize
docs: improve quick-start in README
```

Types: `feat`, `fix`, `refactor`, `docs`, `test`, `perf`, `build`, `ci`, `chore`.

## Pull requests

- Title is a Conventional Commit-style summary.
- Description explains the **why**, not just the what.
- CI must be green before merge.
- Self-review your own diff in the PR view before merging.

## Tests

- New code must come with tests.
- Aim for ≥85 % coverage on `fieldnav_core` and `fieldnav_filters`.
- Run `ctest --test-dir build/debug` locally before pushing.

## Architecture Decision Records (ADR)

For non-trivial decisions, add an ADR in `docs/decisions/`. See
`docs/decisions/template.md`.
