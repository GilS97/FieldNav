# ADR-0003 — Apache-2.0 license

## Status

Accepted

## Context

The project will be public on GitHub from the start. We need a license that:

- allows industrial use (Naïo, Vitirover, etc. should be able to use it)
- protects against patent trolling
- is widely understood and accepted

Options: MIT, BSD-2/3, Apache-2.0, GPL-3.0, AGPL-3.0.

## Decision

Use **Apache License 2.0**.

## Consequences

- ✅ Permissive: usable in commercial closed-source products.
- ✅ Explicit patent grant (better than MIT/BSD for industrial contexts).
- ✅ Compatible with most robotics frameworks (ROS 2 is Apache-2.0).
- ⚠️ Slightly more verbose than MIT — but boilerplate is one file.
