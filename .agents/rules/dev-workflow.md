---
trigger: always_on
description: Development workflow, agent roles, build pipeline, and stage/substage testing rules for this project.
---

## Development Workflow

Claude agents only. Gemini is retired.

### Agent Roles

| Agent | Model | Role |
|-------|-------|------|
| Orchestrator | main Claude Code session | Talks to the user. Plans, writes stage spec packs, launches builders, reviews at stage end. Does NOT write product code |
| Builder | Claude Sonnet subagents | ALL product implementation: code, tests, builds, bug fixes, graphify updates, substage commits |
| Helper | Claude Haiku subagents | Very low tasks only: git add/commit/push, file search, PROGRESS.md bookkeeping from decisions already made, run tests and report |
| Advisor | Claude Opus | Only when necessary (core numerical method, architecture/schema later stages depend on, CUDA memory model, a fix that failed twice). Also looped in after each major chunk of development (stage end) for advice and to review the next plan / spec pack |

### Stage Workflow

- Orchestrator writes the whole stage spec pack (`docs/specs/<stage>-spec.md`) before the stage starts. Opus reviews it when it is a new major plan.
- Sonnet builders build all substages of the stage in order. No test runs, no reviews, no checks between substages. One commit per substage: `<stage>.<n>: <summary>`.
- When the stage is complete: full test suite once (CTest + pytest + stage milestone), `graphify update .`, update `PROGRESS.md`, commit, tag `stage-<x>-done`, push `main` + tag.
- Stage review is non-blocking (no stage lock): the next stage starts at once. Review findings become `<x>-fix` substages done first in the next stage.
- Opus is looped in at stage end for advice and to review the next plan.
- Stop rule: stage tests still fail after two fix attempts → stop, write the failure in `PROGRESS.md` → "Blockers", wait for the user.
- Never force-push.

### Work Routing

- Writes or changes product code (`app/`, `core/`, `bindings/`, `cuda/`, `tests/`, CMake) → **Sonnet builder subagent**, spec section pasted in.
- Decides what to build, in what order, and whether it is correct → **Orchestrator**.
- Very low mechanical chore → **Haiku subagent**.
- Hard, high-risk decision, or stage-end plan review → **Opus**, then log it in `PROGRESS.md` → "Decisions".

### User Shortcuts

- "continue" → build from the first substage not `committed`, all substages of the stage in order, then stage tests, gate, push. See `.agents/rules/continue-protocol.md`.
- "review" → review the stage marked `stage-ready-for-review` (non-blocking), Opus reviews the next spec pack.

### Progress File

`PROGRESS.md` (repo root) is the shared handoff file. Read it at the start of every session. Update it at the end of every stage.

### Fix Rule

If the stage tests fail, add `<stage>-fix` substages that make them pass. Do not move on with red stage tests.

### Key Rules

- Git: builder commits each substage `<stage>.<n>: <summary>`. Commit `PROGRESS.md` and `graphify-out/graph.json` with the stage-end commit. Never commit `build/` or binaries (see `.gitignore`). Do not rewrite history. Push at stage end. Never force-push.
- Do not add new dependencies without first verifying Windows compatibility in a proof-of-compatibility build.
- Benchmark before optimizing. Record hardware/software versions with every benchmark.
- Each feature needs tests and documentation before the stage is marked complete.

### Hardware Context (OMEN 16)

- GPU: NVIDIA RTX 5060 Laptop — **8 GB VRAM** (8151 MiB, confirmed by nvidia-smi)
- RAM: 24 GB system RAM
- CPU: AMD Ryzen 9 HX series

GPU memory budget: 8 GB VRAM is the hard ceiling. Design all CUDA batch sizes, chunk sizes, and memory allocation strategies with this limit. Never hardcode batch sizes; always compute dynamically from available VRAM.
