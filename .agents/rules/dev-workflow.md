---
trigger: always_on
description: Development workflow, agent roles, build pipeline, and stage/substage testing rules for this project.
---

## Development Workflow

### Agent Roles

| Agent | Model | Where | Role |
|-------|-------|-------|------|
| Builder | Gemini 3.8 Medium | Antigravity | ALL product implementation: code, scaffolding, builds, tests, bug fixes, graphify updates, git commits and pushes |
| Orchestrator / Reviewer | Claude Sonnet 5.5 | Claude Code (user talks to it here) | Planning and orchestration only: break stages into substages, write substage specs/acceptance criteria, review Builder output, pick model tier. Does NOT write product code |
| Helper | Claude Haiku 5.5 | Claude Code subagents | Low-intelligence planning-side chores: edit PROGRESS.md / plan docs from Orchestrator's decisions, git config, file search, read/summarize files, run tests and report output |
| Advisor | Claude Opus 5.5 | Claude Code `/advisor` or Opus subagent | EXTREMELY RARE: only when Sonnet cannot decide safely (core numerical method, architecture/schema later stages depend on, CUDA memory model, a fix that failed twice) |

The user drives the work. The user talks to the Orchestrator in Claude Code and relays its specs/reviews to the Builder in Antigravity.

### Stage-Batch Workflow (active from Stage R3)

Authority: `docs/MASTER_PLAN.md` section 1. Where it conflicts with "Workflow Per Substage", "Stage Completion Gate" or the review steps below, MASTER_PLAN wins for Stage R3 onward.

- Claude writes the whole stage spec pack (`docs/specs/R<x>-spec.md`) before the stage starts.
- Gemini builds ALL substages of the stage in order. Per substage: tests green, `graphify update .`, one commit `R<x>.<y>: <summary>`. No wait for review between substages.
- Stage gate (MASTER_PLAN 1.3), then push `main` and tag `stage-R<x>-done`.
- Claude reviews at stage end.
  - Blocking (Gemini waits for `stage-approved` before the next stage): R3, R4, and the R5.2 precision decision.
  - Non-blocking (next stage starts at once): R5 (after R5.2), R6, R7, R8.
  - Max one unreviewed stage. Findings become `R<x>-fix` substages, done first.
- Stop rule: a substage fails its tests twice → stop. Write the failure in `PROGRESS.md` → "Blockers". Wait for the user.
- No remote yet. The user runs `git remote add origin <url>` once. Gemini pushes only when a remote exists. Never force-push.

### Work Routing

- Writes or changes product code (`app/`, `core/`, `bindings/`, `cuda/`, `tests/`, CMake) → **Gemini**.
- Decides what to build, in what order, and whether it is correct → **Sonnet**.
- Mechanical doc/bookkeeping/git chore on the Claude side → **Haiku subagent**.
- Hard, high-risk decision Sonnet is not confident about → **Opus**, then log it in `PROGRESS.md` → "Decisions".

### Claude Model Tiering (token saving)

- Sonnet 5.5 orchestrates by default and adjusts the tier per task.
- Delegate to Haiku when the task needs no judgement (search, read, run, report, bookkeeping).
- Escalate to Opus only when Sonnet is not confident in the answer, or the cost of a wrong answer is high (core math, GPU memory model, schema design that later stages depend on).
- Drop back to Sonnet/Haiku as soon as the hard part is done.
- Record every Opus consultation in `PROGRESS.md` → "Decisions".

### User Shortcuts

- To Gemini: "continue" → continue the current stage from the first substage not `committed`, following the stage spec pack, then stage gate and push. See `.agents/rules/continue-protocol.md` (it also lists the read order for a new session).
- To Claude: "review" → review the stage marked `stage-ready-for-review` in `PROGRESS.md`, set `stage-approved` or `stage-changes-requested`, and write the next stage spec pack if approved.

### Progress File

`PROGRESS.md` (repo root) is the shared handoff file between Builder and Reviewer.
Read it at the start of every session. Update the substage log row and "Current"
section at every handoff. Follow the handoff protocol inside that file.

### Workflow Per Substage (legacy R0-R2; from R3 see Stage-Batch Workflow and `continue-protocol.md`)

0. **Gemini (Builder)** reads `PROGRESS.md`, then runs `graphify update .` and checks the graph BEFORE writing code. Use `graphify query` to find files that are new or changed since the last session, and existing code to reuse. Do not create a file or class that already exists in the graph.
1. **Gemini (Builder)** implements the substage: writes code, runs commands, verifies it compiles/runs.
2. **Gemini (Builder)** runs `graphify update .` after every build to keep the knowledge graph current.
3. **Claude (Reviewer)** reads `graphify-out/GRAPH_REPORT.md` and queries the graph (`graphify query`) to get full context BEFORE reviewing any code.
4. **Claude (Reviewer)** reviews the code and reports findings in Caveman English (per agent-styles rule).
5. **Gemini (Builder)** applies fixes if needed, rebuilds, updates graphify again.
6. Repeat until substage is clean.

### Stage Completion Gate (legacy wording; from R3 the gate is `docs/MASTER_PLAN.md` 1.3)

A **stage** consists of multiple **substages**.

The stage is considered complete ONLY when ALL of the following pass:
- All substage unit tests pass (CTest or Python test framework).
- All numerical reference tests pass (FK, IK, Jacobians, dynamics — where applicable).
- GPU/CPU equivalence tests pass (where applicable from R5 onward).
- Claude has reviewed and approved the final build.
- `graphify update .` has been run and `graphify-out/graph.json` is current.

Do NOT proceed to the next stage until the current stage's gate passes.

### Error Substage Rule

If a substage fails during implementation or testing, do NOT skip it or move on.
Instead, insert a new substage immediately after the failing one, named `<stage>.<n>-fix`.
Example: if R0.4 fails, insert R0.4-fix before R0.5.
The fix substage has one goal: make the previous substage pass its test gate.
Only after the fix substage passes do you advance to the original next substage.
This keeps the sequence strictly linear. No substage is skipped. No substage is left broken.

### Key Rules

- Git (Builder owns commits): after each substage is implemented and its tests pass, Gemini commits with message `R<x>.<y>: <summary>`. Review fixes get their own commit `R<x>.<y>-fix: <summary>`. Commit `PROGRESS.md` and `graphify-out/graph.json` in the same commit. Never commit `build/` or binaries (see `.gitignore`). Do not rewrite history. Gemini pushes at the stage gate (not after each commit) once a remote is configured (none yet). Never force-push.
- Implement one substage at a time. Do not start the next substage until the current one compiles, runs, and has passing tests.
- Keep the application runnable after every substage.
- Do not add new dependencies without first verifying Windows compatibility in a proof-of-compatibility build.
- Benchmark before optimizing. Record hardware/software versions with every benchmark.
- Each feature requires tests and documentation before it is marked complete.

### Hardware Context (OMEN 16)

- GPU: NVIDIA RTX 5060 Laptop — **8 GB VRAM** (8151 MiB, confirmed by nvidia-smi)
- RAM: 24 GB system RAM
- CPU: AMD Ryzen 9 HX series

GPU memory budget: 8 GB VRAM is the hard ceiling. Design all CUDA batch sizes, chunk sizes, and memory allocation strategies with this limit. Never hardcode batch sizes; always compute dynamically from available VRAM.

