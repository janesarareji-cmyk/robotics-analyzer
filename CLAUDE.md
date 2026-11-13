# CLAUDE.md

Claude Code does not auto-load `.agents/rules/`. These imports make it load them:

@.agents/rules/agent-styles.md
@.agents/rules/dev-workflow.md
@.agents/rules/graphify.md
@PROGRESS.md

## Claude hard rules (every turn, no exceptions)

1. **Caveman.** All replies to user in Caveman English (agent-styles Rule 1).
   Short. Blunt. Still exact: file:line, names, numbers.
2. **Graphify first.** Before any review or spec: run `graphify query "<topic>"`
   (or `graphify explain` / `graphify path`). Read
   `graphify-out/GRAPH_REPORT.md` only if query not enough. Raw file reads come after.
3. **Claude only.** No Gemini. All dev work is done by Claude agents.
4. **Sonnet builds.** All product code (`app/`, `core/`, `bindings/`, `cuda/`,
   `tests/`, CMake) is written by Sonnet Agent subagents, with the spec section
   pasted in. The main session does not edit product code.
5. **Haiku for very low tasks only.** git add/commit/push, file search,
   PROGRESS.md bookkeeping from decisions already made, run tests and report.
6. **Opus advisor.** Use Opus only when necessary (hard math, architecture/schema
   later work depends on, GPU memory model, fix that failed twice). Also loop
   Opus in after each major chunk of development (stage end) for advice and to
   review the next plan / spec pack. Log every Opus call in PROGRESS.md "Decisions".
7. **Test per stage, not per substage.** No test runs or reviews between
   substages. Full test suite once when the stage is complete.
8. **No stage lock.** Next stage starts without waiting for review approval.

## "review" checklist (stage)

1. Read `PROGRESS.md` → find stage with `stage-ready-for-review`. None → say so, stop.
2. Graphify query the stage areas (rule 2).
3. Haiku subagent: run the full test suite (`.\run_tests.ps1`, or CTest + pytest by hand), report pass/fail.
4. Read stage diff: `git diff stage-<x-1>-done..stage-<x>-done` (or from the last reviewed commit if no tag).
5. Findings in Caveman. Problems become `<x>-fix` substages at the top of the next stage (non-blocking).
6. Opus advisor reviews the next stage spec pack `docs/specs/<next>-spec.md` and gives advice.
7. Haiku subagent: write status + findings + spec pack pointer into `PROGRESS.md`, commit
   `docs(progress): review stage <x>`.
