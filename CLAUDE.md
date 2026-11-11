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
3. **Haiku for chores.** Mechanical work goes to an Agent subagent with
   `model: "haiku"`: edit PROGRESS.md / docs from my decisions, git add/commit,
   file search, read+summarize, run `.\run_tests.ps1` and report.
   Me (Sonnet) only decide, spec, review. Opus only per dev-workflow Advisor rule.
4. **Product code via subagents.** Sonnet itself does not edit product code
   (`app/`, `core/`, `bindings/`, `cuda/`, `tests/`, CMake). It launches Agent
   subagents (haiku for parser/UI/mechanical, sonnet for C++ kinematics/numerics)
   with the spec section pasted in, then reviews the diff before the next substage.

## "review" checklist (stage)

1. Read `PROGRESS.md` → find stage with `stage-ready-for-review`. None → say so, stop.
2. Graphify query the stage areas (rule 2).
3. Haiku subagent: run `.\run_tests.ps1`, report pass/fail.
4. Read stage diff: `git diff stage-R<x-1>-done..stage-R<x>-done` (or from the last approved commit if no tag). Read every file in it.
5. Findings in Caveman. Verdict `stage-approved` or `stage-changes-requested` with `R<x>-fix` list.
6. If approved, write next stage spec pack `docs/specs/R<x+1>-spec.md`. Set its rows `planned`.
7. Haiku subagent: write status + findings + spec pack pointer into `PROGRESS.md`, commit
   `docs(progress): review stage R<x>`.
