---
trigger: always_on
description: What the Sonnet orchestrator does when the user says "continue" (or "go", "next", "proceed"). It builds the stage with Claude Sonnet subagents.
---

## Continue Protocol (Sonnet builders, one stage at a time)

Claude only. Gemini is retired. Sonnet builder subagents write all product code.
Haiku does very low chores. Tests run once per stage, not per substage.
Authority: `.agents/rules/dev-workflow.md` and `docs/MASTER_PLAN.md` section 1.

### Read order (every new session)

1. `.agents/rules/*.md` (all files).
2. `PROGRESS.md` → "Current" and the "Substage log".
3. `docs/MASTER_PLAN.md` section 1 (points to dev-workflow).
4. The current stage spec pack `docs/specs/<stage>-spec.md`. Read its
   "Common rules" and "Code facts" first.
5. Docs the spec names.

### Stage loop

When the user says "continue" (or "go", "next", "proceed") with no other instruction:

1. Read the files in "Read order". Run `graphify update .`. Use `graphify query`
   for the stage areas. Do not create a file or class that already exists.
2. Look at the stage status in "Current":

| Stage status in "Current" | Action |
|---------------------------|--------|
| `stage-ready-for-review` | No stage lock. Start the next stage at step 3 if its spec pack exists. If not, tell the user: "Need `docs/specs/<next>-spec.md`." Stop. |
| `stage-changes-requested` | Put the `<x>-fix` substages first in the next stage. Then go to step 3. |
| `blocked` | Tell the user the blocker. Stop. |
| `in-progress` | Go to step 3. |

3. Launch Sonnet builder subagents with the stage spec pasted in. The builder
   builds all substages of the stage in order. One commit per substage:
   `<stage>.<n>: <summary>`. No tests, no review, no graphify between substages.
4. Stage end: full test suite once (CTest + pytest + stage milestone), stage gate
   (`docs/MASTER_PLAN.md` 1.3), `graphify update .`. Record results under "Current".
   Commit `PROGRESS.md` and `graphify-out/graph.json` with the stage-end commit.
5. Tag `stage-<x>-done`. Push `main` and the tag if a remote exists. Never force-push.
6. Set stage status `stage-ready-for-review`. Owner → Reviewer. Next stage starts
   at once (no stage lock). Opus is looped in at stage end for advice and to
   review the next spec pack. Log each Opus call in "Decisions".
7. Stop rule: stage tests still red after two fix attempts → write the failure in
   "Blockers", set status `blocked`, stop, wait for the user.
8. Report to the user in 3 lines or fewer: substages done, test result, last commit hash.
