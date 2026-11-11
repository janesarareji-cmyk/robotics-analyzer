---
trigger: always_on
description: What the Sonnet orchestrator does when the user says "continue" (or "go", "next", "proceed"). It runs the loop by launching Builder subagents.
---

## Continue Protocol (Sonnet orchestrator + Builder subagents)

The Sonnet orchestrator runs this loop. It launches Builder subagents (Agent tool, haiku or sonnet tier per substage) with the spec section pasted in. Gemini is retired.

From Stage R3 the project uses the **stage-batch loop** below.
Authority: `docs/MASTER_PLAN.md` section 1. The old per-substage protocol
(R0 to R2) is at the end, for history only. Do not use it.

### Read order (every new session)

1. `.agents/rules/*.md` (all files).
2. `PROGRESS.md` → "Current" and the "Substage log".
3. `docs/MASTER_PLAN.md` section 1 (workflow, gate, push).
4. The current stage spec pack `docs/specs/R<x>-spec.md`. Read its
   "Common rules" and "Code facts" first.
5. Docs the spec names (for R3: `docs/urm-schema-v2.md`).

### Stage-batch loop

When the user says "continue" (or "go", "next", "proceed") with no other
instruction:

1. Read the files in "Read order".
2. Run `graphify update .`. Use `graphify query` for the area of the
   substage. Do not create a file or class that already exists.
3. Look at "Current" in `PROGRESS.md`:

| Stage status in "Current" | Action |
|---------------------------|--------|
| `stage-ready-for-review` | Do nothing. Tell the user: "Waiting for Claude review of stage R<x>." Stop. |
| `stage-changes-requested` | Do the `R<x>-fix` rows first (spec in "Open review items"), one commit each `R<x>-fix: <summary>`. Then run the stage gate again and set `stage-ready-for-review`. Stop. |
| `stage-approved` | Start the next stage at step 4 if its spec pack exists. If not, tell the user: "Need `docs/specs/R<x+1>-spec.md` from Claude." Stop. |
| `blocked` | Tell the user the blocker. Stop. |
| `in-progress` | Go to step 4. |

4. Take the first substage row of the stage whose status is not
   `committed`. Set it `in-progress`. Implement it. Follow its spec exactly.
   If the spec is unclear or contradicts the code: do not guess. Write the
   question in `PROGRESS.md` → "Blockers", set the row `blocked`, stop,
   tell the user.
   Sonnet reviews the substage diff before the next substage starts.
5. Build if C++ changed: `cmake --build build --config Release`.
   Run `.\run_tests.ps1`. All tests must pass.
   A failure you fix in the same substage is normal. If the same substage
   is still red after a second full fix attempt: stop. Write the failure in
   `PROGRESS.md` → "Blockers". Set the row `blocked`. Tell the user. Stop.
6. Run `graphify update .`. Set the row `committed`. Update "Current"
   (next substage). Commit `R<x>.<y>: <summary>` with `PROGRESS.md` and
   `graphify-out/graph.json`. Never commit `build/` or binaries.
7. Repeat from step 4 until every substage of the stage is `committed`. Do
   not wait for review between substages.
8. Run the stage milestone test and the stage gate
   (`docs/MASTER_PLAN.md` 1.3). Record the results in `PROGRESS.md` →
   "Current".
9. Tag `stage-R<x>-done` locally. Push only if a remote exists:
   `git push origin main`, then `git push origin stage-R<x>-done`. Never
   push with a red test. Never force-push.
10. Set stage status `stage-ready-for-review`. Set "Owner now" to Reviewer.
11. If the stage is blocking review (R3, R4, R5.2 decision): stop. Wait for
    `stage-approved`. Otherwise: start the next stage if its spec pack
    exists; if not, stop and tell the user.
12. Report to the user in 3 lines or fewer: substages done, test result,
    last commit hash.

### Legacy per-substage protocol (R0 to R2 only, do not use)

R0 to R2 used one review per substage: status `ready-for-review` after each
substage, Claude set `approved` / `changes-requested`, and the Builder did
one substage per "continue". Specs for that period are archived in
`docs/specs/R2-spec.md`.
