---
trigger: always_on
description: What the Builder (Gemini) does when the user says "continue" (or "go", "next", "proceed").
---

## Continue Protocol (Builder)

When the user says "continue" (or "go", "next", "proceed") with no other instruction, do these steps in order.

1. Read `.agents/rules/` and `PROGRESS.md`.
2. Run `graphify update .`. Query the graph for new or changed files. Do not create a file or class that already exists.
3. Find the substage named in `PROGRESS.md` → "Current". Read its status in the substage log.
4. Do the action for that status:

| Status | Action |
|--------|--------|
| `planned` or `in-progress` | Implement the substage. Follow its spec in `PROGRESS.md` → "Substage specs" exactly. |
| `changes-requested` | Apply every item in "Open review items" for that substage. Then tag each item `(fixed — pending review)`. Do not delete items; the Reviewer clears them on approval. |
| `ready-for-review` | Do nothing. Tell the user: "Waiting for Claude review of `<id>`." Stop. |
| `approved` | Set "Current" → "Substage" to the next row in the log and "Owner now" to Builder. If that row has no spec in "Substage specs", tell the user: "Need spec for `<id>` from Claude." Stop. Otherwise implement it as `planned`. |
| `blocked` | Tell the user the blocker. Stop. |

5. After implementing or fixing:
   - Build the code (`cmake --build build --config Release`).
   - Run `.\run_tests.ps1`. All tests must pass. If a test fails and you cannot fix it, add a `<id>-fix` row, set status `blocked`, explain in "Open review items", and stop.
   - Run `graphify update .`.
   - Update `PROGRESS.md`: set status `ready-for-review`, set "Owner now" to Reviewer, list the changed files under the substage spec.
   - Commit: `R<x>.<y>: <summary>` (or `R<x>.<y>-fix: <summary>` for review fixes). Include `PROGRESS.md` and `graphify-out/graph.json`. Push if a remote exists.
6. Stop. Do one substage per "continue". Do not start the next substage.
7. Report to the user in 3 lines or fewer: substage, test result, commit hash.

## Stage-Batch Loop (from Stage R3)

Authority: `docs/MASTER_PLAN.md` section 1. Before Stage R3, steps 1-7 above apply. From Stage R3 on, this loop replaces steps 6 and 7 for the stage.

1. Read `PROGRESS.md` and the stage spec pack `docs/specs/R<x>-spec.md`.
2. Run `graphify update .`.
3. Take the first substage in the stage not `committed`. Implement it. Follow its spec exactly.
4. Build. Run `.\run_tests.ps1`. All tests must pass. If a substage fails twice, stop. Write the failure in `PROGRESS.md` → "Blockers". Tell the user. Stop.
5. Run `graphify update .`. Commit `R<x>.<y>: <summary>` with `PROGRESS.md` and `graphify-out/graph.json`.
6. Repeat from step 3 until every substage of the stage is committed. Do not wait for review between substages.
7. Run the stage gate (`docs/MASTER_PLAN.md` 1.3). Record the results in `PROGRESS.md`.
8. Push if a remote exists: `git push origin main`, then `git tag stage-R<x>-done`, then `git push origin stage-R<x>-done`. Never push with a red test. Never force-push.
9. Set stage status `stage-ready-for-review`. Set "Owner now" to Reviewer.
10. If the stage is blocking review (R3, R4, R5.2 decision): stop. Wait for `stage-approved`.
11. Otherwise: if the next stage spec pack exists, start it at step 1. If not, stop. Tell the user: Claude must write `docs/specs/R<x+1>-spec.md` first.
