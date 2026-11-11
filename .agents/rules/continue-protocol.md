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
