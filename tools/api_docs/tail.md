## Script runner (`app/script_runner.py`)

`ScriptRunner(task_runner, project_provider=None, time_limit=None)`: `run_string(code, name)`, `run_file(path)`,
`cancel()`, `is_running`; signals `started()`, `output(text, is_err)` (one line), `finished(status, message)` with
status `ok` / `error` / `cancelled` / `timeout` (message = traceback for `error`). One script at a time; the
script runs in a `TaskRunner` worker thread, the GUI stays responsive. The script namespace has
`robotics_api` (also `api`) and `app`, a read-only snapshot: `has_project`, `project_name`, `project_path`,
`robot` (a `Robot` copy or None), `q` (current joint vector), `version`. For `run_file`, `__file__` is set and the
script folder is on `sys.path` while it runs. Output uses thread-aware stdout / stderr: only writes made by the
script thread are captured, the app's own output is untouched (streams are restored when the script ends,
even if the script replaced them). A script exception never leaves the runner. Setting: `time_limit` seconds
(default None).

UI: View -> Script console (dock, hidden at start: editor, Run, Cancel, output); File -> Run script... (file
dialog). Output also goes to the log panel (`[script]` / `[script stderr]`).

Security: the namespace is not a sandbox. Scripts run in the application process with full Python rights.

## Examples

`examples/fk_sweep.py` (FK sweep), `examples/plan_and_simulate.py` (plan around a box, time parametrize,
simulate the tracking), `examples/export_data.py` (simulate and export CSV and NPZ). They use
`examples/planar_2r.urdf`. Run them from a shell with the repo root and `bindings/` on `PYTHONPATH`, or from
the script console (it sets `__file__`).

## Cancel behaviour

`simulate`, `plan` and `workspace` call `check_cancel()` often and raise `ScriptCancelled` when the runner asked
to stop (Cancel button, or the time limit -> `ScriptTimeLimit`). A script with its own long loop calls
`api.check_cancel()` inside the loop. `api.is_cancelled()` tests without raising. Outside the runner these are
no-ops. The token is thread-local: threads started by the script are not cancelled. A script stuck in pure
Python code that never calls the API cannot be stopped.

## Error notes

-   `InvalidArgument` is also a `ValueError`.
-   `ScriptTimeLimit` is a subclass of `ScriptCancelled`.
-   The original exception is kept as `__cause__` of the API error.
