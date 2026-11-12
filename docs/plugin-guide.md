# Plugin guide (R8.2)

Plugins extend Robotics Analyzer with a file importer, a path planner or an analysis panel.
Samples: `plugins_examples/` (URM JSON importer, linear planner, joint info panel).

## Read this first: what the guard is NOT

Plugins are Python code that runs **inside the application process**. The permission system is a
**cooperative guard, not an OS sandbox**. It stops an honest plugin from touching files, network
or GPU it did not declare. It cannot stop hostile code: a plugin can import `os`, open sockets
or read any file by itself. **Install only plugins you trust.** The permission dialog says the
same.

Limits you should know:

-   A plugin call that hangs cannot be killed. The host runs each call in a daemon thread, waits
    for the timeout, then abandons the thread (it keeps running in the background until the
    process ends), reports it and disables the plugin for the session.
-   An analysis panel is built on the GUI thread (it is Qt code). It cannot be timed out. An
    exception is caught and the plugin is disabled; an endless loop there freezes the UI.
-   Plugin code is imported at load time (guarded, with a load timeout, default 10 s).

## Folder layout

The plugins folder is a setting (`plugins/folder`); default `%APPDATA%/RoboticsAnalyzer/plugins`.
One sub folder per plugin:

```
plugins/
  my_plugin/
    plugin.json
    my_module.py        (or a package folder my_module/__init__.py)
```

## plugin.json

| key | required | meaning |
|-----|----------|---------|
| `name` | yes | letters, digits, `_ . -`, unique |
| `version` | yes | `1.2` or `1.2.3` |
| `api_version` | yes | `robotics_api` version you wrote for, `MAJOR.MINOR`. Same MAJOR as the app, MINOR not newer than the app |
| `entry_point` | yes | `module:function`; the module is searched inside the plugin folder only |
| `kind` | yes | `importer`, `planner` or `analysis` |
| `permissions` | no | list from `file_read`, `file_write`, `network`, `gpu` (default none) |
| `description` | no | text for the manager |

An unknown key, unknown kind, unknown permission or an incompatible `api_version` rejects the
plugin with a clear message (status `rejected`). The manifest is checked without running any
plugin code. A disabled plugin is never imported.

## The entry point

`entry_point` names a function `register(host)`. It runs once at load time and registers one or
more items of the plugin's kind on the `host` object.

-   importer: `host.register_importer(label, file_filter, import_file)`.
    `import_file(path) -> ImportResult-like`: an object with `.model` (a `core_bridge.RobotModel`,
    or `None` plus a `report` for a clean failure), optional `.report`, `.visuals`,
    `.collisions`; a bare `RobotModel` is accepted. The model must pass `model.validate()`.
    The file the user picked is readable by the plugin while `import_file` runs.
    The importer appears under File > Import as "<label> (plugin)...".
-   planner: `host.register_planner(name, plan_fn)`. `plan_fn` has the signature of
    `app.planning.planner.plan`: `plan_fn(model, start_q, goal_q, collision=None, limits=None,
    cancel=None, **options)`. It is called with extra keyword options (`time_limit`, `seed`,
    `resolution`, `progress`, `do_shortcut`): accept `**kwargs`. Return an object with `success`
    (bool), `path` (array `(n, dof)`, first row = `start_q`, last row = `goal_q` within 1e-9,
    all rows inside the joint limits), and optional `time`, `iterations`, `reason`. On failure
    return `success=False` with a `reason`; that is not an error. A joint vector goal is a list
    of `dof` numbers; a TCP pose goal is a 4x4 matrix (return a failure if you do not support it).
    The planner appears in the Planning dock combo as "<name> (plugin)". Check `cancel.is_set()`.
-   analysis: `host.register_analysis_panel(title, factory)`. `factory(view) -> QWidget`.
    `view` is a read-only live project view (same attributes as the script `app` handle:
    `has_project`, `project_name`, `robot`, `q`, ...; every access takes a fresh snapshot,
    assignment raises). The panel becomes a dock listed in View > Plugin panels.

## Host capabilities

Every plugin gets `host.name`, `host.log(msg)` and `host.permissions` (what it declared).
Beyond that, only what the manifest declares:

-   `file_read`: `host.read_text(path)`, `host.read_bytes(path)`. Scope: the plugin's own folder
    and the file (and its folder) the user picked for the current import call. Anything else
    raises `PermissionError`.
-   `file_write`: `host.write_text(path, text)`, `host.write_bytes(path, data)`. Scope: only
    `host.data_dir` (`<plugin folder>/data`).
-   `network`: `host.http_get(url, timeout=10)`. The attribute does **not exist** unless the
    manifest declares `network` (network is off by default).
-   `gpu`: `host.gpu_info()` returns a dict (`available`, `name`, ...).

First use of a declared permission opens a dialog (Yes / No). The answer is remembered per plugin
in the settings; the manager shows it and "Revoke permissions" forgets it (the next use asks
again). A denied or undeclared use raises `PermissionError` inside the plugin; the host catches it
and reports it, the plugin stays enabled. File scope is checked before the dialog.

## Failure handling

Each call runs guarded. These disable the plugin **for the session** and show the reason in the
manager and the log: the plugin raised, the call timed out (default 60 s; planners get
`2 * time_limit + 10 s`; after the user cancels, 2 s of grace), or the result is garbage (wrong
type, failed validation). Enabling the plugin again in the manager reloads it. A permission denial
alone does not disable it.

Status values in the manager: `ok`, `rejected` (bad manifest), `error` (load failed), `disabled`
(by the user, or by a failure this session). Enable / disable is saved in the settings.

## API version rule

Plugins declare the `robotics_api` version they target. A plugin written for `1.0` loads on every
later `1.y`. A new MAJOR may break it; the loader then rejects the plugin instead of crashing.
