# Security audit — 2026-11-15

Scope: installer and helper scripts (`install.bat`, `install.ps1`, `setup.ps1`, `run.ps1`, `run.bat`,
`uninstall.bat`, `uninstall.ps1`, `packaging/`), untrusted file parsing (URDF, `.robotproj`, meshes,
JSON), code execution surfaces (plugins, scripts, subprocess), supply chain, app data, repo hygiene.
Read-only: no code changed, no network, no installer run.

Threat model: a stranger clones the public repo and runs the installer; the user then opens untrusted
robot files (URDF, URM JSON, `.robotproj`, meshes), plugins and scripts.

Method: `graphify query` for the areas, then code read. "Confirmed" = code path traced (and where
noted, behaviour checked with a small Python run in a scratch folder). "Suspicion" = not traced to the
end, needs a test.

Local checks run:

-   `git grep -nEi "(api[_-]?key|secret|token|password|BEGIN (RSA|PRIVATE))"`: no secrets. Hits are
    the cancel token, a test file named `secret.txt`, and the `.gitignore` comment.
-   `git ls-files` for key/cert/env/db files: none.
-   `git grep` for `shell=True|eval(|exec(|pickle|allow_pickle|yaml.load|Invoke-Expression|iex `:
    see S-items. No `pickle`, `yaml.load`, `Invoke-Expression`; `np.load(..., allow_pickle=False)` in
    `app/sim/export.py:177`.
-   Absolute user paths in tracked files: none (graphify-out has relative paths only).
-   `pip_audit`: NOT installed in `.venv`, skipped.
-   Expat in `.venv`: 2.8.2 (has billion-laughs amplification limits).

## Summary

| ID | Sev | Area | Finding | Status |
|---|---|---|---|---|
| F1 | High | `.robotproj` load | Drive-relative zip member name (`assets/D:numpy.py`) writes outside the temp asset dir, into the cwd of another drive = the install folder when it is not on the `%TEMP%` drive; next start imports it (code execution) | confirmed; **fixed (S1) 2026-11-15** |
| F2 | Med | `.robotproj` / URDF | `mesh_path` / `file://host/...` / absolute paths are used verbatim: UNC path triggers SMB (NTLM hash leak) on open; any local file is copied into the project on save | confirmed (known M2, was "won't fix"); **fixed (S2) 2026-11-15** |
| F3 | Med | `uninstall.ps1` | `cmd /c "rmdir /s /q "$path""` expands `%VAR%` inside the folder name: folder `C:\%HOMEPATH%` deletes the user profile | confirmed by code, not run; **fixed (S3) 2026-11-15** |
| F4 | Med | `uninstall.ps1` / `install.ps1` | Marker trust fails open: no `dir=` line skips the ownership check; missing `mode` = `fresh` = delete whole folder; installer adopts `mode` from a marker found in an existing clone | confirmed; **fixed (S4) 2026-11-15** |
| F5 | Med | plugins | Every folder in `%APPDATA%\RoboticsAnalyzer\plugins` is imported and run at app start, enabled by default, no first-run prompt; permissions are cooperative only | confirmed (documented) |
| F6 | Med | supply chain | Install pulls unpinned `main` (git or zip), unpinned `requirements.txt` (`>=`, no hashes, `pip install --upgrade pip`), FetchContent by mutable tag (pybind11 not shallow) | confirmed |
| F7 | Low | `.robotproj` / mesh | No size limits: whole zip members read into memory, all assets written to temp, meshes fully parsed before the 2e6 triangle cap | confirmed |
| F8 | Low | uninstall | Fallback `Remove-Item -Recurse -Force` in Windows PowerShell 5.1 can follow junctions inside the tree | suspicion |
| F9 | Low | Windows search order | `shutil.which("ffmpeg")`, `subprocess.run(["git", ...])` search the cwd (= install folder) first | confirmed (needs a file in the install folder, see F1) |
| F10 | Low | installer | `-RepoUrl` is pasted into a string-built git command line: `"` in it injects git options (`--upload-pack=...`, `-c core.sshCommand=...`) | confirmed |
| F11 | Low | startup recovery | A `<name>.autosave.robotproj` planted next to a recent project is offered at start ("Recover?") | confirmed |
| F12 | Low | JSON | Deeply nested JSON in a project member raises `RecursionError`, not in the `load()` except list | suspicion (Python behaviour known, not run) |
| F13 | Low | misc | `exec` with `{"__builtins__": {}}` in `parse_numpy_block` (escapable; test-only caller); `shell=True` with constant strings in `benchmark.py` / `bench_fk.py`; `uninstall.bat` temp copy name `%RANDOM%` (same-user race only); shortcut owner check is a string prefix | confirmed |
| F14 | Info | repo hygiene | No `LICENSE`, no `SECURITY.md`, no `.github/` (Dependabot / CodeQL), tags not signed, commit author e-mail public in history | confirmed |

No Critical. No secrets in tracked files.

## Confirmed findings

### F1 High — zip member with a drive letter escapes the asset folder

-   Code: `app/project/serializer.py:304-311` `_read_visuals`:
    `target = Path(asset_dir) / PurePosixPath(n).name` then `target.write_bytes(zf.read(n))`.
    The comment says "no zip-slip". `PurePosixPath(...).name` drops `/` and `..` (and `zipfile` turns
    `\` into `/` on Windows), but a last component like `D:numpy.py` is a drive-relative Windows path.
    `WindowsPath(asset_dir) / "D:numpy.py"` gives `D:numpy.py` when `asset_dir` is on another drive.
-   Checked in a scratch run (Python 3.13.15, no file written): with `%TEMP%` on C: and cwd
    `D:\M.Tech`, `assets/D:numpy.py` resolves to `D:\M.Tech\numpy.py`; `assets/C:x.txt` stays inside
    the asset folder (same drive).
-   Runs on open, before any validation: `load()` -> `_read_visuals` (line 209). Drag-and-drop
    (`app/main_window.py:2058`) and startup recovery also call `open_project`.
-   Exploit: install folder on D: (this repo is on D:), `%TEMP%` on C:. Victim opens `robot.robotproj`
    with member `assets/D:numpy.py` (or `D:pyvista.py`, `D:ffmpeg.exe`). `run.ps1:7` / `run.bat` set
    cwd to the install folder and start `python -m app.main`, which puts cwd first on `sys.path`, so
    the next start imports the planted `numpy.py`: code execution as the user. On other drives the file
    lands in the drive root (`E:\`), which is a file drop, not execution.
-   Default install (`%LOCALAPPDATA%\RoboticsAnalyzer` on C:, same drive as `%TEMP%`) is not
    affected; `C:\RoboticsAnalyzer` and any D:/E: clone are.
-   Fix: accept an asset name only when it matches `^[A-Za-z0-9._ -]{1,128}$`, has no `:` and is not
    `.`/`..`; then `real = (Path(asset_dir) / name).resolve()` and require
    `real.parent == Path(asset_dir).resolve()`. Skip the rest with a warning. Same check for any
    other writer that takes names from an archive. Effort S.

### F2 Med — mesh paths from untrusted files are used verbatim

-   `.robotproj`: `app/importers/visuals.py:60` `LinkVisual.from_dict` keeps `mesh_path`;
    `serializer.py:318` rewrites only paths starting with `assets/`; others stay as given.
-   Open: `visuals.py:186` `os.path.isfile(v.mesh_path)` and `pv.read` run on any path, including
    `\\attacker\share\m.stl` -> outbound SMB with the user's NTLM hash (offline cracking / relay).
-   Save: `serializer.py:286` copies `mesh_path` into `assets/` when `not fallback` and `is_file()`,
    with no extension check. A project with `mesh_path = C:\Users\<u>\.ssh\id_ed25519`,
    `fallback = false` embeds that file when the user saves, and leaks it when the project is shared.
-   URDF: `visuals.py:117-121` `file://host/share/x.stl` becomes a UNC path; `:127-131` absolute
    paths and `../` are accepted; `_first_file` stats them -> same SMB leak on import. Non-mesh
    extensions are marked fallback in `urdf.py:589`, so URDF alone does not embed foreign files.
-   Fix: on project load keep `mesh_path` only if it points into the extracted asset folder, else
    fallback + warning. URDF: reject UNC / `file://` with a host; resolve relative paths and allow only
    the URDF folder, its ancestors used for `package://`, and the configured search roots; on save copy
    only files with a mesh extension. Effort S-M.

### F3 Med — `%` in the install path is expanded by cmd in uninstall

-   `uninstall.ps1:81`: `& cmd.exe /c "rmdir /s /q `"$path`""`. cmd expands `%VAR%` even inside
    quotes. Windows allows `%` in folder names; `install.ps1:482-515` `Test-DirChoice` does not reject it.
-   Exploit: an install (or a stranger's "install to this folder" instructions) at `C:\%HOMEPATH%`.
    Uninstall runs `rmdir /s /q "C:\\Users\<u>"` = the whole profile. Rare, but the loss is total.
-   Fix: drop `cmd /c`; delete with `[System.IO.Directory]::Delete($path, $true)` (removes reparse
    points without following them) inside try/catch, then report what is left. Reject `%`, `!`, `^`,
    `&` in `Test-DirChoice`. Effort S.

### F4 Med — marker trust fails open

-   `uninstall.ps1:118`: the ownership check runs only `if ($mk.ContainsKey("dir") ...)`. A marker
    with just the header line passes. `:128-129`: no `mode` = `"fresh"` = `Remove-Tree $Dir`
    (whole folder).
-   `install.ps1:557`: for an existing clone the installer keeps `mode` from a marker already there
    (`$Script:Mode = $mk["mode"]`), then rewrites it with that mode (`:654`, `:818`). Only
    `Dir == $PSScriptRoot` forces `clone` (`:558`).
-   Exploit: a fork commits `.robotics-analyzer-install` (header + `mode=fresh`; `.gitignore` does not
    stop a committed file). A user clones it by hand, runs the installer from another copy with
    `-InstallDir <clone>`, or runs `uninstall.bat` in the clone: their whole clone, including their own
    work, is deleted (one "Really uninstall?" prompt, default No; `-Yes` skips it).
-   Fix: uninstall requires `created_by=install.ps1`, `dir=` equal to the folder, and `mode` in
    {`fresh`,`clone`}, else refuse. Unknown → treat as `clone` (remove only build output). Installer:
    ignore a marker's `mode` for a folder that already held source before this run; write `mode=clone`.
    Add an install id (GUID) to the marker and to a per-user record
    (`%LOCALAPPDATA%\RoboticsAnalyzer\installs.txt`); uninstall deletes a whole folder only when both
    match. Effort S.

### F5 Med — plugins auto-run at start

-   `app/main_window.py:879-888` creates `PluginManager` and calls `scan()` at start.
    `app/plugins/loader.py:128-129` `is_enabled` defaults to `"1"`; `:195-219` imports the entry module
    with `spec.loader.exec_module` and calls `register(host)` in-process.
-   File/network permissions (`app/plugins/host.py:141-168`) only guard the helper functions; plugin
    code can import `os`/`subprocess` freely. The UI and docs say so
    (`app/panels/plugin_manager_panel.py:42-43`, `docs/plugin-guide.md:9-11`, `docs/user-guide.md:269`).
-   A project file cannot run code on open (checked: no script or plugin path in the project
    schema; scripts run only from File -> Run script / console, `app/main_window.py:1933-1938`).
-   Exploit: user unzips a "plugin pack" into the plugins folder (or malware with user rights drops one);
    it runs at every start before the user sees the Plugins dock.
-   Fix: a new plugin (name + sha256 of its folder) is `disabled` until the user enables it in the
    Plugins dock with a "this runs code with your rights" confirm; re-ask when the hash changes. Effort S-M.

### F6 Med — unpinned supply chain

-   `install.ps1:28,613`: `git clone` of the default branch; `:619-634` zip of `refs/heads/main`, no
    hash or signature check. TLS is HTTPS by `Invoke-WebRequest` defaults (Win 11 / .NET 4.8: TLS 1.2+);
    GitHub redirects to codeload, fine.
-   `setup.ps1:140,144`: `pip install --upgrade pip` and `pip install -r requirements.txt` with `>=`
    ranges, no `--require-hashes`, no lock file (`requirements.txt:2-9`). Only `mujoco==4.0.1` pinned
    (`requirements-dev.txt`).
-   `CMakeLists.txt:53-58` Eigen from gitlab tag `3.4.0`, `:62-67` nlohmann_json `v3.11.3`,
    `bindings/CMakeLists.txt:3-7` pybind11 `v2.13.6` (not shallow): tags can be moved by the upstream
    account; no `URL_HASH`.
-   winget IDs are exact (`-e`) and well known (`Git.Git`, `Python.Python.3.13`, `Kitware.CMake`,
    `Microsoft.VisualStudio.2022.BuildTools`); `--accept-package-agreements` accepts licences for the
    user (consent issue, not a vuln).
-   Dev env (PROGRESS.md): an unrelated PyPI package `pinocchio` 0.5.1 is installed. It is not in any
    requirements file. Name confusion with the real `pin` package; uninstall it.
-   Fix: installer clones a release tag (`--branch v1.1.x --depth 1`) and the zip path checks a SHA-256
    printed in the README/release; `requirements.lock` from `pip-compile --generate-hashes`, setup uses
    `pip install --require-hashes -r requirements.lock`; FetchContent `GIT_TAG <commit sha>` or
    `URL ...tar.gz URL_HASH SHA256=...`. Effort M.

### F7 Low — no resource limits on project and mesh input

-   `serializer.py:311,384` `zf.read(n)` loads each member fully; no limit on member count, size or
    compression ratio. A 40 KB zip bomb fills RAM or the temp disk (`mkdtemp` folder).
-   `visuals.py:195-196`, `urdf.py:598-611`: `pv.read` parses the whole mesh, the 2 000 000 triangle cap
    is checked afterwards.
-   Fix: before extraction check `sum(info.file_size) <= 512 MB`, each member `<= 256 MB`,
    `file_size / compress_size <= 200`, `<= 2000` members; mesh file size cap (e.g. 200 MB) before
    `pv.read`. Effort S.

### F9 Low — cwd in the program search order

-   `app/sim/video_export.py:49` `shutil.which("ffmpeg")` (Python on Windows checks the cwd first);
    `app/sim/export.py:30`, `app/compute/benchmark.py:44` run `git` by name (CreateProcess also looks in
    the cwd). cwd = install folder (`run.ps1:7`, `run.bat`, `setup.ps1:25`).
-   Only matters together with F1 (a planted `ffmpeg.exe` / `git.exe`) or a malicious checkout.
-   PowerShell scripts call tools by name (`py`, `python`, `cmake`, `nvidia-smi`, `git`, `winget`);
    PowerShell does not run from the cwd, but `Start-Process` (`install.ps1:229`) resolves `git`/`winget`
    with CreateProcess rules (cwd = folder of `install.bat`). Same "malicious checkout" caveat.
-   Fix: resolve tools once with `shutil.which(..., path=os.environ["PATH"])` after removing `""`/`"."`
    entries, or set `NoDefaultCurrentDirectoryInExePath=1` in `app/main.py` before any lookup; in the
    installer use `(Get-Command git -CommandType Application).Source` as `-FilePath`. Effort S.

### F10 Low — `-RepoUrl` argument injection

-   `install.ps1:613`: `Invoke-Logged "git" ("clone `"$RepoUrl`" `"$Dir`"")` — one argument string.
    `-RepoUrl 'x" --upload-pack="calc.exe'` (or `-c core.sshCommand=...` with an ssh URL) runs a
    command. Needs the user to pass the parameter (social engineering).
-   Fix: validate `^https://github\.com/[\w.-]+/[\w.-]+(\.git)?$` (or drop the parameter outside tests);
    pass `--` before the URL. Effort S.

### F11 Low — autosave next to the project is trusted

-   `app/project/project_controller.py:337-347` `check_startup_recovery` opens any recent project that has
    a newer `<name>.autosave.robotproj` (`app/project/recovery.py:29-32`, same folder). In a shared or
    download folder another party can plant one; the user gets a "Recover?" prompt at start and F1/F2 apply.
-   Fix: keep autosaves under `%LOCALAPPDATA%\RoboticsAnalyzer\autosave\<sha1(path)>.robotproj`. Effort S.

### F13 Low — small items

-   `app/analysis/matrices.py:162` `exec(compile(text, ...), {"__builtins__": {}}, ns)`: empty builtins is
    not a sandbox (`().__class__.__mro__[1].__subclasses__()`). Only caller is
    `tests/python/test_matrix_panel.py:69,192`. Fix: parse with `ast.literal_eval` on the list part or move
    to tests. Effort S.
-   `app/compute/benchmark.py:31` and `tools/bench_fk.py:37` `shell=isinstance(cmd, str)`: constant
    strings only (`:54`, `:67`), no injection today. Fix: list form. Effort S.
-   `uninstall.bat:5-8`: `%TEMP%\ra_uninstall_%RANDOM%.ps1` (15-bit name, `copy /y`): only a same-user
    process can race it; no privilege boundary. Fix: run the copy from a `New-Guid` folder. Effort S.
-   `install.ps1:800`, `uninstall.ps1:148`: shortcut owner check `TargetPath.StartsWith($Dir)` also matches
    `C:\RoboticsAnalyzer2\...`. Fix: compare with `$Dir + "\"`. Effort S.
-   `setup.ps1:45,49` `Remove-Item -Recurse -Force $p` without `-LiteralPath` (wildcard chars `[ ]` in the
    path). Effort S.
-   `install.log` (in the install folder, `.gitignore`d) holds answers, commands and full paths (user
    name). No secrets. OK.
-   C++/CUDA: `cuda/workspace_gpu.cpp:411` size check uses division (no `n*n_dof*4` overflow); `from_arrays`
    range checks were added in A1.4 (`core/kin/compiled_arrays.cpp`). Kernels not re-traced here.
-   Logs: `%APPDATA%\RoboticsAnalyzer\logs`, fallback `%TEMP%\...` (`app/logger.py:85-91,153-156`):
    per-user folders, rotating. OK.
-   URDF XML: `xml.etree.ElementTree.fromstring` (`app/importers/urdf.py:131`) does not fetch external
    entities; expat 2.8.2 limits entity amplification. OK.

### F14 Info — repo hygiene (propose only)

-   No `LICENSE` (public repo without a licence = all rights reserved; add MIT/BSD/Apache-2.0).
-   No `SECURITY.md` (how to report, supported version, "plugins/scripts are not sandboxed",
    "open only trusted project files" until F1/F2 are fixed).
-   No `.github/`: add Dependabot (pip, github-actions), a CI job with `pip-audit -r requirements.txt`,
    CodeQL for Python and C++, and secret scanning + push protection in repo settings.
-   Branch protection on `main` (no force push, PR required); signed tags for releases
    (`git tag -s v1.1.0`) and SHA-256 of release zips.
-   Commit author e-mail is public in history (187 commits). Use the GitHub noreply address for new
    commits (`git config user.email <id>+<login>@users.noreply.github.com`); old commits stay.

## Suspicions (not traced)

-   F8 Low: `uninstall.ps1:82-84` fallback `Remove-Item -LiteralPath $path -Recurse -Force` runs when
    `rmdir` left files. Windows PowerShell 5.1 is known to recurse into directory junctions/symlinks in
    some cases (fixed in PowerShell 6+). A junction a user made inside the install folder (e.g.
    `robots\ -> D:\my_robots`) could have its target emptied. `rmdir /s /q` itself does not follow
    junctions. Test: junction to a scratch folder, lock a file, run uninstall `-DryRun` then real in a VM.
-   F12 Low: Python `json.loads` on `[[[[...` 100 000 deep raises `RecursionError` (a `RuntimeError`), not
    caught by `serializer.py:215-233`; the "Cannot Open Project" path and `.prev` fallback are skipped and
    the exception goes to the Qt slot. nlohmann_json 3.11.3 (`core/urm/robot_model.cpp:478`) parses
    iteratively; deep-structure destruction was made iterative in 3.10 — not checked here.
-   `Expand-Archive` (`install.ps1:634`) zip-slip handling in the PowerShell 5.1 Archive module was not
    checked; the zip comes from GitHub over TLS, so it only matters with F6 (repo compromise).
-   QLabel / QTableWidget text from URDF names (link/joint names) may be shown as Qt rich text
    (`Qt::AutoText`); an `<img src="\\host\x.png">` name could trigger an SMB fetch. Not traced.
-   CUDA kernels with model sizes from untrusted URDF (very many joints): shared-memory table size
    per block vs joint count not re-checked in this audit.

## Fix plan

Order: data-loss and code-execution first. One commit per item `S<n>: <summary>`, tests per stage.

| ID | Summary | Files | Test | Effort | Status |
|---|---|---|---|---|---|
| S1 | Asset names: allow-list, no `:`, resolved path must stay in the asset dir (F1) | `app/project/serializer.py` | `tests/python/test_load_safety.py`: zip with `assets/D:x.py`, `assets/C:x`, `assets/..`, `assets/a:b` -> nothing written outside `asset_dir`, warning logged, project still opens | S | **fixed 2026-11-15** |
| S2 | Confine `mesh_path`: project paths only inside the asset dir; URDF no UNC / `file://host`, relative paths inside allowed roots; save copies mesh extensions only (F2) | `app/project/serializer.py`, `app/importers/visuals.py`, `app/importers/urdf.py` | project with `mesh_path=\\h\s\m.stl` and `C:\...\secret.txt`: no `isfile` on UNC (monkeypatch), save does not embed it; URDF `file://h/s/m.stl` -> `URDF_MESH_MISSING` | S-M | **fixed 2026-11-15** |
| S3 | Uninstall without `cmd /c`; `Directory.Delete` (no junction follow); reject `% ! ^ &` in install path (F3, F8) | `uninstall.ps1`, `install.ps1` | Pester or `tests/python/test_installer_scripts.py` running `uninstall.ps1 -DryRun` on `%TMP%\%HOMEPATH%`-named folder: refuses / deletes only that folder; junction target survives | S | **fixed 2026-11-15** |
| S4 | Marker fail-closed: require `created_by`, `dir`, known `mode`, install GUID + per-user record; installer never adopts `mode=fresh` for pre-existing source (F4) | `install.ps1`, `uninstall.ps1` | header-only marker -> uninstall refuses; marker with `mode=fresh` in a clone -> installer writes `clone` | S | **fixed 2026-11-15** |
| S5 | Plugins: new or changed plugin (folder sha256) disabled until the user confirms in the Plugins dock (F5) | `app/plugins/loader.py`, `app/panels/plugin_manager_panel.py`, `docs/plugin-guide.md` | `test_plugins.py`: new folder -> status `disabled` ("needs approval"), approve -> `ok`; edit file -> `disabled` again | S-M |
| S6 | Size limits for project zips and mesh files (F7) | `app/project/serializer.py`, `app/importers/visuals.py` | zip with 1 GB declared member / ratio 1000 -> `ProjectFormatError`; 300 MB fake STL -> fallback cube without `pv.read` | S |
| S7 | Catch `RecursionError` (and `MemoryError`) in `load()` (F12) | `app/project/serializer.py` | 100 000-deep JSON member -> `ProjectFormatError`, `.prev` fallback runs | S |
| S8 | Pin the supply chain: release tag clone + zip SHA-256, hashed lock file, FetchContent by commit or URL_HASH; remove stray `pinocchio` from the dev env (F6) | `install.ps1`, `setup.ps1`, `requirements.lock` (new), `CMakeLists.txt`, `bindings/CMakeLists.txt`, `README.md` | fresh `setup.ps1 -Clean` in a VM with the lock; tampered zip hash -> installer stops | M |
| S9 | Tool lookup without cwd; `-RepoUrl` validation and `--`; shortcut prefix with `\`; `-LiteralPath` in setup (F9, F10, F13) | `app/main.py`, `app/sim/video_export.py`, `app/sim/export.py`, `app/compute/benchmark.py`, `install.ps1`, `uninstall.ps1`, `setup.ps1` | `ffmpeg.exe` dummy in cwd is not picked; `-RepoUrl 'x" --upload-pack=y'` -> refused | S |
| S10 | Autosave in `%LOCALAPPDATA%` instead of next to the project (F11) | `app/project/recovery.py`, `app/project/project_controller.py`, `docs/project-recovery.md` | `test_recovery.py`: autosave path under the app data folder; planted sibling autosave is ignored | S |
| S11 | Remove `exec` from `parse_numpy_block` (move to tests or literal parse); list-form subprocess (F13) | `app/analysis/matrices.py`, `app/compute/benchmark.py`, `tools/bench_fk.py` | `test_matrix_panel.py` unchanged and green | S |
| S12 | Repo hygiene: `LICENSE`, `SECURITY.md`, Dependabot, pip-audit + CodeQL workflow, branch protection, signed tags, noreply e-mail (F14) | `LICENSE`, `SECURITY.md`, `.github/dependabot.yml`, `.github/workflows/security.yml` | workflow runs green on a PR | S |

Until S1 and S2 land: open `.robotproj` files only from trusted sources, and keep the install on the same
drive as `%TEMP%` (the default `%LOCALAPPDATA%\RoboticsAnalyzer`).

## Status of S1-S4 (fixed 2026-11-15)

-   S1 (F1): `app/project/serializer.py` `_read_visuals` extracts an asset only when its name passes
    `is_safe_asset_name` (`app/importers/visuals.py`: `^[A-Za-z0-9._-]{1,128}$`, no trailing dot, no
    Windows reserved name, one component) and the resolved target is directly inside the temp folder; bad
    names are skipped with a warning. On save, names are sanitised the same way. Tests:
    `tests/python/test_project_security.py`.
-   S2 (F2): `is_remote_or_device_path` refuses UNC, device paths, `file://host`, stream colons. Open never
    reads a `mesh_path` outside `assets/` (cleared, fallback cube). Save embeds only existing regular local
    files with a mesh extension. URDF `resolve_mesh` and `load_visual_mesh` refuse remote paths.
    Not done: URDF absolute local paths and `../` still resolve (same as before); `package://` roots
    are unchanged, so normal robots keep working.
-   S3 (F3, F8): `uninstall.ps1` has no `cmd.exe`; it deletes with `[System.IO.Directory]::Delete`, removes
    inner junctions first (the link, never the target), refuses a junction as the install folder, paths
    with `% ! ^ &`, drive roots, system folders and the profile or any folder that contains it.
    `install.ps1` `Test-DirChoice` rejects `% ! ^ &`. Shortcut ownership now compares `Dir + "\"`.
-   S4 (F4): uninstall needs `created_by=install.ps1`, `version=<n>`, `mode` in {fresh, clone} and
    `dir=` equal to the resolved folder, else it refuses. `install.ps1` writes `version=1` and never takes
    `mode` from a marker found in a folder that already holds source (always `clone`). Not done: the
    install GUID and per-user record.
