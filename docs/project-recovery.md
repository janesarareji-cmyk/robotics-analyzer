# Project migration and recovery (R8.4)

Code: `app/project/migrations.py`, `recovery.py`, `diagnose.py`, serializer
(`write_atomic`, `load`), `ProjectController` (autosave timer, recover prompt).

## Files next to a project `name.robotproj`

| File | Meaning | Lifetime |
|------|---------|----------|
| `name.robotproj` | the project | - |
| `name.robotproj.tmp` | archive being written | removed after success or failure |
| `name.robotproj.prev` | previous file while a save runs | removed after the save succeeds |
| `name.autosave.robotproj` | autosave | written only while modified; removed on clean save and on close |
| `name.v<old>.bak.robotproj` | copy of an old-format file | created once when it is first opened; never overwritten |

## Atomic save

1. Write `name.robotproj.tmp`, `fsync`.
2. If the old file is a valid ZIP, move it to `name.robotproj.prev` (a damaged file never replaces a good `.prev`).
3. Move `.tmp` over `name.robotproj` (`os.replace`), delete `.prev`.

A kill at any point leaves the old file, the new file, or (very short window between steps 2 and 3)
only `.prev`. A truncated project file is never produced. A write error removes the `.tmp` and keeps
the old file. `load_best(path)` opens the project, and falls back to `.prev`.

## Autosave and crash recovery

- Timer in `ProjectController`; interval from the setting `project/autosave_s`
  (default 120 s, minimum 15 s; `AppSettings.autosave_interval_s`). It runs only while a project is
  open; it writes only if the project has a file path and unsaved changes. Atomic (tmp + replace).
  Untitled (never saved) projects are not autosaved.
- Opening a project whose autosave is newer asks "Recover?". Yes: the autosave is loaded, marked
  modified (save to keep it). No: the autosave is deleted.
- On startup (`app/main.py`) the recent projects are scanned for a newer autosave.
- If the main file cannot be read but `.prev` can, `.prev` is opened (warning) and marked modified.

## Migration

`MIGRATIONS = {from_version: fn}`; each `PROJECT_FORMAT_VERSION` step has a function (so far all steps
only added optional members, so they are no-ops):

| From | To | Added |
|------|----|-------|
| 1.0.0 | 1.1.0 | `visuals.json`, `assets/` |
| 1.1.0 | 1.2.0 | `trajectory.json` |
| 1.2.0 | 1.3.0 | `collision.json`, `configs.json` |

Opening an older file (via the app): backup `name.v<old>.bak.robotproj` first, then migrate IN MEMORY
(chained). The file on disk changes only when the user saves (written in the current format).
`ProjectData.metadata.format_version` keeps the version read from the file until the next save;
`ProjectData.migrated_from` is that version when a migration ran.
A newer or unknown format is refused with a message; nothing is written, no backup is made.

## Diagnosis

`app.project.diagnose.diagnose(path)` returns a `DiagnosisReport` (`issues`: code, severity, message,
suggested action): `not_found`, `not_a_zip`, `bad_crc`, `missing_metadata`, `invalid_json` (with line
and column), `wrong_schema`, `missing_asset`, `unknown_version`, `newer_version`.
`salvage(path)` loads what it can (damaged parts become defaults plus a warning).

## Fixtures and tests

`tests/python/fixtures/projects/make_fixtures.py` writes one real archive per format version
(`v1.0.0` ... `v1.3.0`) and the corrupt samples (`corrupt_*.robotproj`); the files are committed.
Tests: `test_migration.py`, `test_recovery.py` (includes the kill-during-save subprocess test).
