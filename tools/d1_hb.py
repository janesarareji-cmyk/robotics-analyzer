"""Heartbeat harness: python tools/d1_hb.py <robot> [platform] [shotdir]  (real event loop, no manual advance)."""
import faulthandler, os, sys, time, statistics
from pathlib import Path
os.environ.setdefault("QT_QPA_PLATFORM", sys.argv[2] if len(sys.argv) > 2 else "windows")
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from PySide6.QtCore import QCoreApplication, QEvent, QTimer
from PySide6.QtWidgets import QApplication
from app.settings import AppSettings
AppSettings._ORGANIZATION = "RoboticsAnalyzer_hb"
AppSettings._APPLICATION = "RoboticsAnalyzer_hb"
from app.main_window import MainWindow


def main():
    name = sys.argv[1]
    shotdir = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    QApplication.instance() or QApplication([])
    win = MainWindow(); win.settings._settings.clear()
    win.collision_action.setChecked(True)
    win.resize(1600, 950); win.show()
    win._import_sample = True
    from tests.python.test_live_blocking import load
    load(win, name)
    for _ in range(100):
        QCoreApplication.processEvents(); time.sleep(0.01)
    gaps = []; last = [time.perf_counter()]
    def beat():
        now = time.perf_counter(); gaps.append(now - last[0]); last[0] = now
    hb = QTimer(); hb.setInterval(20); hb.timeout.connect(beat)
    demo = win.target_panel.demo
    faulthandler.dump_traceback_later(25, repeat=True)
    last[0] = time.perf_counter(); hb.start()
    demo.status_changed.connect(lambda t: print(f'[{time.time()-T0:.1f}] {t}', flush=True))
    T0 = time.time()
    demo.play_requested.connect(lambda tr,info: print(f'[{time.time()-T0:.1f}] play_requested {info["name"]} dur={tr.duration:.2f}', flush=True))
    demo._planned.connect(lambda g,p: print(f'[{time.time()-T0:.1f}] planned {p[0]} ok={p[2]["ok"]} {p[2]["message"]}', flush=True))
    demo._failed.connect(lambda g,m: print('FAILED', m, flush=True))
    ok = win.start_demo()
    t0 = time.time(); k = 0
    while demo.running and time.time() - t0 < 40:
        QCoreApplication.processEvents(); time.sleep(0.002)
        if shotdir and time.time() - t0 > k * 3.0:
            win.viewport.plotter.render(); shotdir.mkdir(parents=True, exist_ok=True)
            win.grab().save(str(shotdir / f"demo_live_{name}_{k}.png")); k += 1
    dt = time.time() - t0
    print('waiting_playback', demo.waiting_playback, 'active', win.task_runner.active_count, 'bg',win.sim_panel._bg_play,'timer',win.sim_panel._timer.isActive(),'clock',win.sim_panel._engine.clock.state,'player',win.sim_panel._player, 'pt', getattr(win.sim_panel._player,'t',None), 'shown', win.sim_panel._shown(), 'dockvis', win.sim_dock.isVisible(), flush=True)
    hb.stop(); faulthandler.cancel_dump_traceback_later()
    g = sorted(gaps) or [0]
    print(f"RESULT {name} ok={ok} running={demo.running} dur={dt:.1f}s n={len(gaps)} median={statistics.median(g)*1000:.0f}ms p95={g[int(len(g)*.95)]*1000:.0f}ms max={g[-1]*1000:.0f}ms summary={demo.last_summary.text() if demo.last_summary else None}", flush=True)
    win.project_controller._is_modified = False
    win.sim_dock.hide(); win.close()
    for _ in range(50):
        QCoreApplication.processEvents(); QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete); time.sleep(0.01)

main()
