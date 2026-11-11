"""
tests/python/conftest.py
pytest configuration — adds build paths so core_bridge can be imported.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Add bindings/ directory to sys.path so core_bridge.pyd can be imported
bindings_dir = Path(__file__).parent.parent.parent / "bindings"
if str(bindings_dir) not in sys.path:
    sys.path.insert(0, str(bindings_dir))
