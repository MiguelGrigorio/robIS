from pathlib import Path
import subprocess
import sys


def test_flake8():
    package_root = Path(__file__).parents[1]
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "flake8",
            "robis_semantic_navigation",
            "test",
        ],
        cwd=package_root,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
