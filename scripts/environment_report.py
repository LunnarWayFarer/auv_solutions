import json
import platform
import shutil
import subprocess
import sys


def version(command):
    if shutil.which(command[0]) is None:
        return {"available": False}
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return {"available": True, "returncode": result.returncode,
                "output": (result.stdout or result.stderr).strip()}
    except subprocess.TimeoutExpired:
        return {"available": True, "error": "timeout"}


def main():
    print(json.dumps({
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": sys.version.split()[0],
        "git": version(["git", "--version"]),
        "docker": version(["docker", "version", "--format", "{{json .}}"]),
        "ros2_available": shutil.which("ros2") is not None,
        "colcon_available": shutil.which("colcon") is not None,
    }, indent=2))


if __name__ == "__main__":
    main()
