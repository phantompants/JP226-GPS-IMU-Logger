# PlatformIO pre-build script: stamps the git commit and build date into
# JP226_GIT_HASH and JP226_BUILD_DATE for shared/Version.h.
import datetime
import subprocess

Import("env")  # noqa: F821 - provided by PlatformIO


def git(*args):
    try:
        return subprocess.check_output(
            ["git", *args], cwd=env.subst("$PROJECT_DIR"),  # noqa: F821
            stderr=subprocess.DEVNULL).decode().strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


commit = git("rev-parse", "--short", "HEAD") or "nogit"
if git("status", "--porcelain", "--untracked-files=no"):
    commit += "+"
env.Append(CPPDEFINES=[  # noqa: F821
    ("JP226_GIT_HASH", env.StringifyMacro(commit)),  # noqa: F821
    ("JP226_BUILD_DATE",
     env.StringifyMacro(datetime.date.today().isoformat())),  # noqa: F821
])
