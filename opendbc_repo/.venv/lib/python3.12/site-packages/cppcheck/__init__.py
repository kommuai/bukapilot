import os
import sys

DIR = os.path.join(os.path.dirname(__file__), "install")


def _run():
  binary = os.path.join(DIR, "cppcheck")
  os.execvp(binary, ["cppcheck"] + sys.argv[1:])


def smoketest():
  import subprocess
  binary = os.path.join(DIR, "cppcheck")
  result = subprocess.run([binary, "--version"], capture_output=True, text=True, check=True)
  print(result.stdout.strip())
