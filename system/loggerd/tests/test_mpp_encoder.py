from pathlib import Path
import shlex
import shutil
import subprocess


def test_mpp_encoder_private_input_helper(tmp_path):
  repo_root = Path(__file__).resolve().parents[3]
  native_test = Path(__file__).with_suffix(".cc")
  compiler = shutil.which("clang++")
  readelf = shutil.which("readelf")
  assert compiler is not None, "clang++ is required for the native helper test"
  assert readelf is not None, "readelf is required to verify the test binary has no MPP/IPC dependencies"

  binary = tmp_path / "mpp_encoder_helper_test"
  compile_command = [
    compiler,
    "-std=c++1z",
    f"-I{repo_root}",
    f"-I{repo_root / 'third_party/linux/include'}",
    "-I/usr/local/include/rockchip",
    str(native_test),
    "-o",
    str(binary),
  ]
  print("native compile:", shlex.join(compile_command))
  compiled = subprocess.run(compile_command, cwd=repo_root, text=True, capture_output=True, check=False)
  assert compiled.returncode == 0, compiled.stdout + compiled.stderr

  dynamic = subprocess.run([readelf, "-d", str(binary)], cwd=repo_root, text=True,
                           capture_output=True, check=False)
  assert dynamic.returncode == 0, dynamic.stdout + dynamic.stderr
  print("native dynamic dependencies:", dynamic.stdout)
  for forbidden in ("librockchip_mpp", "librga", "libzmq"):
    assert forbidden not in dynamic.stdout, f"native harness unexpectedly links {forbidden}"

  print("native run:", str(binary))
  result = subprocess.run([str(binary)], cwd=repo_root, text=True, capture_output=True, check=False)
  assert result.returncode == 0, result.stdout + result.stderr
  print(result.stdout, end="")
  print(result.stderr, end="")
