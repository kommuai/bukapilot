#!/usr/bin/env python3
import hashlib
import json
import lzma
import os
import struct
import subprocess
import time
from collections.abc import Generator
from typing import Callable

import requests

import openpilot.system.updated.casync.casync as casync

SPARSE_CHUNK_FMT = struct.Struct('H2xI4x')
CAIBX_URL = "https://commadist.azureedge.net/agnosupdate/"


def _pct_done(numer: int, denom: int) -> int:
  return 0 if denom <= 0 else max(0, min(100, (numer * 100 + denom - 1) // denom))


def _emit_progress(progress_callback: Callable[[int], None] | None, p: int) -> None:
  progress_callback and progress_callback(p)


class StreamingDecompressor:
  def __init__(self, url: str, on_download_percent: Callable[[int], None] | None = None) -> None:
    self.buf = b""
    self.on_download_percent = on_download_percent
    self.downloaded_bytes = 0

    self.req = requests.get(url, stream=True, headers={'Accept-Encoding': None}, timeout=60)
    self.total_bytes = int(cl) if (cl := self.req.headers.get("Content-Length")) is not None else 0
    self.it = self.req.iter_content(chunk_size=1024 * 1024)
    self.decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
    self.eof = False
    self.sha256 = hashlib.sha256()

  def read(self, length: int) -> bytes:
    while len(self.buf) < length:
      self.req.raise_for_status()

      try:
        compressed = next(self.it)
      except StopIteration:
        self.eof = True
        if (progress_callback := self.on_download_percent) is not None and (tb := self.total_bytes) > 0:
          _emit_progress(progress_callback, _pct_done(max(self.downloaded_bytes, tb), tb))
        break
      self.downloaded_bytes += len(compressed)
      if (tb := self.total_bytes) > 0:
        _emit_progress(self.on_download_percent, _pct_done(self.downloaded_bytes, tb))
      out = self.decompressor.decompress(compressed)
      self.buf += out

    result = self.buf[:length]
    self.buf = self.buf[length:]

    self.sha256.update(result)
    return result


def unsparsify(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  # https://source.android.com/devices/bootloader/images#sparse-format
  magic = struct.unpack("I", f.read(4))[0]
  assert(magic == 0xed26ff3a)

  # Version
  major = struct.unpack("H", f.read(2))[0]
  minor = struct.unpack("H", f.read(2))[0]
  assert(major == 1 and minor == 0)

  f.read(2)  # file header size
  f.read(2)  # chunk header size

  block_sz = struct.unpack("I", f.read(4))[0]
  f.read(4)  # total blocks
  num_chunks = struct.unpack("I", f.read(4))[0]
  f.read(4)  # crc checksum

  for _ in range(num_chunks):
    chunk_type, out_blocks = SPARSE_CHUNK_FMT.unpack(f.read(12))

    if chunk_type == 0xcac1:  # Raw
      # TODO: yield in smaller chunks. Yielding only block_sz is too slow. Largest observed data chunk is 252 MB.
      yield f.read(out_blocks * block_sz)
    elif chunk_type == 0xcac2:  # Fill
      filler = f.read(4) * (block_sz // 4)
      for _ in range(out_blocks):
        yield filler
    elif chunk_type == 0xcac3:  # Don't care
      yield b""
    else:
      raise Exception("Unhandled sparse chunk type")


# noop wrapper with same API as unsparsify() for non sparse images
def noop(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  while not f.eof:
    yield f.read(1024 * 1024)


def get_target_slot_number() -> int:
  current_slot = subprocess.check_output(["/boot/abctl"], encoding='utf-8').splitlines()[0].split('=')[1]
  assert(current_slot == "_a" or current_slot == "_b")
  return 1 if current_slot == "_a" else 0


def slot_number_to_suffix(slot_number: int) -> str:
  assert slot_number in (0, 1)
  return '_a' if slot_number == 0 else '_b'


def get_partition_path(target_slot_number: int, partition: dict) -> str:
  # On KA2 the inactive slot is always labeled `<name>_b` and the active
  # slot is always labeled `<name>` (see /usr/kommu/rename_labels.sh, which
  # runs on every swap). `target_slot_number` is, by definition, the
  # inactive slot, so resolve writes via the label convention instead of
  # a slot-index lookup. The previous implementation indexed by slot
  # number (slot 0 -> "", slot 1 -> "_b"), which was only correct while
  # slot A was active; on slot B the "target" path resolved to the
  # currently-mounted root partition and a flash would overwrite it.
  del target_slot_number  # unused; kept in signature for API compatibility
  path = f"/dev/disk/by-partlabel/{partition['name']}"
  if partition.get('has_ab', True):
    path += "_b"
  return path


def _assert_safe_target_path(path: str) -> None:
  """Refuse to write to a device node currently mounted as the root fs.

  Defense-in-depth against any future slot-resolution bug that could cause
  the updater to target the running filesystem's backing partition.
  """
  mounted_root: str | None = None
  try:
    with open("/proc/mounts", "r") as f:
      for line in f:
        parts = line.split()
        if len(parts) >= 2 and parts[1] == "/":
          mounted_root = parts[0]
          break
  except Exception:
    return  # fail-open if /proc/mounts is unreadable; don't block legit flashes
  if not mounted_root:
    return
  try:
    target_real = os.path.realpath(path)
    root_real = os.path.realpath(mounted_root)
  except Exception:
    return
  if target_real and target_real == root_real:
    raise Exception(
      f"refusing to flash {path}: it resolves to the currently-mounted "
      f"root partition ({mounted_root}); aborting to protect the active slot."
    )


def get_raw_hash(path: str, partition_size: int) -> str:
  raw_hash = hashlib.sha256()
  pos, chunk_size = 0, 1024 * 1024

  with open(path, 'rb+') as out:
    while pos < partition_size:
      n = min(chunk_size, partition_size - pos)
      raw_hash.update(out.read(n))
      pos += n

  return raw_hash.hexdigest().lower()

def verify_partition(target_slot_number: int, partition: dict[str, str | int], force_full_check: bool = False) -> bool:
  full_check = partition['full_check'] or force_full_check
  path = get_partition_path(target_slot_number, partition)

  if not isinstance(partition['size'], int):
    return False

  partition_size: int = partition['size']

  if not isinstance(partition['hash_raw'], str):
    return False

  partition_hash: str = partition['hash_raw']

  if full_check:
    return get_raw_hash(path, partition_size) == partition_hash.lower()
  else:
    with open(path, 'rb+') as out:
      out.seek(partition_size)
      return out.read(64) == partition_hash.lower().encode()


def clear_partition_hash(target_slot_number: int, partition: dict) -> None:
  path = get_partition_path(target_slot_number, partition)
  _assert_safe_target_path(path)
  with open(path, 'wb+') as out:
    partition_size = partition['size']

    out.seek(partition_size)
    out.write(b"\x00" * 64)
    os.sync()


def extract_compressed_image(target_slot_number: int, partition: dict, cloudlog, on_progress: Callable[[int], None] | None = None):
  path = get_partition_path(target_slot_number, partition)
  _assert_safe_target_path(path)
  size, last_download_p, last_emitted_p = partition['size'], -1, -1

  def emit_monotonic_progress(progress_percent: int) -> None:
    nonlocal last_emitted_p
    if (p := max(last_emitted_p, progress_percent)) != last_emitted_p:
      last_emitted_p = p
      _emit_progress(on_progress, p)

  def on_download_progress(download_percent: int) -> None:
    nonlocal last_download_p
    if (p := max(0, min(download_percent, 100))) != last_download_p:
      last_download_p = p
      emit_monotonic_progress(p)

  downloader = StreamingDecompressor(partition['url'], on_download_percent=on_download_progress)
  with open(path, 'wb+') as out:
    # Flash partition
    last_p, raw_hash, f = 0, hashlib.sha256(), unsparsify if partition['sparse'] else noop
    for chunk in f(downloader):
      raw_hash.update(chunk)
      out.write(chunk)
      if (p := _pct_done(out.tell(), size)) != last_p:
        last_p = p
        print(f"Installing {partition['name']}: {p}", flush=True)
        emit_monotonic_progress(p)

    if (rh := raw_hash.hexdigest().lower()) != partition['hash_raw'].lower():
      raise Exception(f"Raw hash mismatch '{rh}'")
    if (dh := downloader.sha256.hexdigest().lower()) != partition['hash'].lower():
      raise Exception(f"Uncompressed hash mismatch '{dh}'")
    if (tell := out.tell()) != size:
      raise Exception("Uncompressed size mismatch")
    emit_monotonic_progress(_pct_done(tell, size))
    os.sync()


def extract_casync_image(target_slot_number: int, partition: dict, cloudlog, on_progress: Callable[[int], None] | None = None):
  path = get_partition_path(target_slot_number, partition)
  # Seed from the currently-active slot. On KA2 the active partition is
  # always labeled `<name>` (no suffix) per /usr/kommu/rename_labels.sh.
  # Note: casync is not exercised on KA2 today (flash_partition always
  # takes the extract_compressed_image path), but keep this correct in
  # case it is ever enabled.
  size = partition['size']
  seed_path = f"/dev/disk/by-partlabel/{partition['name']}" if partition.get('has_ab', True) else path

  target = casync.parse_caibx(partition['casync_caibx'])

  sources: list[tuple[str, casync.ChunkReader, casync.ChunkDict]] = []

  # First source is the current partition.
  try:
    raw_hash = get_raw_hash(seed_path, size)
    caibx_url = f"{CAIBX_URL}{partition['name']}-{raw_hash}.caibx"

    try:
      cloudlog.info(f"casync fetching {caibx_url}")
      sources += [('seed', casync.FileChunkReader(seed_path), casync.build_chunk_dict(casync.parse_caibx(caibx_url)))]
    except requests.RequestException:
      cloudlog.error(f"casync failed to load {caibx_url}")
  except Exception:
    cloudlog.exception("casync failed to hash seed partition")

  # Second source is the target partition, this allows for resuming
  sources += [('target', casync.FileChunkReader(path), casync.build_chunk_dict(target))]

  # Finally we add the remote source to download any missing chunks
  sources += [('remote', casync.RemoteChunkReader(partition['casync_store']), casync.build_chunk_dict(target))]

  last_p = 0
  _emit_progress(on_progress, _pct_done(0, size))

  def progress(cur):
    nonlocal last_p
    if (p := _pct_done(cur, size)) != last_p:
      last_p = p
      print(f"Installing {partition['name']}: {p}", flush=True)
      _emit_progress(on_progress, p)

  stats = casync.extract(target, sources, path, progress)
  cloudlog.error(f'casync done {json.dumps(stats)}')

  os.sync()
  if not verify_partition(target_slot_number, partition, force_full_check=True):
    raise Exception(f"Raw hash mismatch '{partition['hash_raw'].lower()}'")
  _emit_progress(on_progress, _pct_done(size, size))


def flash_partition(target_slot_number: int, partition: dict, cloudlog, standalone=False, on_progress: Callable[[int], None] | None = None):
  cloudlog.info(f"Downloading and writing {partition['name']}")

  if verify_partition(target_slot_number, partition):
    cloudlog.info(f"Already flashed {partition['name']}")
    return

  # Clear hash before flashing in case we get interrupted
  full_check = partition['full_check']
  if not full_check:
    clear_partition_hash(target_slot_number, partition)

  path = get_partition_path(target_slot_number, partition)

  extract_compressed_image(target_slot_number, partition, cloudlog, on_progress)

  # Write hash after successful flash
  if not full_check:
    _assert_safe_target_path(path)
    with open(path, 'wb+') as out:
      out.seek(partition['size'])
      out.write(partition['hash_raw'].lower().encode())


def swap(manifest_path: str, target_slot_number: int, cloudlog) -> None:
  update = json.load(open(manifest_path))
  for partition in update:
    if not partition.get('full_check', False):
      clear_partition_hash(target_slot_number, partition)

  while True:
    suffix = '_a' if target_slot_number == 0 else '_b' 
    out = subprocess.check_output(f"/boot/abctl set-active {suffix}", shell=True, stderr=subprocess.STDOUT, encoding='utf8')
    if ("No such file or directory" not in out) and (f"current_slot={slot_number_to_suffix(target_slot_number)}" in out):
      cloudlog.info(f"Swap successful {out}")
      os.system(f"bash /usr/kommu/rename_labels.sh {target_slot_number}")
      break
    else:
      cloudlog.error(f"Swap failed {out}")


def flash_agnos_update(manifest_path: str, target_slot_number: int, cloudlog, standalone=False,
                       on_progress: Callable[[int], None] | None = None) -> None:
  update = json.load(open(manifest_path))
  last_emitted_p = -1
  total_size = sum(p['size'] for p in update if isinstance(p.get('size'), int))
  completed_size = 0

  def on_overall_progress(progress_percent: int) -> None:
    nonlocal last_emitted_p
    if (p := max(last_emitted_p, progress_percent)) != last_emitted_p:
      last_emitted_p = p
      _emit_progress(on_progress, p)

  cloudlog.info(f"Target slot {target_slot_number}")

  for partition in update:
    partition_size = partition['size'] if isinstance(partition.get('size'), int) else 0

    def on_partition_progress(partition_percent: int, base: int = completed_size, size: int = partition_size) -> None:
      if total_size > 0:
        on_overall_progress((base * 100 + size * max(0, min(partition_percent, 100))) // total_size)
      else:
        on_overall_progress(max(0, min(partition_percent, 100)))

    for retries in range(10):
      try:
        flash_partition(target_slot_number, partition, cloudlog, standalone, on_partition_progress)
        break
      except requests.exceptions.RequestException:
        cloudlog.exception("Failed")
        cloudlog.info(f"Failed to download {partition['name']}, retrying ({retries})")
        time.sleep(10)
    else:
      cloudlog.info(f"Failed to flash {partition['name']}, aborting")
      raise Exception("Maximum retries exceeded")
    completed_size += partition_size
    if total_size > 0:
      on_overall_progress(_pct_done(completed_size, total_size))

  on_overall_progress(_pct_done(total_size, total_size))
  cloudlog.info(f"AGNOS ready on slot {target_slot_number}")


def verify_agnos_update(manifest_path: str, target_slot_number: int) -> bool:
  update = json.load(open(manifest_path))
  return all(verify_partition(target_slot_number, partition) for partition in update)


if __name__ == "__main__":
  import argparse
  import logging

  parser = argparse.ArgumentParser(description="Flash and verify AGNOS update",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--verify", action="store_true", help="Verify and perform swap if update ready")
  parser.add_argument("--swap", action="store_true", help="Verify and perform swap, downloads if necessary")
  parser.add_argument("manifest", help="Manifest json")
  args = parser.parse_args()

  logging.basicConfig(level=logging.INFO)

  target_slot_number = get_target_slot_number()

  if args.verify:
    if verify_agnos_update(args.manifest, target_slot_number):
      swap(args.manifest, target_slot_number, logging)
      exit(0)
    exit(1)
  elif args.swap:
    while not verify_agnos_update(args.manifest, target_slot_number):
      logging.error("Verification failed. Flashing AGNOS")
      flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)

    logging.warning(f"Verification succeeded. Swapping to slot {target_slot_number}")
    swap(args.manifest, target_slot_number, logging)
  else:
    flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)
