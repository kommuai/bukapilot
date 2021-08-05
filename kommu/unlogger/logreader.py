#!/usr/bin/env python3
import os
import sys
import bz2
import urllib.parse
import capnp
from cereal import log as capnp_log

def fileReader(fn):
    return open(fn, "rb")

class LogReader(object):
  def __init__(self, fn, canonicalize=True, only_union_types=False):
    data_version = None

    # check if its a file or directory or a list
    if isinstance(fn, list) or os.path.isdir(fn):
      cum_data = b''
      for log in fn:
          _, ext = os.path.splitext(urllib.parse.urlparse(log).path)
          with fileReader(log) as f:
            dat = f.read()

          dat = bz2.decompress(dat)
          cum_data += dat
      dat = cum_data
    else:
      _, ext = os.path.splitext(urllib.parse.urlparse(fn).path)
      with fileReader(fn) as f:
        dat = f.read()
      dat = bz2.decompress(dat)

    ents = capnp_log.Event.read_multiple_bytes(dat)
    self._ents = list(ents)
    self._ts = [x.logMonoTime for x in self._ents]
    self.data_version = data_version
    self._only_union_types = only_union_types

  def __iter__(self):
    for ent in self._ents:
      if self._only_union_types:
        try:
          ent.which()
          yield ent
        except capnp.lib.capnp.KjException:
          pass
      else:
        yield ent

if __name__ == "__main__":
  import codecs
  # capnproto <= 0.8.0 throws errors converting byte data to string
  # below line catches those errors and replaces the bytes with \x__
  codecs.register_error("strict", codecs.backslashreplace_errors)
  log_path = sys.argv[1]
  lr = LogReader(log_path)

