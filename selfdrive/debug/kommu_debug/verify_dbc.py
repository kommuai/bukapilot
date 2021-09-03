#!/usr/bin/env python3
import sys
import numpy as np
from opendbc.can.dbc import dbc

def verify(f1,f2):

  dat = np.load(f1,allow_pickle=True)
  dat = [dat.item()[i] for i in dat.item()]

  dbc_dat = dbc(f2)
  dbc_dat = dbc_dat.msgs
  prev_sig = None

  for i in dat:
    can_addr = i[1]
    can_sig_name = i[0]

    if prev_sig != can_sig_name:
      print(can_sig_name.center(65, "-"))

    try:
      for j in dbc_dat[can_addr][1]:
        dbc_start_bit = j[1]
        dbc_sig_name= j[0]
        dat_lh_bit = i[2]
        dat_hl_bit = i[3]
        if dat_lh_bit & (1 << dbc_start_bit) or dat_hl_bit & (1 << dbc_start_bit):
          print(f"[MATCH] {dbc_sig_name} matches {can_sig_name} at address {can_addr}")
    except KeyError:
      print(f"[NOISE] Address: {can_addr}, Triggering signal: {can_sig_name}")

    prev_sig = can_sig_name


if __name__ == "__main__":
  if len(sys.argv) > 2:
    verify(sys.argv[1],sys.argv[2])
  else:
    print("Need two arguments: npy & dbc")
