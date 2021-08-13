#!/bin/bash/env python3
import sys
import numpy as np
from collections import defaultdict

def gen_SG(name, start_bit, size=1, is_little_endian=False, is_signed=False, \
    factor=1, offset=0, tmin=0, tmax=1, units=""):

  return f'  SG_ {name} : {start_bit}|{size}@{1 if is_little_endian else 0}{"-" if is_signed else "+"} ({factor},{offset}) [{tmin},{tmax}] "{units}" XXX\n'

def gen_BO(name, address, length):
  return f'\nBO_ {address} {name}: {length} XXX\n'

def gen_init():
  return 'VERSION ""\n\n\nNS_ :\n\tNS_DESC_\n\tCM_\n\tBA_DEF_\n\tBA_\n\tVAL_\n\tCAT_DEF_\n\tCAT_\n\tFILTER\n\tBA_DEF_DEF_\n\tEV_DATA_\n\tENVVAR_DATA_\n\tSGTYPE_\n\tSGTYPE_VAL_\n\tBA_DEF_SGTYPE_\n\tBA_SGTYPE_\n\tSIG_TYPE_REF_\n\tVAL_TABLE_\n\tSIG_GROUP_\n\tSIG_VALTYPE_\n\tSIGTYPE_VALTYPE_\n\tBO_TX_BU_\n\tBA_DEF_REL_\n\tBA_REL_\n\tBA_DEF_DEF_REL_\n\tBU_SG_REL_\n\tBU_EV_REL_\n\tBU_BO_REL_\n\tSG_MUL_VAL_\nBS_:\n\nBU_: XXX\n'

def get_bit_positions(val):
  v = val
  pos = []
  for i in range(64):
    v >>= 1
    if v & 1:
      pos.append(i)
  return pos

def gen_dbc(obj, fn):

    f = open(fn,"a")
    f.write(gen_init())

    dat = np.load(obj,allow_pickle=True)
    dat = [dat.item()[i] for i in dat.item()]
    sig = defaultdict(list)
    for i in dat:
      can_addr = i[1]
      can_sig_name = i[0]
      dat_lh_bit = i[2]
      dat_hl_bit = i[3]

      sig[can_addr].append([can_sig_name,dat_lh_bit,dat_hl_bit])

    for addr in sig:
      BO = gen_BO("SIGNAL", addr, "DLC")
      f.write(BO)
      for signal in sig[addr]:
        bit_pos = get_bit_positions(signal[1])
        for pos in bit_pos:
          SG = gen_SG(signal[0],pos)
          f.write(SG)

    f.close()


if __name__ == "__main__":
  if len(sys.argv) > 2:
    gen_dbc(sys.argv[1],sys.argv[2])
  else:
    print("Need two arguments: npy & dbc")
