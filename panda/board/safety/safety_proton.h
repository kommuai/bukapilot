const CanMsg PROTON_TX_MSGS[] = {{432, 0, 8}, {417, 0, 8}};

RxCheck proton_rx_checks[] = {
  //{.msg = {{0x35F, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},
};

static void proton_rx_hook(const CANPacket_t *to_push) {
  // proton is never at standstill
  vehicle_moving = true;
  controls_allowed = true;
  UNUSED(to_push);
}

static bool proton_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);
  UNUSED(addr);
  UNUSED(len);

  return tx;
}

static int proton_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  if (bus_num == 0) {
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    bool is_lkas_msg = ((addr == 432));// || (addr == 790));
    bool is_acc_msg = (addr == 417);
    bool block_msg = is_lkas_msg || is_acc_msg;
    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config proton_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(proton_rx_checks, PROTON_TX_MSGS);
}


const safety_hooks proton_hooks = {
  .init = proton_init,
  .rx = proton_rx_hook,
  .tx = proton_tx_hook,
  .fwd = proton_fwd_hook,
};
