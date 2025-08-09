const CanMsg DNGA_TX_MSGS[] = {{464, 0, 8}, {628, 0, 8}, {625, 0, 8}, {627, 0, 8}};

RxCheck dnga_rx_checks[] = {
  //{.msg = {{0x35F, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},
};

static void dnga_rx_hook(const CANPacket_t *to_push) {
  // dnga is never at standstill
  vehicle_moving = true;
  controls_allowed = true;
  UNUSED(to_push);
}

static bool dnga_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);
  UNUSED(addr);
  UNUSED(len);

  return tx;
}

static int dnga_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  if (bus_num == 0) {
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    bool is_lkas_msg = ((addr == 464) || (addr == 628));
    bool is_acc_msg = ((addr == 625) || (addr == 627));
    bool block_msg = is_lkas_msg || is_acc_msg;
    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config dnga_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}


const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
  .fwd = dnga_fwd_hook,
};
