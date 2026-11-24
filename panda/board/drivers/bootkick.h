#include "board/drivers/drivers.h"

bool bootkick_ign_prev = false;
BootState boot_state = BOOT_BOOTKICK;
uint8_t bootkick_harness_status_prev = HARNESS_STATUS_NC;
bool bootkick_reset_triggered = false;
uint8_t boot_counter = 0;

void bootkick_tick(bool ignition, bool recent_heartbeat) {
  const bool harness_inserted = (harness.status != bootkick_harness_status_prev) && (harness.status != HARNESS_STATUS_NC);
  boot_counter = (boot_counter < UINT8_MAX) ? boot_counter + 1 : UINT8_MAX;

  if (recent_heartbeat) {
    // disable bootkick once openpilot is up
    boot_state = BOOT_STANDBY;
  } else if ((ignition && !bootkick_ign_prev) || harness_inserted) {
    // bootkick on rising edge of ignition or harness insertion
    // dont allow BOOTKICK for the first 25s
    if (boot_counter > 25) boot_state = BOOT_BOOTKICK;
  } else {

  }

  // update state
  bootkick_ign_prev = ignition;
  bootkick_harness_status_prev = harness.status;
  current_board->set_bootkick(boot_state);
}
