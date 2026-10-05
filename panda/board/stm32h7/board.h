// ///////////////////////////////////////////////////////////// //
// Hardware abstraction layer for all different supported boards //
// ///////////////////////////////////////////////////////////// //
#include "board/boards/board_declarations.h"
#include "board/boards/unused_funcs.h"

// ///// Board definition and detection ///// //
#include "board/stm32h7/lladc.h"
#include "board/drivers/harness.h"
#include "board/drivers/fan.h"
#include "board/stm32h7/llfan.h"
#include "board/stm32h7/sound.h"
#include "board/drivers/fake_siren.h"
#include "board/drivers/clock_source.h"
#include "board/boards/red.h"
#include "board/boards/tres.h"
#include "board/boards/cuatro.h"
#include "board/boards/kedua.h"


void detect_board_type(void) {
  // Kedua has no ID strap; always detect as Kedua.
  hw_type = HW_TYPE_KEDUA;
  current_board = &board_kedua;
}
