from enum import IntEnum


class PrimeType(IntEnum):
  UNKNOWN = -2
  UNPAIRED = -1
  NONE = 0
  MAGENTA = 1
  LITE = 2
  BLUE = 3
  MAGENTA_NEW = 4
  PURPLE = 5


class PrimeState:
  def __init__(self):
    self.prime_type = PrimeType.NONE

  def _load_initial_state(self) -> PrimeType:
    return PrimeType.NONE

  def _fetch_prime_status(self) -> None:
    self.set_type(PrimeType.NONE)

  def set_type(self, prime_type: PrimeType) -> None:
    self.prime_type = PrimeType.NONE

  def start(self) -> None:
    pass

  def stop(self) -> None:
    pass

  def get_type(self) -> PrimeType:
    return self.prime_type

  def is_prime(self) -> bool:
    return False

  def is_paired(self) -> bool:
    return False
