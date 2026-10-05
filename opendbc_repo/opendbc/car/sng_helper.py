from time import monotonic
import cereal.messaging as messaging
from opendbc.car import DT_CTRL

SNG_DREL_MIN_M = 2
SNG_DELAY_MARGIN_S = 0.4
SNG_VLEAD_RESUME_THRESHOLD = 0.1  # m/s
SNG_LEAD_TOLERANCE_S = 0.2


class SngHelper:
  def __init__(self, initial_delay_s):
    self.initial_delay = round((initial_delay_s + SNG_DELAY_MARGIN_S) / DT_CTRL)
    self.ready_enter_delay = round(SNG_DELAY_MARGIN_S / DT_CTRL)
    self.repeat_interval = round((1 + SNG_DELAY_MARGIN_S) / DT_CTRL)
    self.next = 0
    self.entered = False
    self.lead_latched = False
    self.lead_gap_at = None
    self.radar_sm = None

  def lead_ready(self):
    self.radar_sm = self.radar_sm or messaging.SubMaster(["radarState"])
    self.radar_sm.update(0)
    if (lead := self.radar_sm["radarState"].leadOne).status:
      self.lead_gap_at = None
    elif self.lead_latched:
      now = monotonic()
      if self.lead_gap_at is None:
        self.lead_gap_at = now
      elif now - self.lead_gap_at >= SNG_LEAD_TOLERANCE_S:
        self.lead_latched = False
    return self.lead_latched and lead.status and lead.vLead > SNG_VLEAD_RESUME_THRESHOLD and lead.dRel > SNG_DREL_MIN_M

  def pressed(self, frame):
    self.next = frame + self.repeat_interval

  def tick(self, frame, active, gas, res, brake, should_resume, hold_complete=False, ready_on_enter=False):
    if not active:
      self.entered = self.lead_latched = False
      self.lead_gap_at = None
      return False
    if not self.entered:
      self.entered = self.lead_latched = True
      self.lead_gap_at = None
      self.next = frame + (self.ready_enter_delay if ready_on_enter else self.initial_delay)
      return None
    if gas or res or brake or hold_complete:
      self.next = max(self.next, frame + self.repeat_interval)
      return None
    return should_resume and frame > self.next
