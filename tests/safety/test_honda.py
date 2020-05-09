#!/usr/bin/env python3
import unittest
import numpy as np
from panda import Panda
from panda.tests.safety import libpandasafety_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda, make_msg, \
                                      MAX_WRONG_COUNTERS, UNSAFE_MODE

NIDEC_MAX_BRAKE = 255
BOSCH_MAX_BRAKE = 350
BOSCH_MAX_GAS = 2000
BOSCH_NO_GAS = -30000

INTERCEPTOR_THRESHOLD = 344
N_TX_MSGS = [[0xE4, 0], [0x194, 0], [0x1FA, 0], [0x200, 0], [0x30C, 0], [0x33D, 0]]
BH_TX_MSGS = [[0xE4, 0], [0xE4, 1], [0x1DF, 1], [0x1EF, 1], [0x296, 1], [0x30C, 1], [0x33D, 0], [0x33D, 1], [0x39F, 1]]  # Bosch Harness
BG_TX_MSGS = [[0xE4, 0], [0xE4, 2], [0x1DF, 0], [0x1EF, 0], [0x296, 0], [0x30C, 0], [0x33D, 0], [0x33D, 2], [0x39F, 0]]  # Bosch Giraffe

class Btn:
  CANCEL = 2
  SET = 3
  RESUME = 4

HONDA_N_HW = 0
HONDA_BG_HW = 1
HONDA_BH_HW = 2


class TestHondaSafety(common.PandaSafetyTest):
  cnt_speed = 0
  cnt_gas = 0
  cnt_button = 0

  PT_BUS = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHondaSafety":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  # override these inherited tests. honda doesn't use pcm enable
  def test_disable_control_allowed_from_cruise(self): pass
  def test_enable_control_allowed_from_cruise(self): pass
  def test_cruise_engaged_prev(self): pass

  def _speed_msg(self, speed):
    values = {"XMISSION_SPEED": speed, "COUNTER": self.cnt_speed % 4}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_panda("ENGINE_DATA", self.PT_BUS, values)

  def _button_msg(self, buttons):
    values = {"CRUISE_BUTTONS": buttons, "COUNTER": self.cnt_button % 4}
    self.__class__.cnt_button += 1
    return self.packer.make_can_msg_panda("SCM_BUTTONS", self.PT_BUS, values)

  def _brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake, "COUNTER": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_panda("POWERTRAIN_DATA", self.PT_BUS, values)

  def _gas_msg(self, gas):
    values = {"PEDAL_GAS": gas, "COUNTER": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_panda("POWERTRAIN_DATA", self.PT_BUS, values)

  def _send_brake_msg_nidec(self, brake):
    values = {}
    if self.safety.get_honda_hw() == HONDA_N_HW:
      values = {"COMPUTER_BRAKE": brake}
    return self.packer.make_can_msg_panda("BRAKE_COMMAND", 0, values)

  def _send_brake_msg_bosch(self, bus, sign, brake):
    to_send = make_msg(bus, 0x1DF)
    brake = (brake * sign) & 0x7FF
    to_send[0].RDLR = (brake & 0x7F8) << 21
    to_send[0].RDHR = (brake & 0x7) << 5
    # gas command in same message, send idle to pass safety check
    to_send[0].RDLR |= ((BOSCH_NO_GAS & 0xFF) << 8) | ((BOSCH_NO_GAS >> 8) & 0xFF)
    return to_send

  def _send_gas_msg_bosch(self, bus, gas):
    to_send = make_msg(bus, 0x1DF)
    to_send[0].RDLR = ((gas & 0xFF) << 8) | ((gas >> 8) & 0xFF)
    return to_send

  def _send_steer_msg(self, steer):
    values = {"STEER_TORQUE": steer}
    return self.packer.make_can_msg_panda("STEERING_CONTROL", 0, values)

  def test_resume_button(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg(Btn.RESUME))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_set_button(self):
    self.safety.set_controls_allowed(0)
    self._rx(self._button_msg(Btn.SET))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Btn.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_disengage_on_brake(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_unsafe_mode_no_disengage_on_gas(self):
    self.safety.safety_rx_hook(self._gas_msg(0))
    self.safety.set_controls_allowed(1)
    self.safety.set_unsafe_mode(UNSAFE_MODE.DISABLE_DISENGAGE_ON_GAS)
    self.safety.safety_rx_hook(self._gas_msg(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_unsafe_mode(UNSAFE_MODE.DEFAULT)

  def test_allow_engage_with_gas_pressed(self):
    self.safety.safety_rx_hook(self._gas_msg(1))
    self.safety.set_controls_allowed(1)
    self.safety.safety_rx_hook(self._gas_msg(1))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_disengage_on_gas_interceptor(self):
    for g in range(0, 0x1000):
      self.safety.safety_rx_hook(honda_interceptor_msg(0, 0x201))
      self.safety.set_controls_allowed(True)
      self.safety.safety_rx_hook(honda_interceptor_msg(g, 0x201))
      remain_enabled = g <= INTERCEPTOR_THRESHOLD
      self.assertEqual(remain_enabled, self.safety.get_controls_allowed())
      self.safety.safety_rx_hook(honda_interceptor_msg(0, 0x201))
      self.safety.set_gas_interceptor_detected(False)

  def test_unsafe_mode_no_disengage_on_gas_interceptor(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_unsafe_mode(UNSAFE_MODE.DISABLE_DISENGAGE_ON_GAS)
    for g in range(0, 0x1000):
      self.safety.safety_rx_hook(honda_interceptor_msg(g, 0x201))
      self.assertTrue(self.safety.get_controls_allowed())
      self.safety.safety_rx_hook(honda_interceptor_msg(0, 0x201))
      self.safety.set_gas_interceptor_detected(False)
    self.safety.set_unsafe_mode(UNSAFE_MODE.DEFAULT)
    self.safety.set_controls_allowed(False)

  def test_allow_engage_with_gas_interceptor_pressed(self):
    self.safety.safety_rx_hook(honda_interceptor_msg(0x1000, 0x201))
    self.safety.set_controls_allowed(1)
    self.safety.safety_rx_hook(honda_interceptor_msg(0x1000, 0x201))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(honda_interceptor_msg(0, 0x201))
    self.safety.set_gas_interceptor_detected(False)

  def test_brake_safety_check(self):
    hw = self.safety.get_honda_hw()
    bus_pt = 1 if hw == HONDA_BH_HW else 0
    send_brake_msg = self._send_brake_msg_nidec if hw == HONDA_N_HW else self._send_brake_msg_bosch
    brake_max = NIDEC_MAX_BRAKE if hw == HONDA_N_HW else BOSCH_MAX_BRAKE
    brake_sign = 1 if hw == HONDA_N_HW else -1

    for fwd_brake in [False, True]:
      if hw != HONDA_N_HW and fwd_brake:
        continue # can only forward AEB for nidec
      self.safety.set_honda_fwd_brake(fwd_brake)
      for brake in np.arange(0, brake_max + 10, 1):
        for controls_allowed in [True, False]:
          self.safety.set_controls_allowed(controls_allowed)
          if fwd_brake:
            send = False  # block openpilot brake msg when fwd'ing stock msg
          elif controls_allowed:
            send = brake_max >= brake >= 0
          else:
            send = brake == 0
          self.assertEqual(send, self.safety.safety_tx_hook(send_brake_msg(bus_pt, brake_sign, brake)))
      self.safety.set_honda_fwd_brake(False)

  def test_gas_safety_check(self):
    hw = self.safety.get_honda_hw()
    bus_pt = 1 if hw == HONDA_BH_HW else 0
    if hw != HONDA_N_HW:
      for controls_allowed in [True, False]:
        for gas in np.arange(BOSCH_NO_GAS, BOSCH_MAX_GAS + 2000, 100):
          self.safety.set_controls_allowed(controls_allowed)
          send = gas <= BOSCH_MAX_GAS if controls_allowed else gas == BOSCH_NO_GAS
          self.assertEqual(send, self.safety.safety_tx_hook(self._send_gas_msg_bosch(bus_pt, gas)))

  def test_gas_interceptor_safety_check(self):
    if self.safety.get_honda_hw() == HONDA_N_HW:
      for gas in np.arange(0, 4000, 100):
        for controls_allowed in [True, False]:
          self.safety.set_controls_allowed(controls_allowed)
          if controls_allowed:
            send = True
          else:
            send = gas == 0
          self.assertEqual(send, self.safety.safety_tx_hook(honda_interceptor_msg(gas, 0x200)))

  def test_steer_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._send_steer_msg(0x0000)))
    self.assertFalse(self._tx(self._send_steer_msg(0x1000)))

  def test_rx_hook(self):

    # TODO: move this test to common
    # checksum checks
    for msg in ["btn", "gas", "speed"]:
      self.safety.set_controls_allowed(1)
      # TODO: add this coverage back by re-running all tests with the acura dbc
      # to_push = self._button_msg(Btn.SET, 0x1A6)  # only in Honda_NIDEC
      if msg == "btn":
        to_push = self._button_msg(Btn.SET)
      if msg == "gas":
        to_push = self._gas_msg(0)
      if msg == "speed":
        to_push = self._speed_msg(0)
      self.assertTrue(self._rx(to_push))
      if msg != "btn":
        to_push[0].RDHR = 0  # invalidate checksum
        self.assertFalse(self._rx(to_push))
        self.assertFalse(self.safety.get_controls_allowed())

    # counter
    # reset wrong_counters to zero by sending valid messages
    for i in range(MAX_WRONG_COUNTERS + 1):
      self.__class__.cnt_speed += 1
      self.__class__.cnt_gas += 1
      self.__class__.cnt_button += 1
      if i < MAX_WRONG_COUNTERS:
        self.safety.set_controls_allowed(1)
        self._rx(self._button_msg(Btn.SET))
        self._rx(self._speed_msg(0))
        self._rx(self._gas_msg(0))
      else:
        self.assertFalse(self._rx(self._button_msg(Btn.SET)))
        self.assertFalse(self._rx(self._speed_msg(0)))
        self.assertFalse(self._rx(self._gas_msg(0)))
        self.assertFalse(self.safety.get_controls_allowed())

    # restore counters for future tests with a couple of good messages
    for i in range(2):
      self.safety.set_controls_allowed(1)
      self._rx(self._button_msg(Btn.SET))
      self._rx(self._speed_msg(0))
      self._rx(self._gas_msg(0))
    self._rx(self._button_msg(Btn.SET))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_hook_on_pedal_pressed(self):
    for mode in [UNSAFE_MODE.DEFAULT, UNSAFE_MODE.DISABLE_DISENGAGE_ON_GAS]:
      for pedal in ['brake', 'gas']:
        self.safety.set_unsafe_mode(mode)
        allow_ctrl = False
        if pedal == 'brake':
          # brake_pressed_prev and vehicle_moving
          self._rx(self._speed_msg(100))
          self._rx(self._brake_msg(1))
        elif pedal == 'gas':
          # gas_pressed_prev
          self._rx(self._gas_msg(1))
          allow_ctrl = mode == UNSAFE_MODE.DISABLE_DISENGAGE_ON_GAS

        self.safety.set_controls_allowed(1)
        hw = self.safety.get_honda_hw()
        if hw == HONDA_N_HW:
          self.safety.set_honda_fwd_brake(False)
          self.assertEqual(allow_ctrl, self._tx(self._send_brake_msg(MAX_BRAKE)))
        self.assertEqual(allow_ctrl, self._tx(self._send_steer_msg(0x1000)))

        # reset status
        self.safety.set_controls_allowed(0)
        self.safety.set_unsafe_mode(UNSAFE_MODE.DEFAULT)
        self._tx(self._send_brake_msg(0))
        self._tx(self._send_steer_msg(0))
        if pedal == 'brake':
          self._rx(self._speed_msg(0))
          self._rx(self._brake_msg(0))
        elif pedal == 'gas':
          self._rx(self._gas_msg(0))


class TestHondaNidecSafety(TestHondaSafety, common.InterceptorSafetyTest):

  TX_MSGS = [[0xE4, 0], [0x194, 0], [0x1FA, 0], [0x200, 0], [0x30C, 0], [0x33D, 0]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDR = 0xE4
  RELAY_MALFUNCTION_BUS = 0
  FWD_BLACKLISTED_ADDRS = {2: [0xE4, 0x194, 0x33D, 0x30C]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  INTERCEPTOR_THRESHOLD = 344

  def setUp(self):
    self.packer = CANPackerPanda("honda_civic_touring_2016_can_generated")
    self.safety = libpandasafety_py.libpandasafety
    self.safety.set_safety_hooks(Panda.SAFETY_HONDA_NIDEC, 0)
    self.safety.init_tests_honda()

  # Honda gas gains are the different
  def _interceptor_msg(self, gas, addr):
    to_send = make_msg(0, addr, 6)
    gas2 = gas * 2
    to_send[0].RDLR = ((gas & 0xff) << 8) | ((gas & 0xff00) >> 8) | \
                    ((gas2 & 0xff) << 24) | ((gas2 & 0xff00) << 8)
    return to_send

  def test_fwd_hook(self):
    # normal operation, not forwarding AEB
    self.FWD_BLACKLISTED_ADDRS[2].append(0x1FA)
    self.safety.set_honda_fwd_brake(False)
    super().test_fwd_hook()

    # TODO: test latching until AEB event is over?
    # forwarding AEB brake signal
    self.FWD_BLACKLISTED_ADDRS = {2: [0xE4, 0x194, 0x33D, 0x30C]}
    self.safety.set_honda_fwd_brake(True)
    super().test_fwd_hook()

      self.safety.set_controls_allowed(1)
      hw = self.safety.get_honda_hw()
      if hw == HONDA_N_HW:
        self.safety.set_honda_fwd_brake(False)
        self.assertFalse(self.safety.safety_tx_hook(self._send_brake_msg_nidec(0, 1, NIDEC_MAX_BRAKE)))
        self.assertFalse(self.safety.safety_tx_hook(interceptor_msg(INTERCEPTOR_THRESHOLD, 0x200)))
      self.assertFalse(self.safety.safety_tx_hook(self._send_steer_msg(0x1000)))

      # reset status
      self.safety.set_controls_allowed(0)
      self.safety.safety_tx_hook(self._send_brake_msg_nidec(0, 1, 0))
      self.safety.safety_tx_hook(self._send_steer_msg(0))
      self.safety.safety_tx_hook(honda_interceptor_msg(0, 0x200))
      if pedal == 'brake':
        self.safety.safety_rx_hook(self._speed_msg(0))
        self.safety.safety_rx_hook(self._brake_msg(0))
      elif pedal == 'gas':
        self.safety.safety_rx_hook(self._gas_msg(0))
      elif pedal == 'interceptor':
        self.safety.set_gas_interceptor_detected(False)

  def test_tx_hook_on_pedal_pressed_on_unsafe_gas_mode(self):
    for pedal in ['brake', 'gas', 'interceptor']:
      self.safety.set_unsafe_mode(UNSAFE_MODE.DISABLE_DISENGAGE_ON_GAS)
      if pedal == 'brake':
        # brake_pressed_prev and honda_moving
        self.safety.safety_rx_hook(self._speed_msg(100))
        self.safety.safety_rx_hook(self._brake_msg(1))
        allow_ctrl = False
      elif pedal == 'gas':
        # gas_pressed_prev
        self.safety.safety_rx_hook(self._gas_msg(1))
        allow_ctrl = True
      elif pedal == 'interceptor':
        # gas_interceptor_prev > INTERCEPTOR_THRESHOLD
        self.safety.safety_rx_hook(honda_interceptor_msg(INTERCEPTOR_THRESHOLD+1, 0x201))
        self.safety.safety_rx_hook(honda_interceptor_msg(INTERCEPTOR_THRESHOLD+1, 0x201))
        allow_ctrl = True

      self.safety.set_controls_allowed(1)
      hw = self.safety.get_honda_hw()
      if hw == HONDA_N_HW:
        self.safety.set_honda_fwd_brake(False)
        self.assertEqual(allow_ctrl, self.safety.safety_tx_hook(self._send_brake_msg_nidec(0, 1, NIDEC_MAX_BRAKE)))
        self.assertEqual(allow_ctrl, self.safety.safety_tx_hook(interceptor_msg(INTERCEPTOR_THRESHOLD, 0x200)))
      self.assertEqual(allow_ctrl, self.safety.safety_tx_hook(self._send_steer_msg(0x1000)))
      # reset status
      self.safety.set_controls_allowed(0)
      self.safety.set_unsafe_mode(UNSAFE_MODE.DEFAULT)
      self.safety.safety_tx_hook(self._send_brake_msg_nidec(0, 1, 0))
      self.safety.safety_tx_hook(self._send_steer_msg(0))
      self.safety.safety_tx_hook(honda_interceptor_msg(0, 0x200))
      if pedal == 'brake':
        self.safety.safety_rx_hook(self._speed_msg(0))
        self.safety.safety_rx_hook(self._brake_msg(0))
      elif pedal == 'gas':
        self.safety.safety_rx_hook(self._gas_msg(0))
      elif pedal == 'interceptor':
        self.safety.set_gas_interceptor_detected(False)

class TestHondaBoschGiraffeSafety(TestHondaSafety):
  @classmethod
  def setUp(cls):
    TestHondaSafety.setUp()
    cls.safety = libpandasafety_py.libpandasafety
    cls.safety.set_safety_hooks(Panda.SAFETY_HONDA_BOSCH_GIRAFFE, 0)
    cls.safety.init_tests_honda()


if __name__ == "__main__":
  unittest.main()
