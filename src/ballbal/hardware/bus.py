"""Robust half-duplex serial bus for Feetech STS servos.

The Feetech SDK is a thin, unforgiving protocol layer: one dropped status
packet turns into a failure code, and the caller is expected to cope. On a
half-duplex 1 Mbaud bus, packets *do* get dropped, especially while a motor is
drawing current. This module adds the three things that make the difference
between a script that runs for hours and one that dies on the first glitch:

* retries with an input-buffer reset between attempts, so a partial reply
  cannot desync every packet that follows it;
* sync read / sync write, so N servos cost one bus transaction instead of N;
* a torque guard, so torque is never left enabled by an exception, a Ctrl-C, or
  a SIGTERM.
"""

from __future__ import annotations

import fcntl
import functools
import logging
import signal
import termios
import time
from contextlib import contextmanager
from dataclasses import dataclass
from types import TracebackType
from typing import Iterable, Iterator, Sequence

import serial

from .vendor.scservo_sdk import (  # type: ignore[attr-defined]
    COMM_SUCCESS,
    GroupSyncRead,
    PortHandler,
    sms_sts,
)

from . import registers as reg

logger = logging.getLogger(__name__)

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_RETRIES = 3
SERIAL_READ_TIMEOUT_S = 0.005
"""Small blocking read timeout.

The SDK opens the port non-blocking (``timeout=0``), which makes its receive
loop spin at 100% CPU for the whole 50 ms packet timeout whenever a reply is
late. A few milliseconds of blocking read costs nothing on the success path
(``read`` returns as soon as the requested bytes arrive) and turns that spin
into a sleep.
"""


class BusError(RuntimeError):
    """A servo transaction failed after every retry."""


def _wrap_serial_errors(operation: str):  # noqa: ANN201 - decorator factory
    """Turn a pyserial failure into a BusError.

    An unplugged adapter surfaces as SerialException from deep inside the SDK.
    Letting it through means a stack trace instead of a sentence, and it slips
    past every `except BusError` the callers already have -- including the ones
    that release torque.

    termios.error has to be caught by name: it derives straight from Exception,
    not from OSError, so it slips through a tuple that only names the latter.
    An adapter that re-enumerates mid-session -- which is what a servo supply
    being switched while the port is open looks like -- leaves a stale file
    descriptor whose next tcflush raises exactly that.
    """

    def decorate(function):  # noqa: ANN001, ANN202
        @functools.wraps(function)
        def wrapper(*args, **kwargs):  # noqa: ANN002, ANN003, ANN202
            try:
                return function(*args, **kwargs)
            except (serial.SerialException, OSError, termios.error) as exc:
                raise BusError(
                    f"{operation} failed: the serial link dropped ({exc}). "
                    "Check the USB cable and the servo supply. If the servo "
                    "supply was switched while this was running, the adapter "
                    "has re-enumerated and the port must be reopened: run the "
                    "command again."
                ) from exc

        return wrapper

    return decorate


class ServoError(RuntimeError):
    """A servo answered, but reported an error in its status packet."""


@dataclass(frozen=True)
class Telemetry:
    servo_id: int
    model: int
    position: int
    speed: int
    load: int
    voltage: float
    temperature: int
    current: int
    status: int
    mode: int
    torque_enabled: bool
    min_angle_limit: int
    max_angle_limit: int
    min_voltage_limit: float
    max_voltage_limit: float

    @property
    def degrees(self) -> float:
        return reg.counts_to_degrees(self.position)

    @property
    def status_text(self) -> str:
        return reg.describe_status(self.status)


class ServoBus:
    """A connection to one Feetech servo chain."""

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        retries: int = DEFAULT_RETRIES,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.retries = retries
        self._handler = PortHandler(port)
        self._sdk = sms_sts(self._handler)
        self._energised: set[int] = set()
        """Servos that may be holding torque because of something we sent.

        Not just explicit torque-enable writes: on these servos, writing a goal
        position sets TORQUE_ENABLE by itself (measured: the register reads 0
        before a goal write and 1 after). Tracking only explicit enables would
        leave a servo energised at close() simply because it was given a goal.
        """
        self._sync_readers: dict[tuple[int, int], GroupSyncRead] = {}
        self._limits: dict[int, tuple[int, int]] = {}
        """Travel limits, enforced on every goal that leaves this object.

        They used to live only in Rig.goto, which meant any caller reaching the
        bus directly -- a diagnostic script, a REPL session -- could command a
        goal past the mechanism's end stops with nothing to stop it. That is not
        a hypothetical: it is how this project drove an axis into its stop.
        """

        if not self._handler.setBaudRate(baudrate):
            raise BusError(
                f"{baudrate} baud is not a supported rate for {port}. "
                "STS servos ship at 1000000."
            )
        self._claim_port()
        # The SDK's clearPort() calls pyserial's flush(), which drains the
        # *write* buffer and leaves stale input bytes in place. Every
        # transaction starts with it, so this is exactly the right hook for
        # discarding the tail of a reply we gave up waiting for.
        self._handler.clearPort = self._handler.ser.reset_input_buffer
        self._handler.ser.timeout = SERIAL_READ_TIMEOUT_S

    def _claim_port(self) -> None:
        """Take an exclusive lock on the serial device.

        Nothing in the OS stops two processes opening the same tty, and the SDK
        does not lock either. Two of them on a half-duplex bus interleave
        packets, so replies land in the wrong reader -- and worse, a second
        process can command motion while the first sits at an interactive
        prompt, which is how a `recover` run once had its servos moved out from
        under it by a concurrent `home`.

        The lock is advisory, which is enough: every process that drives this
        bus takes it.
        """
        try:
            fcntl.flock(self._handler.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._handler.closePort()
            raise BusError(
                f"{self.port} is already in use by another ballbal process. "
                "Only one may drive the bus at a time -- close the other one "
                "(an interactive jog or calibrate session counts) and retry."
            ) from exc

    # -- lifecycle ----------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return bool(self._handler.is_open)

    def close(self) -> None:
        """Release anything we may have energised, then close the port."""
        try:
            if self._energised:
                self.set_torque(sorted(self._energised), False)
        finally:
            if self._handler.is_open:
                self._handler.closePort()

    def __enter__(self) -> "ServoBus":
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    # -- primitives ---------------------------------------------------------

    def _check(self, what: str, servo_id: int, error: int) -> None:
        if error:
            raise ServoError(
                f"servo {servo_id} reported {reg.describe_status(error)} during {what}"
            )

    @_wrap_serial_errors("ping")
    def ping(self, servo_id: int, *, retries: int | None = None) -> int | None:
        """Return the model number, or ``None`` if the servo does not answer."""
        attempts = self.retries if retries is None else retries
        for _ in range(attempts + 1):
            model, comm, error = self._sdk.ping(servo_id)
            if comm == COMM_SUCCESS and not error:
                return model
            self._handler.ser.reset_input_buffer()
        return None

    @_wrap_serial_errors("read")
    def read(self, servo_id: int, register: reg.Register) -> int:
        """Read one register, retrying on communication failure."""
        reader = {
            1: self._sdk.read1ByteTxRx,
            2: self._sdk.read2ByteTxRx,
            4: self._sdk.read4ByteTxRx,
        }[register.size]
        comm = None
        for attempt in range(self.retries + 1):
            value, comm, error = reader(servo_id, register.address)
            if comm == COMM_SUCCESS:
                self._check(f"read {register.name}", servo_id, error)
                return value
            logger.debug(
                "read %s from servo %d failed (attempt %d): %s",
                register.name,
                servo_id,
                attempt + 1,
                self._sdk.getTxRxResult(comm),
            )
            self._handler.ser.reset_input_buffer()
        raise BusError(
            f"read {register.name} from servo {servo_id} failed after "
            f"{self.retries + 1} attempts: {self._sdk.getTxRxResult(comm)}"
        )

    @_wrap_serial_errors("write")
    def write(
        self,
        servo_id: int,
        register: reg.Register,
        value: int,
        *,
        ignore_servo_error: bool = False,
    ) -> None:
        """Write one register, retrying, then confirming by read-back.

        A lost *status* packet is indistinguishable from a lost *instruction*
        packet at this layer, and the servo may well have applied the write
        either way. Reading the register back is the only way to tell, and it
        matters most for torque enable: retrying blindly there can leave torque
        on when the caller believes it failed.
        """
        writer = {
            1: self._sdk.write1ByteTxRx,
            2: self._sdk.write2ByteTxRx,
            4: self._sdk.write4ByteTxRx,
        }[register.size]
        comm = None
        for attempt in range(self.retries + 1):
            comm, error = writer(servo_id, register.address, value)
            if comm == COMM_SUCCESS:
                if not ignore_servo_error:
                    self._check(f"write {register.name}", servo_id, error)
                return
            self._handler.ser.reset_input_buffer()
            try:
                if self.read(servo_id, register) == value:
                    logger.debug(
                        "write %s=%d on servo %d confirmed by read-back after a "
                        "lost reply",
                        register.name,
                        value,
                        servo_id,
                    )
                    return
            except (BusError, ServoError):
                # A faulted servo answers every packet with its error flag set.
                # That says nothing about whether this write landed.
                pass
            logger.debug(
                "write %s=%d on servo %d failed (attempt %d): %s",
                register.name,
                value,
                servo_id,
                attempt + 1,
                self._sdk.getTxRxResult(comm),
            )
            time.sleep(0.02)
        raise BusError(
            f"write {register.name}={value} on servo {servo_id} failed after "
            f"{self.retries + 1} attempts: {self._sdk.getTxRxResult(comm)}"
        )

    # -- bulk transfers -----------------------------------------------------

    @_wrap_serial_errors("sync read")
    def read_positions(self, servo_ids: Sequence[int]) -> dict[int, int]:
        """Read every servo's position in a single bus transaction."""
        if not servo_ids:
            return {}
        reader = self._sync_reader(reg.PRESENT_POSITION)
        for attempt in range(self.retries + 1):
            reader.clearParam()
            for servo_id in servo_ids:
                reader.addParam(servo_id)
            if reader.txRxPacket() == COMM_SUCCESS and all(
                reader.isAvailable(
                    servo_id, reg.PRESENT_POSITION.address, reg.PRESENT_POSITION.size
                )[0]
                for servo_id in servo_ids
            ):
                return {
                    servo_id: reader.getData(
                        servo_id,
                        reg.PRESENT_POSITION.address,
                        reg.PRESENT_POSITION.size,
                    )
                    for servo_id in servo_ids
                }
            logger.debug("sync read of positions failed (attempt %d)", attempt + 1)
            self._handler.ser.reset_input_buffer()
        # Sync read is a broadcast: one silent servo fails the whole batch, so
        # fall back to individual reads, which name the servo that is missing.
        return {sid: self.read(sid, reg.PRESENT_POSITION) for sid in servo_ids}

    def _sync_reader(self, register: reg.Register) -> GroupSyncRead:
        key = (register.address, register.size)
        if key not in self._sync_readers:
            self._sync_readers[key] = GroupSyncRead(
                self._sdk, register.address, register.size
            )
        return self._sync_readers[key]

    @_wrap_serial_errors("goal write")
    def write_goals(self, goals: dict[int, int], speed: int, acceleration: int) -> None:
        """Command every servo's goal position in a single broadcast.

        Sync write is unacknowledged by design, which is what makes it fast
        enough for a control loop -- and why callers should confirm motion by
        reading positions back rather than trusting the write.
        """
        if not goals:
            return
        if self._limits:
            outside = {
                servo_id: (position, self._limits[servo_id])
                for servo_id, position in goals.items()
                if servo_id in self._limits
                and not self._limits[servo_id][0]
                <= position
                <= self._limits[servo_id][1]
            }
            if outside:
                # Raise rather than clamp. Callers that legitimately need
                # clamping (a control loop chasing a setpoint) go through
                # Rig.goto, which clamps first; anything arriving here out of
                # range is a bug, and silently correcting it hides the bug.
                detail = "; ".join(
                    f"servo {sid}: {pos} outside {low}..{high}"
                    for sid, (pos, (low, high)) in outside.items()
                )
                raise BusError(f"goal outside configured travel -- {detail}")
        writer = self._sdk.groupSyncWrite
        writer.clearParam()
        for servo_id, position in goals.items():
            if not self._sdk.SyncWritePosEx(servo_id, position, speed, acceleration):
                raise BusError(f"could not queue a goal for servo {servo_id}")
        # Record before sending: a goal write energises the servo, and it does
        # so even if the acknowledgement never comes back to us.
        self._energised.update(goals)
        comm = writer.txPacket()
        writer.clearParam()
        if comm != COMM_SUCCESS:
            raise BusError(
                f"sync write of {len(goals)} goals failed: "
                f"{self._sdk.getTxRxResult(comm)}"
            )

    # -- EPROM --------------------------------------------------------------

    def write_eprom(self, servo_id: int, register: reg.Register, value: int) -> None:
        """Write a persistent (EPROM) register, unlocking and relocking around it.

        EPROM registers ignore writes while the servo's lock byte is set, and
        they have a finite number of erase cycles, so this is deliberately not
        something the motion path ever touches.
        """
        encoded = reg.encode_signed(value, register)
        self.write(servo_id, reg.LOCK, 0)
        try:
            self.write(servo_id, register, encoded)
            written = self.read(servo_id, register)
            if written != encoded:
                raise BusError(
                    f"servo {servo_id}: {register.name} read back as {written} "
                    f"after writing {encoded}"
                )
        finally:
            self.write(servo_id, reg.LOCK, 1)

    def restore_factory_settings(self, servo_id: int) -> list[tuple[str, int, int]]:
        """Undo role-specific torque caps and gains. Returns what changed."""
        changed: list[tuple[str, int, int]] = []
        for register, factory in reg.FACTORY_SETTINGS:
            present = self.read(servo_id, register)
            if present != factory:
                self.write_eprom(servo_id, register, factory)
                changed.append((register.name, present, factory))
        return changed

    @_wrap_serial_errors("id write")
    def set_servo_id(self, current_id: int, new_id: int) -> None:
        """Change a servo's ID.

        The ID lives in EPROM, so this survives a power cycle -- which is the
        point, and also why only one servo may be on the bus when it runs.
        Writing an ID another servo already answers to leaves two responding to
        every packet sent to it, and neither can be told apart or corrected
        afterwards without unplugging one.

        This cannot go through :pymeth:`write_eprom`: the servo starts answering
        to the new address the instant the write lands, so the read-back and the
        re-lock have to be addressed to the new ID, not the old one. Nor can the
        write be verified by its acknowledgement -- a lost status packet says
        nothing about whether the ID changed. Pinging both addresses does.
        """
        if not 1 <= new_id <= 253:
            raise ValueError(f"servo id {new_id} is outside 1..253")
        if current_id == new_id:
            return

        self.write(current_id, reg.TORQUE_ENABLE, 0, ignore_servo_error=True)
        self._energised.discard(current_id)
        self.write(current_id, reg.LOCK, 0)

        self._sdk.write1ByteTxRx(current_id, reg.ID.address, new_id)
        self._handler.ser.reset_input_buffer()

        if self.ping(new_id) is None:
            # Nothing at the new address. Re-lock the old one so the servo is
            # not left with its EPROM writable.
            if self.ping(current_id) is not None:
                self.write(current_id, reg.LOCK, 1)
                raise BusError(
                    f"servo {current_id} did not take id {new_id}; it still "
                    f"answers to {current_id} and has been re-locked"
                )
            raise BusError(
                f"after writing id {new_id}, neither {current_id} nor {new_id} "
                "answers. Power-cycle the servo and re-scan before retrying."
            )

        self.write(new_id, reg.LOCK, 1)
        confirmed = self.read(new_id, reg.ID)
        if confirmed != new_id:
            raise BusError(
                f"servo answered at {new_id} but its id register reads "
                f"{confirmed}"
            )

    def factory_reset_travel(self, servo_id: int) -> None:
        """Clear the homing offset and open the angle limits to a full turn.

        This is what removes another project's calibration from a reused servo.
        The offset is cleared first: it shifts every position the servo reports,
        so leaving it in place would make the new limits mean something else.
        """
        self.write(servo_id, reg.TORQUE_ENABLE, 0)
        self._energised.discard(servo_id)
        self.write_eprom(servo_id, reg.OFFSET, reg.FACTORY_OFFSET)
        self.write_eprom(servo_id, reg.MIN_ANGLE_LIMIT, reg.FACTORY_MIN_ANGLE_LIMIT)
        self.write_eprom(servo_id, reg.MAX_ANGLE_LIMIT, reg.FACTORY_MAX_ANGLE_LIMIT)

    def set_limits(self, limits: dict[int, tuple[int, int]]) -> None:
        """Install per-servo travel limits enforced on every goal write."""
        for servo_id, (low, high) in limits.items():
            if low > high:
                raise ValueError(f"servo {servo_id}: limits {low}..{high} inverted")
        self._limits = dict(limits)

    # -- higher level -------------------------------------------------------

    def telemetry(self, servo_id: int) -> Telemetry:
        model = self.ping(servo_id)
        if model is None:
            raise BusError(f"servo {servo_id} did not answer a ping")
        return Telemetry(
            servo_id=servo_id,
            model=model,
            position=self.read(servo_id, reg.PRESENT_POSITION),
            speed=self.read_signed(servo_id, reg.PRESENT_SPEED),
            load=self.read_signed(servo_id, reg.PRESENT_LOAD),
            voltage=self.read(servo_id, reg.PRESENT_VOLTAGE) / 10.0,
            temperature=self.read(servo_id, reg.PRESENT_TEMPERATURE),
            current=self.read(servo_id, reg.PRESENT_CURRENT),
            status=self.read(servo_id, reg.STATUS),
            mode=self.read(servo_id, reg.MODE),
            torque_enabled=bool(self.read(servo_id, reg.TORQUE_ENABLE)),
            min_angle_limit=self.read(servo_id, reg.MIN_ANGLE_LIMIT),
            max_angle_limit=self.read(servo_id, reg.MAX_ANGLE_LIMIT),
            min_voltage_limit=self.read(servo_id, reg.MIN_VOLTAGE_LIMIT) / 10.0,
            max_voltage_limit=self.read(servo_id, reg.MAX_VOLTAGE_LIMIT) / 10.0,
        )

    def read_signed(self, servo_id: int, register: reg.Register) -> int:
        """Read a register whose value is sign-magnitude encoded."""
        return reg.decode_signed(self.read(servo_id, register), register)

    def voltage(self, servo_id: int) -> float:
        return self.read(servo_id, reg.PRESENT_VOLTAGE) / 10.0

    def set_torque(self, servo_ids: Iterable[int], enabled: bool) -> None:
        """Enable or disable torque.

        Disabling ignores the servo's error flag. A servo in a fault state --
        the voltage error that hard braking provokes, say -- sets that flag on
        every packet it answers, and treating it as a failed write would refuse
        to release the motor at exactly the moment it needs releasing. Enabling
        still checks: energising a faulted servo is not something to do quietly.
        """
        errors: list[str] = []
        for servo_id in servo_ids:
            try:
                self.write(
                    servo_id,
                    reg.TORQUE_ENABLE,
                    1 if enabled else 0,
                    ignore_servo_error=not enabled,
                )
            except (BusError, ServoError) as exc:
                # Keep going: leaving torque on elsewhere is the worse failure.
                errors.append(str(exc))
                continue
            if enabled:
                self._energised.add(servo_id)
            else:
                self._energised.discard(servo_id)
        if errors:
            raise BusError("; ".join(errors))

    @contextmanager
    def torque(
        self, servo_ids: Sequence[int], *, hold: bool = False
    ) -> Iterator[None]:
        """Enable torque for the duration of the block, then release it.

        Torque comes off on the way out no matter how the block ends: normal
        return, exception, Ctrl-C, or SIGTERM. Pass ``hold=True`` to leave the
        servos energised on a clean exit (they are still released on a fault).
        """
        clean_exit = False
        previous_handlers: dict[int, object] = {}

        def _stop(signum: int, _frame: object) -> None:
            raise KeyboardInterrupt(f"signal {signum}")

        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            try:
                previous_handlers[sig] = signal.getsignal(sig)
                signal.signal(sig, _stop)
            except (ValueError, OSError):  # not on the main thread
                previous_handlers.pop(sig, None)

        try:
            self.set_torque(servo_ids, True)
            yield
            clean_exit = True
        finally:
            try:
                if not (clean_exit and hold):
                    self.set_torque(servo_ids, False)
                elif hold:
                    self._energised.difference_update(servo_ids)
            finally:
                for sig, handler in previous_handlers.items():
                    signal.signal(sig, handler)  # type: ignore[arg-type]

    def scan(self, servo_ids: Iterable[int]) -> dict[int, int]:
        """Ping a range of IDs and return ``{id: model number}`` for responders."""
        found: dict[int, int] = {}
        for servo_id in servo_ids:
            model = self.ping(servo_id, retries=0)
            if model is not None:
                found[servo_id] = model
        return found
