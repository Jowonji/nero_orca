"""Rokoko glove → ``CombinedNeroOrcaSink`` pipeline (replaces the MediaPipe ingress + retargeter).

``run_rokoko`` mirrors ``orca_teleop.pipeline.run``: one worker thread turns glove frames into
``OrcaJointPositions`` on ``actions_q`` and the main thread is handed to ``sink.run_loop``.
No wrist positions are produced, so the Nero arm stays at its home pose; the hand moves and,
in pick-place mode, the Orca wrist follows the glove's hand tilt (``sink.update_hand_pitch``).
"""

from __future__ import annotations

import logging
import queue
import signal
import threading
import time

from collections.abc import Callable

from orca_teleop.pipeline import _SHUTDOWN, RobotSink, _install_stop_handler

from rokoko.retarget import RokokoHandRetargeter, hand_pitch
from rokoko.stream import DEFAULT_PORT, RokokoReceiver, frame_to_positions

logger = logging.getLogger(__name__)

QUEUE_MAXSIZE = 8
JOIN_TIMEOUT = 5.0


def rokoko_worker(
    receiver: RokokoReceiver,
    retargeter: RokokoHandRetargeter,
    actions_q: "queue.Queue[object]",
    stop_event: threading.Event,
    max_hz: float = 30.0,
    pitch_hook: Callable[[float], None] | None = None,
) -> None:
    """Poll the newest glove frame, retarget, and enqueue the hand command at ``max_hz``.

    ``pitch_hook`` (the sink's ``update_hand_pitch``) receives the hand tilt each frame so the
    Orca wrist can follow it in pick-place mode; the arm itself gets no position input.
    """
    period = 1.0 / max_hz
    n_cmd = 0
    last_log = time.monotonic()
    logger.info("Rokoko worker on :%d; hold the hand open for the first second", receiver.port)
    try:
        while not stop_event.is_set():
            t0 = time.monotonic()
            frame = receiver.latest()
            pos = frame_to_positions(frame) if frame is not None else None
            if pos is not None:
                if pitch_hook is not None:
                    pitch = hand_pitch(pos)
                    if pitch is not None:
                        try:
                            pitch_hook(pitch)
                        except Exception:
                            logger.debug("pitch_hook failed", exc_info=True)
                q = retargeter(pos)
                if q is not None:
                    # Keep only the newest command if the sink falls behind.
                    while True:
                        try:
                            actions_q.put_nowait(q)
                            break
                        except queue.Full:
                            try:
                                actions_q.get_nowait()
                            except queue.Empty:
                                pass
                    n_cmd += 1
            if time.monotonic() - last_log > 5.0:
                logger.info("rokoko: %d hand commands, %d packets", n_cmd, receiver.packets)
                last_log = time.monotonic()
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        try:
            actions_q.put_nowait(_SHUTDOWN)
        except queue.Full:
            pass


def run_rokoko(
    sink: RobotSink,
    urdf_path: str | None,
    model_path: str | None = None,
    port: int = DEFAULT_PORT,
    max_hz: float = 30.0,
    smoothing: float = 0.8,
    max_step_deg: float | None = 30.0,
    calibrate: bool = False,
    calib_file: str | None = None,
) -> None:
    """``calibrate`` runs the guided 3-pose calibration at start and saves it to ``calib_file``
    (default ``rokoko/calib/hand.json``), which later runs load automatically."""
    receiver = RokokoReceiver(port)  # bind first so a port clash fails before the slow connect
    actions_q: "queue.Queue[object]" = queue.Queue(maxsize=QUEUE_MAXSIZE)
    stop_event = threading.Event()
    previous_sigint = _install_stop_handler(stop_event)
    logger.info("Quit: Ctrl+C in this terminal, or close MuJoCo.")

    sink.connect()
    if model_path is None:
        model_path = getattr(sink, "retarget_model_path", None)
        if model_path:
            logger.info("Using sink-provided retargeter model: %s", model_path)
    kwargs = {"calibrate": calibrate}
    if calib_file:
        kwargs["calib_file"] = calib_file
    retargeter = RokokoHandRetargeter(
        model_path, urdf_path, smoothing=smoothing, max_step_deg=max_step_deg, **kwargs
    )

    thread = threading.Thread(
        target=rokoko_worker,
        args=(receiver, retargeter, actions_q, stop_event, max_hz),
        kwargs={"pitch_hook": getattr(sink, "update_hand_pitch", None)},
        name="rokoko",
        daemon=True,
    )
    thread.start()
    try:
        sink.run_loop(actions_q, stop_event)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        thread.join(timeout=JOIN_TIMEOUT)
        receiver.close()
        sink.close()
        signal.signal(signal.SIGINT, previous_sigint)
