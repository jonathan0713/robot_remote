"""Meta Quest controllers through NVIDIA Isaac Teleop + CloudXR (no ROS, no robot).

QuestController.read_all() -> {hand: (tracked, pos[3], quat xyzw, squeeze, trigger)} in the operator frame
(X forward, Y left, Z up); .buttons -> {hand: (primary A/X, secondary B/Y)}. Runs in .venv_quest.
"""

from pathlib import Path

import numpy as np

# OpenXR anchor (X right, Y up, Z backward) -> operator frame (X forward, Y left, Z up).
OPERATOR_T_ANCHOR = np.array(
    [[0.0, 0.0, -1.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=np.float32
)



# ----------------------------------------------------------------------------- XR input
class QuestController:
    """Raw grip pose / squeeze / trigger of Quest controllers via Isaac Teleop + CloudXR.

    One CloudXR runtime + one TeleopSession per process (the runtime is a process-wide singleton),
    so both hands must be read through the same instance."""

    def __init__(self, hands, env_file: Path, app_name: str = "QuestReBeL"):
        from isaacteleop.cloudxr import CloudXRLauncher
        from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
        from isaacteleop.retargeting_engine.interface import (
            ExecutionEvents, ExecutionState, OutputCombiner, TensorGroup, ValueInput,
        )
        from isaacteleop.retargeting_engine.tensor_types import TransformMatrix
        from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex
        from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig

        self._idx = ControllerInputIndex
        print("[quest] launching CloudXR runtime (~30 s) ...", flush=True)
        self._launcher = CloudXRLauncher(install_dir=str(Path.home() / ".cloudxr"), env_config=str(env_file))
        self.hands = [hands] if isinstance(hands, str) else list(hands)
        self.buttons = {h: (False, False) for h in self.hands}
        controllers = ControllersSource(name="controllers")
        xform = ValueInput("operator_T_anchor", TransformMatrix())
        transformed = controllers.transformed(xform.output("value"))
        pipeline = OutputCombiner({h: transformed.output(f"controller_{h}") for h in self.hands})
        tg = TensorGroup(TransformMatrix())
        tg[0] = OPERATOR_T_ANCHOR
        self._inputs = {"operator_T_anchor": {"value": tg}}
        self._events = ExecutionEvents(execution_state=ExecutionState.RUNNING, reset=False)
        self._session = TeleopSession(TeleopSessionConfig(app_name=app_name, pipeline=pipeline))
        self._session.__enter__()

    def read_all(self) -> dict:
        """Step the session once -> {hand: (tracked, pos[3] operator frame, quat xyzw, squeeze, trigger)}"""
        out = self._session.step(execution_events=self._events, external_inputs=self._inputs)
        self.buttons = {h: self._buttons(out[h]) for h in self.hands}  # (primary A/X, secondary B/Y; dual-arm teleop: Y = scene reset)
        return {h: self._parse(out[h]) for h in self.hands}

    def _buttons(self, c):
        if getattr(c, "is_none", False):
            return False, False
        try:
            return float(c[self._idx.PRIMARY_CLICK]) > 0.5, float(c[self._idx.SECONDARY_CLICK]) > 0.5
        except (IndexError, KeyError, TypeError, ValueError):
            return False, False

    def read(self):
        """Single-hand convenience: the first hand's tuple."""
        return self.read_all()[self.hands[0]]

    def _parse(self, c):
        if getattr(c, "is_none", False):
            return False, None, None, 0.0, 0.0
        try:
            pos = np.asarray(c[self._idx.GRIP_POSITION], dtype=float)
            quat = np.asarray(c[self._idx.GRIP_ORIENTATION], dtype=float)
            squeeze, trigger = float(c[self._idx.SQUEEZE_VALUE]), float(c[self._idx.TRIGGER_VALUE])
        except (IndexError, KeyError, TypeError, ValueError):
            return False, None, None, 0.0, 0.0
        # Right after the headset connects (and on tracking loss) the stream can carry an all-zero
        # quaternion / non-finite pose. Treat that as "not tracked" instead of crashing.
        if quat.shape != (4,) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-6 \
                or pos.shape != (3,) or not np.all(np.isfinite(pos)):
            return False, None, None, 0.0, 0.0
        return True, pos, quat / np.linalg.norm(quat), squeeze, trigger

    def close(self):
        try:
            self._session.__exit__(None, None, None)
        finally:
            self._launcher.stop()
