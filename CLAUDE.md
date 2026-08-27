# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Controller software for a Westwood Robotics BEAR actuator that provides assistive torque for a knee/elbow joint in "Eksovest", a semi-active exoskeleton vest. The actuator is driven from a Raspberry Pi over serial (`pybear` SDK). The core control law is a physics model (gravity compensation minus a characterized spring, blended in over a velocity band) — it does not use a generic PID/RL approach, and its constants come from bench measurement campaigns, not guesses.

## Safety model — read before touching control constants

This code commands current into a motor that can have a person's arm in the loop. Treat any change to `main_controller/controller.py` or `main_controller/config.py` as safety-relevant.

- `config.SAFETY_CHECKS_CONFIRMED` gates the entry point (`main_controller/main.py`). If `False`, it refuses to run and prints a checklist (ESTOP wiring traced, bench fixture in place, real tunables not placeholders). Don't flip it to `True` on someone's behalf — it means a human has verified those things.
- Hard limits enforced in `bear_interface.py`: `MAX_IQ` (absolute current clamp), `joint_limit_scale` (zeroes torque at/beyond `MIN_ANGLE`/`MAX_ANGLE`, stays enabled/backdrivable), `thermal_scale` (soft-scales torque from `TEMP_WARN` to `TEMP_MAX`, zero above `TEMP_MAX`).
- `controller.py`'s rate limiter (`MAX_TAU_RATE`) prevents the command from stepping instantly to full torque on engagement — an earlier live run without this jerked the wearer's arm and contributed to a mounting-flange failure. It only applies inside `compute()`; emergency stops must bypass it and write `iq = 0` directly.
- `ARM_MASS_KG = 0.0` / `ARM_COM_M = 0.0` are deliberate fail-safe defaults, not unset placeholders: with zero arm mass the residual torque is negative everywhere, so the model commands zero torque on a bare rig by construction. Never default these to a nonzero "typical" value.
- `ASSIST_RATIO`, `FRICTION_COMP_RATIO`, and the velocity blend band (`V_ON`/`V_OFF`/`V_HI`) are marked in-code as needing PI sign-off — they aren't free tuning knobs, they set how much load comes off the wearer and how the system avoids self-driving instability. Don't change them without that context (see the module docstring and inline comments in `controller.py`, which explain the reasoning for every term in detail).
- `bear_interface.configure_watchdog()` deliberately raises `NotImplementedError` rather than guessing at an unconfirmed firmware register — don't fill it in without checking the Westwood SDK manual against the actual register.

## Commands

There is no build step, linter, or automated test suite (no requirements.txt, pytest/unittest, or CI config). Verification is done by running scripts against hardware or logged data:

- **Dry-run the control law with no hardware**: `python -m main_controller.controller` — prints the command curve across the reachable range for bare-rig and with-arm cases, plus sanity checks (bare rig must command zero everywhere, worst measured incidental velocity must command zero, no `iq` may exceed `LIMIT_I_MAX`).
- **Check actuator connectivity** (read-only, safe with ESTOP off): `python main_controller/test_connection.py`.
- **Main entry point**: `python -m main_controller.main` (or `python main_controller/main.py` — the package is installed editable in `venv/`, so imports resolve from any cwd). Note: this currently calls `AssistController.start()`, which is not yet implemented on the class in `controller.py` — the entry point is a stub, not a working runtime loop.
- **Bench characterization / live-test scripts** in `useful_tools/` are run standalone, typically in this order: spring/friction characterization (`Bidirectional_sweep.py`, `spring_characterization.py`, `position_hold_characterization.py`, `extract_spring.py`) → velocity-threshold tuning (`Velocity_profile.py`, `Analyse_velocity_profile.py`) → open-loop model validation (`validate_model.py`) → gated human testing (`run_assist_human.py --monitor` logs what would be commanded at zero torque; `--live` actually applies torque and requires separate PI approval) → post-hoc diagnostics over the resulting CSVs (`analyze_assist_run.py`, `analyze_human_run.py`).
- `test_examples/` (`first_motion_test.py`, `setpos.py`) are interactive Westwood-SDK-style hardware demos, not automated tests.

## Architecture

- **`main_controller/controller.py`** — the control law. Explicitly hardware-free ("PURE — no hardware imports, cannot actuate"), so it's unit-testable by just calling `compute()`. `AssistController` is stateless except for a latch (Schmitt trigger, hysteresis between `V_ON`/`V_OFF` to avoid chatter) and a rate limiter. Formula: `tau_cmd = w(theta_dot) * [alpha * max(0, tau_residual) + k_f * tau_friction]`, where `tau_residual = tau_gravity_total(theta_vest) - tau_spring(theta_vest)` and `w()` is a smoothstep blend over velocity, not a hard threshold. `tau_spring()` interpolates over `TAU_SPRING_TABLE`, a table measured via bidirectional sweep (friction-cancelled by averaging both directions); it returns `None` outside the characterized range and callers must treat that as "do not assist," never extrapolate. Read the module docstring before changing any constant — it documents why each term is shaped the way it is and what the model does *not* account for (viscous friction, unverified `KT`, single-spring-configuration characterization).
- **`main_controller/config.py`** — hardware/comm settings (port, baudrate, actuator ID), joint angle limits, safety limits (`MAX_IQ`, thermal thresholds), the `SAFETY_CHECKS_CONFIRMED` gate, and `LOG_DIR` (`<repo>/logs`).
- **`main_controller/bear_interface.py`** — the only module that talks to hardware, wrapping `pybear.Manager.BEAR`. Provides `enable()`/`enable_position_mode()`/`disable()` (torque-IQ vs. position mode), `set_iq()` (clamped, returns the actually-applied value — always track that return value rather than your own running total), `get_state()` (read-only, safe regardless of ESTOP), and the joint-limit/thermal soft-scaling described above.
- **`useful_tools/`** — standalone scripts for characterization, validation, and analysis (not part of the installed package; `pyproject.toml` only packages `main_controller`). These are the actual dev/test workflow for this project in lieu of a test suite — see Commands above for the pipeline order.
- **`logs/`** — run data. `.gitignore` lists `logs/*.csv`, but CSV run logs are in fact tracked in git as records of past characterization/validation/human runs — don't assume this directory is disposable or untracked.
