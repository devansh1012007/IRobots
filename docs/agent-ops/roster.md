# Agent roster

The definitions live in `.opencode/agent/`. This table is what each is for
and, more importantly, what each is *not* for.

| Agent | Mode | Use it for | Do not use it for |
|---|---|---|---|
| `irobots-explorer` | read-only | Answering a question about the code, the configs, the git history, or the CAD/STL assets | Anything that changes a file |
| `irobots-implementer` | writes | Implementing a change against a written spec, with tests | Deciding what the design should be |
| `irobots-reviewer` | read-only | Trying to break a change before it is merged | Writing the fix |
| `irobots-perception` | writes | Detector, calibration, coordinate transform, height estimation, identity tracking, grasp candidates | Kinematics, control, or the agent loop |
| `irobots-physics` | writes | Kinematics, trajectories, the safety supervisor, the MuJoCo model, determinism | Perception, the agent loop, or the firmware protocol |
| `irobots-firmware` | writes | The ESP32 sketch, the wire protocol, the host-side emulator, the conformance suite | The host-side control stack |

## Roles that are not in the roster, and why

* **No "orchestrator" agent.** Deciding what to delegate, and in what order,
  is the operator's judgement. An agent that decides the plan is a plan
  nobody approved.
* **No "test writer" agent.** A test written by someone who did not write the
  code tends to assert the implementation rather than the behaviour. The
  implementer writes the test; the reviewer tries to break it.
* **No agent writes the research report.** `docs/RESULTS.md` is the operator's
  claim about what the numbers mean, and it has to be written by whoever is
  willing to defend it.

## The built-in agents

opencode also ships `build`, `plan`, `general` and `explore`. They are
fine for one-off lookups. The agents above exist because this project's
invariants, its provenance rule and its "the core must not import ROS" rule
need to be in front of the agent on every task, and the built-ins do not
carry that context.

## The specialisation, in one line each

* **explorer** - read-only; cites `path:line`; never proposes a design.
* **implementer** - writes code and tests; runs them and pastes the output.
* **reviewer** - hostile; tries to break the change; runs the tests itself.
* **perception** - owns the three legacy defects: identity, the coordinate
  frame, and invented Z.
* **physics** - owns the reach, the limits, the safety supervisor, and
  determinism.
* **firmware** - owns the wire protocol, and the rule that the firmware is
  verified against a host-side emulator before any hardware is involved.
