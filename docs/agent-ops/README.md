# Agent operations: how work is delegated in this repository

This is the framework for using sub-agents. It exists because delegating
without it produces the two failure modes that actually happen: agents that
commit to the wrong branch, and agents that return a summary with no evidence
in it.

Read [`../../AGENTS.md`](../../AGENTS.md) first: it has the invariants
(I1-I6), the trace-tag convention, and the data rules. This document covers
the mechanics.

---

## 1. The one rule that matters most

> **Every agent is told, explicitly and in full, which branch and which
> working directory it may commit to. No exceptions. No "commit to main".**

An agent that guesses will either commit to `main` (unreviewable, and it
destroys the phase structure) or to whatever branch happens to be checked out
(possibly another agent's branch, corrupting it).

## 2. Parallel agents need separate working directories

Two agents cannot share one checkout: a `git checkout` by one of them moves
the other onto the wrong branch, and two `git commit`s race on
`.git/index.lock`.

So every parallel task gets its own **git worktree**:

```bash
git worktree add /tmp/opencode/wt/<task> -b feature/<task> main
```

The agent is given the absolute worktree path and the branch name, and is
told to work *only* inside that path. The worktree is a full checkout, so
`config/`, `ros2_ws/` and the tests are all present.

The shared virtualenv lives in the main checkout and is used by every
worktree:

```bash
/home/devansh/Code/IRobots-Devansh_Gaur/.venv/bin/python -m pytest ...
```

## 3. What a brief must contain

Every delegated task uses [`brief-template.md`](brief-template.md). A brief
without these fields is not ready to send:

| Field | Why it is required |
|---|---|
| **Goal** | One sentence. The agent cannot ask a question in a long task. |
| **Context** | What exists, what is broken, and the evidence for it. |
| **Scope: files you may create or modify** | Prevents an agent from wandering into files another agent owns. |
| **Files you must NOT touch** | Same, from the other side. |
| **Interface contract** | The exact function signatures, so two agents' modules fit together without a second round. |
| **Branch and working directory** | The absolute path and the branch name. |
| **Test command** | The exact command, and the requirement to paste its real output. |
| **Constraints** | The rules from `AGENTS.md` that apply to this task. |
| **Return format** | What the agent must report back. |

## 4. What an agent must return

An agent's report is not a narrative. It is:

1. The files it changed and what changed in each, with the plan change ID.
2. The test command it ran and **the actual output**, pasted. An agent that
   says "tests pass" without the output has not run them.
3. Any place it deviated from the brief, and why.
4. Anything it could not do, and what it needs.
5. The commit SHA, and confirmation of which branch it is on.

An agent that cannot complete the task says so and stops. It does not
half-finish a module and report it as done.

## 5. The review protocol

An agent's branch is not trusted because it exists. Before a branch is merged:

1. Read the diff. Look for: violations of I1-I6, hardcoded values, missing
   provenance tags, missing error handling, and tests that cannot fail.
2. Run the tests **in the main checkout** after merging, not only in the
   agent's worktree. An agent's worktree may have a stale dependency state.
3. Run the linter and the type checker.
4. Merge with `--no-ff` so the phase structure stays visible, and put the
   plan change IDs in the merge message.

The `irobots-reviewer` agent exists for step 1 and does not edit code.

## 6. Cost and task selection

Delegation is worth it when a task is:

- **well specified** - the interface is known and the acceptance test is
  mechanical;
- **independent** - it does not need the answer to another task;
- **verifiable in under a minute** - there is a test command.

Delegation is a waste when the task is exploratory, when the design is still
being decided, or when the work needs the judgement that the operator
already has in context. The perception problem currently being worked on is
an example of the second kind: it is being done in the main thread.

## 7. The roster

See [`roster.md`](roster.md) for the agents, what each is for, and their
constraints. The agent definitions live in `.opencode/agent/`.

**Note on availability:** opencode loads agent definitions at startup, so a
newly added agent is only selectable after a restart. Until then, the
brief's *role* section carries the specialisation, and the built-in `general`
agent fills it.
