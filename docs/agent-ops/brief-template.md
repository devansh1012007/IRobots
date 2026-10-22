# Task brief template

Copy this, fill every field, delete nothing. A brief with a blank field is
not ready to send.

---

## Goal

<One sentence. What exists at the end that does not exist now?>

## Context

<What is in the repository today, what is broken or missing, and the evidence
for it. Cite `path:line` or quote the actual output. The agent cannot see
your conversation, so anything it needs to know has to be here.>

## Scope: files you may create or modify

- `<path>`
- `<path>`

## Files you must NOT touch

<Everything else, especially files another agent owns. If you believe you
need to change one, say so in your report and stop.>

## Interface contract

<The exact signatures the module must expose, so it fits with modules other
agents are writing in parallel. Include the types, the error behaviour, and
what each function returns.>

```python
<signature>
```

## Branch and working directory

- **Working directory:** `<absolute path>` (a git worktree)
- **Branch:** `<branch name>`

Work ONLY inside that path. Do not run `git checkout`, `git switch`, or
`git worktree` yourself. Do not touch the main checkout. Do not merge.

## Test command

```bash
cd <absolute path> && /home/devansh/Code/IRobots-Devansh_Gaur/.venv/bin/python -m pytest <path> -q
```

The tests must pass, and you must paste the **actual output** in your report.

## Constraints

- Read `AGENTS.md` and the relevant part of `PLAN.md` first.
- Every physical constant carries a `provenance` tag.
- No hardcoded IP, port, path or model id in source.
- Every random draw goes through a seeded `numpy.random.Generator`.
- The pure-Python core must not import `rclpy` or any ROS package.
- Trace tag `PLAN:irobots-v2:C##` in the code comments and in the commit
  message.
- Write the failing test first for a bug fix, alongside the code for a
  feature.
- Never weaken, skip or delete an existing test.
- `ruff check` and `mypy` must be clean on the files you touched.

## Return format

Report exactly these, in this order:

1. **Files changed** - path, what changed, plan change ID.
2. **Test output** - the command and its real output, pasted.
3. **Lint and type check** - the commands and their real output.
4. **Deviations** - anything you did differently from this brief, and why.
5. **Not done** - anything you could not complete, and what you need.
6. **Commit** - the SHA, and the branch you committed to.
