# AGENTS.md

Guidelines for AI agents contributing to this repository.

## 1. Code & Documentation

- Always write source code and documentation in English.
- When modifying source code, update the corresponding documentation
  (e.g. `README.md`, command-line help text) to reflect the changes.
- When writing documentation, always consult the corresponding source
  code so that the description does not drift from the actual
  behavior. Verify claims against the implementation rather than
  relying on memory or assumptions.
- Use bold (`**...**`) sparingly in Markdown (documentation, PR
  descriptions, commit message bodies). Reserve it for genuinely
  critical warnings or terms that must stand out from surrounding
  prose; do not bold ordinary keywords, type names, or section
  labels — over-use dilutes the signal and makes documents feel
  shouty.
- Never write phrases that only make sense within the context of an
  AI–developer conversation. A future contributor or user reading the
  source must be able to understand the reasoning from the code and
  documentation alone. Concretely, do not embed shorthand references
  to design-discussion artifacts that are not in-tree — e.g. "plan A",
  "option B", "decision 3/D", "project decision 12/A", "Q6 decision",
  "per the design doc", or numbered selections from a chat-mode
  question/answer. Such labels are meaningless to anyone who did not
  participate in the conversation. Instead, spell out the rationale
  inline — what the rule is and why — so the comment, log message, or
  documentation stands on its own. If a deeper write-up exists, link
  to a tracked, in-repo document (e.g. an ADR file under `docs/`),
  not to a private conversation.

## 2. Attribution

- Do NOT include AI agent signatures (e.g. `Co-Authored-By: <agent name> ...`)
  in any generated code, commit messages, pull request descriptions,
  documentation, or other output.

## 3. Commit Messages & Branch Names

- Follow the [Conventional Commits](https://www.conventionalcommits.org/)
  specification for every commit message. Use one of the standard types
  (`feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`, `ci`,
  `style`, `build`, `revert`) with an optional scope, e.g.
  `feat(bag): add multi-topic inspect`.
- Use the same type as the branch prefix when creating a branch for a
  pull request (e.g. `feat/multi-topic-inspect`, `fix/hesai-sop-order`).
  Do not use tool- or author-specific prefixes such as `claude/*`.

## 4. Pre-commit Hooks

- Do not bypass pre-commit hooks (e.g. `--no-verify`). When a hook
  reports an error, fix the underlying issue and re-commit — never
  skip or disable the hook to push work through.

## 5. Local builds

- Build this package with `./build.sh` from the repository root, after
  sourcing the ROS 2 underlay (same prerequisite as in `README.md`
  Installation). The script wraps `colcon` with the expected workspace
  layout and flags; use `./build.sh --help` for clean rebuilds, build type,
  and parallelism.
- Prefer `./build.sh` over ad-hoc `colcon build` invocations when
  verifying changes, unless you are reproducing a CI or tooling issue that
  requires a different command line.

## 6. GitHub Actions / CI

- Be mindful of the GitHub Actions workflows configured in this
  repository; ensure changes do not cause them to fail.
- When investigating workflow failures, use the `gh` command
  (e.g. `gh run view`, `gh run view --log-failed`) to retrieve and read
  the actual logs. Base bug fixes on evidence from those logs, not on
  assumptions.

## 7. Remote Repository Operations

- Always obtain explicit developer approval before making any changes
  to the remote repository — pushing commits, creating/closing pull
  requests or issues, commenting on PRs, and so on.
- Do not push directly to the `main` branch. Always open a pull request
  first.
- Write PR descriptions that are comprehensive and detailed, yet
  concise: cover the problem, the solution, and the test plan without
  unnecessary verbosity.

## 8. Resource Management

- When writing code that acquires or releases a resource (memory, file
  handles, sockets, mutex locks, terminal modes, ROS handles, etc.),
  follow the RAII principle as much as possible: tie the resource's
  lifetime to a stack object whose constructor acquires it and whose
  destructor releases it. Prefer standard wrappers (`std::unique_ptr`,
  `std::lock_guard`, `std::fstream`, ...) or a small purpose-built
  guard type over manual `new` / `delete`, paired `open` / `close`
  calls scattered through the body, or `try` / `catch` blocks whose
  only job is cleanup.
