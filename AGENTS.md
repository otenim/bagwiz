# AGENTS.md

Guidelines for AI agents contributing to this repository.

The rules below are split into two top-level groups:

- **General Rules** apply to any contribution and are not specific
  to bagwiz's domain or codebase. Most would carry over unchanged
  to another repository.
- **Project-Specific Rules** apply only to this repository —
  bagwiz's build workflow, its CLI surface, and similar choices
  that would not generalize.

The final section ("Maintaining These Guidelines") describes how
to keep this file itself consistent over time.

## General Rules

### 1. Code & Documentation

- Always write source code and documentation in English.
- When writing documentation, always consult the corresponding source
  code so that the description does not drift from the actual
  behavior. Verify claims against the implementation rather than
  relying on memory or assumptions.
- Use bold (`**...**`) and emoji sparingly in Markdown
  (documentation, PR descriptions, commit message bodies). Reserve
  bold for genuinely critical warnings or terms that must stand out
  from surrounding prose; do not bold ordinary keywords, type names,
  or section labels. Use emoji only when it adds clear meaning or
  improves readability, and avoid decorative emoji in source comments,
  generated documentation, commit messages, and pull request
  descriptions.
- When authoring Markdown files, write mathematical notation as LaTeX
  (`$...$` for inline math, `$$...$$` for display math) and express
  diagrams — process flowcharts, architecture diagrams, and similar
  visuals — as [Mermaid](https://mermaid.js.org/) in `mermaid`-labeled
  fenced code blocks, rather than as ASCII art or committed image
  files. GitHub renders both natively in its Markdown, so they stay as
  plain-text source that is easy to review and edit. Reach for a static
  image only when a diagram genuinely cannot be expressed in Mermaid —
  notably spatial / 3D-geometry figures in the command documentation,
  which are committed as plain SVG under `docs/commands/assets/` (see
  `docs/commands/TEMPLATE.md`).
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

### 2. Public Communication

- Write all GitHub-facing communication in English. This includes pull
  request titles and descriptions, issue titles and bodies, review
  comments, inline code comments on PRs, GitHub Discussions, and any
  other text posted through GitHub's interface. The only exception is
  non-English content that is intentionally committed or included as
  test data, fixtures, or samples (for example, a multi-language
  string used to exercise encoding handling).

### 3. Attribution

- Do NOT include AI agent signatures (e.g. `Co-Authored-By: <agent name> ...`)
  in any generated code, commit messages, pull request descriptions,
  documentation, or other output.

### 4. Commits, Branches & Worktrees

- Follow the [Conventional Commits](https://www.conventionalcommits.org/)
  specification for every commit message. Use one of the standard types
  (`feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`, `ci`,
  `style`, `build`, `revert`) with an optional scope, e.g.
  `feat(bag): add multi-topic inspect`.
- Use the same type as the branch prefix when creating a branch for a
  pull request (e.g. `feat/multi-topic-inspect`, `fix/hesai-sop-order`).
  Do not use tool- or author-specific prefixes such as `claude/*`.
- Do all implementation work inside a dedicated git worktree created
  for the branch before the first change
  (`git worktree add <path> -b <type>/<name>`), rather than switching
  branches in the primary checkout. Keep the primary checkout on the
  main branch as the stable base. A per-branch worktree keeps parallel
  efforts from trampling one shared working tree, keeps each change's
  build products and scratch state together (ccache still shares
  compiled objects across worktrees — see "Build Workflow"), and makes
  abandoning an attempt as cheap as removing one directory. Place the
  worktree outside the repository (e.g. a sibling directory) or under
  a git-ignored path, never at a tracked path.
- The worktree lives exactly as long as its branch: it is removed as
  the first step of the branch-retirement sequence that runs once the
  branch's pull request has merged into the main branch — see "Remote
  Repository Operations" (rule 7), which owns that cleanup. Do not
  leave merged worktrees behind, and do not remove one while its pull
  request is still open.
- Prefer a new commit over rewriting an existing one. History rewrites
  (`git commit --amend`, interactive rebase, and the rest) are for a
  commit you created moments ago that nothing has seen yet: no reviewer
  read it, no CI ran on it, no push happened, and its hash is recorded
  nowhere. Outside that window, correct the work with a follow-up
  commit. A branch that shows the correction as its own commit is
  easier to review than one whose history was quietly edited to look
  right.
- Never amend to fold in review feedback. The value of a review is that
  the reviewed state and the correction are both visible; amending
  destroys the commit the reviewer actually read and silently
  invalidates the hash they cited.
- Never rewrite a commit that has already been pushed. Undoing that
  needs a force-push, which rule 7 forbids without explicit approval.
- The real remedy is getting the commit right the first time: stage
  deliberately, run the hooks, and read the diff before committing,
  rather than committing early and repairing afterwards. Repeated
  amends are a symptom of committing too soon, not a workflow.

### 5. Pre-commit Hooks

- Before committing, inspect staged and unstaged changes and ensure the
  commit does not include secrets, credentials, private keys, tokens,
  `.env` files, large generated or binary artifacts, personal
  information, or other files that should not be published to GitHub. If
  any such file is present or ambiguous, stop and warn the developer
  instead of adding it to the commit.
- Do not bypass pre-commit hooks (e.g. `--no-verify`) and do not
  disable a check globally. When a hook reports an error, fix the
  underlying issue and re-commit rather than skipping the hook to
  push work through. The only exception is a genuine false positive:
  suppress it narrowly at the offending site (e.g. an inline
  `// NOLINT(...)`, `// cppcheck-suppress`, or equivalent
  hook-specific directive scoped to the smallest unit possible).

### 6. GitHub Actions / CI

- Be mindful of the GitHub Actions workflows configured in this
  repository; ensure changes do not cause them to fail.
- When investigating workflow failures, use the `gh` command
  (e.g. `gh run view`, `gh run view --log-failed`) to retrieve and read
  the actual logs. Base bug fixes on evidence from those logs, not on
  assumptions.

### 7. Remote Repository Operations

- Always obtain explicit developer approval before making any changes
  to the remote repository — pushing commits, creating/closing pull
  requests or issues, commenting on PRs, and so on.
- Every modification to the existing codebase, no matter how trivial,
  must go through a pull request. Do not push directly to `main` or
  any other shared branch — open a PR first, even for small fixes
  such as typo corrections, formatting, or one-line changes.
  Direct commits to the main branch are forbidden: no matter how
  small the change, always create a branch and merge it into the
  main branch through a pull request.
- Never merge a pull request unless every required CI check has
  completed successfully. If any CI job is failing, pending, or
  skipped in a way that bypasses required checks, investigate and fix
  the underlying issue before merging — do not merge to "unblock" the
  branch or rely on follow-up PRs to clean up red CI.
- Write PR descriptions that are comprehensive and detailed, yet
  concise: cover the problem, the solution, and the test plan without
  unnecessary verbosity.
- Retire the working branch as soon as its pull request is merged, so
  merged work stops accumulating locally and on the remote. Standing
  from a checkout other than the branch's own worktree, in this
  order: remove the branch's worktree — every implementation branch
  has one, per rule 4 of this group
  (`git worktree remove <path>`), delete the local branch
  (`git branch -d <branch>`), delete the remote branch
  (`git push origin --delete <branch>`, skipped when the repository
  already deleted it on merge), and update the main branch
  (`git switch main && git pull --prune`). The approval the developer
  gave to merge covers the deletions that follow it, so do not ask a
  second time for them.
- Stop and report rather than forcing a step of that cleanup that
  refuses to run — a worktree holding uncommitted changes, or a local
  branch git will not delete because it carries commits `main` does
  not contain. A squash or rebase merge always produces the latter,
  since it lands rewritten commits; confirm the pull request really
  is merged (`gh pr view <number> --json state,mergedAt`) before
  reaching for `git branch -D`, and never use it on a branch whose
  fate is unconfirmed.

### 8. Resource Management

- When writing code that acquires or releases a resource (memory, file
  handles, sockets, mutex locks, terminal modes, ROS handles, etc.),
  follow the RAII principle as much as possible: tie the resource's
  lifetime to a stack object whose constructor acquires it and whose
  destructor releases it. Prefer standard wrappers (`std::unique_ptr`,
  `std::lock_guard`, `std::fstream`, ...) or a small purpose-built
  guard type over manual `new` / `delete`, paired `open` / `close`
  calls scattered through the body, or `try` / `catch` blocks whose
  only job is cleanup.

### 9. Temporary Scratch Files

- When you generate code or data purely for temporary, single-use
  purposes — throwaway scripts, scratch experiments, synthetic or
  mock test data, and similar artifacts not intended to become part
  of the repository — write them under `/tmp` rather than inside the
  working tree. Delete them as soon as they are no longer needed so
  they do not accumulate, leak into a commit, or clutter the project
  directory.

## Project-Specific Rules

Each subsection below scopes its rules to a specific surface of
the repository. A rule applies only to the surface named by its
subsection — for example, the conventions under "bagwiz CLI"
govern only the CLI surface and do not extend to internal C++
APIs, build scripts, or any other code that is not part of the
CLI.

### 1. Build Workflow

Applies when building bagwiz from this repository. Does not
govern source code or CLI behavior.

- bagwiz is built and run through [pixi](https://pixi.sh); no system ROS 2
  install is required. pixi provisions ROS 2 from RoboStack (one conda channel
  per distro) and the C/C++ toolchain from conda-forge. Build with
  `pixi run -e <distro> build` from the repository root, where `<distro>` is
  `humble` or `jazzy`; a bare `pixi run build` targets
  the default environment (Jazzy). Use `build-full` instead when you need the
  `map`/SLAM command group. Each distro builds into its own
  `build/<distro>` and `install/<distro>`, so switching distros never reuses
  another distro's CMake cache.
- Builds are a `{core, full} x {cpu, cuda}` matrix, but the CPU/CUDA choice is
  derived from the active pixi environment. `build-full` is the default full
  build: on `humble`/`jazzy` it builds the GLIM stack (via
  `scripts/bagwiz-build.sh` → `scripts/build-glim-deps.sh`) and links the `map`
  command group (`bagwiz map slam`). `build` builds every command group
  EXCEPT `map`, with no GLIM stack — the fast build, on any distro. The CUDA
  SLAM build is `pixi run -e <distro>-cuda build-full` (humble-cuda/jazzy-cuda;
  needs an NVIDIA GPU); `build` in a `*-cuda` env is the core build inside a
  `*-cuda` env.
- Run the tests with `pixi run -e <distro> test-core` / `test-full` and the built
  binary with `pixi run -e <distro> run -- <args>` (or `pixi shell -e <distro>`
  then `bagwiz`). `pixi run install` does NOT build — it installs an optional
  `bagwiz` launcher on `PATH` plus shell completion for your current shell, in one
  step (always overwriting existing copies), wiring them to whatever `build`
  or `build-full` already produced in that environment (build first, or it errors
  with the command to run). It runs the binary in its pixi-managed ROS
  environment. See `scripts/bagwiz-install.sh`. `run` targets the full build
  (`build-full`) in whichever environment it is invoked.
- Prefer the `pixi run` tasks over ad-hoc `colcon build` invocations when
  verifying changes, unless you are reproducing a CI or tooling issue that
  requires a different command line. The task definitions live in `pixi.toml`.
  Both build tasks take an optional second positional arg forwarded to colcon's
  `--parallel-workers` (pixi task args are positional-only, so the build type
  comes first: `pixi run build Release 8`); unset means the default of
  half the physical CPU cores, and `BAGWIZ_BUILD_PARALLELISM` sets the same cap.
  `BAGWIZ_BUILD_TYPE` likewise selects the CMake build type when no positional
  arg is given.
- Builds compile through ccache and the Ninja generator, both pixi-provided and
  wired up by `scripts/bagwiz-build.sh` (CMAKE\_\*\_COMPILER_LAUNCHER and
  CMAKE_GENERATOR; build-glim-deps.sh repeats the launcher exports for
  standalone runs). ccache reuses objects across rebuilds, distros and git
  worktrees (CCACHE_NOHASHDIR=1), and CI restores the same cache; a build dir
  configured with a different generator is wiped and reconfigured automatically.

### 2. bagwiz CLI

Applies only to the `bagwiz` executable and the subcommands defined
under `bagwiz/src/commands/`. The rules below govern the user-facing CLI
surface — argument naming, ordering, help text, and similar concerns
— and do not apply to internal C++ APIs, library headers under the
packages' `include/` directories, build scripts, tests, or any other
code that is not part of the CLI itself.

- Order positional arguments on every `bagwiz` subcommand to follow
  common Unix utility conventions: read-side / source operands come
  first and the write-side / destination operand comes last,
  mirroring `cp src dst`, `mv src dst`, and `ln target name`.
  POSIX.1-2017 Base Definitions, Chapter 12 ("Utility Conventions",
  XBD §12) defines the surrounding option/operand syntax (option
  placement, the `--` separator, and so on); the
  source-before-destination ordering itself is not a separately
  published standard but the de facto convention codified by the
  POSIX utility specifications (`cp`, `mv`, `ln`, `install`, ...)
  and reinforced by the GNU Coding Standards. When a bagwiz
  subcommand must deviate — for example, when a single operand
  acts as both source and destination, or when the read-side
  selector is more naturally placed last — briefly state the
  reasoning in that subcommand's help text so users do not have
  to infer it from the signature.

- Name a pair of TF frame ids that names a rigid transform `--of`
  and `--ref`: `--of` is the frame of the object whose pose is
  resolved, `--ref` is the reference frame the pose is expressed
  in. The pair has exactly one meaning, on every subcommand: the
  command resolves **the pose of `--of`, expressed in the `--ref`
  frame** — `T = lookupTransform(target=<ref>, source=<of>)`,
  satisfying `p_ref = T · p_of`. This is equivalent to
  `ros2 run tf2_ros tf2_echo <ref> <of>`. The flag form
  (`--of <frame> --ref <frame>`) is the standard surface because it
  lets each operand name itself at the call site. A subcommand may
  take the pair positionally instead, in the order `<ref> <of>`
  (the `tf2_echo` order); the positional-ordering rule above does
  not apply to that choice, since a frame pair has no natural
  source/destination reading.

- Keep every subcommand's help text simple: cover what the command
  does, its operands and options, and only the key points a user
  needs to invoke it correctly. Do not embed extended rationale,
  background discussion, or tutorials in the help output; write
  those details in the documentation (e.g. `README.md` or files
  under `docs/`) instead. Notes that another rule in this section
  requires in the help text (such as the reason for a non-standard
  operand order) are still welcome, but keep them brief as well
  and leave the full explanation to the documentation.

### 3. Progress Indicators

Applies to any bagwiz subcommand or supporting code that renders an
in-terminal progress bar, spinner, or similar progress UI.

- When a command needs to show progress — a determinate bar, an
  indeterminate bar, or a spinner — use the
  [p-ranav/indicators](https://github.com/p-ranav/indicators) library
  rather than hand-rolling terminal control sequences or introducing a
  second progress-bar dependency. indicators is already pulled in via
  `FetchContent` and wrapped by the SLAM progress reporter
  (`bagwiz_slam/include/bagwiz/core/slam/progress_bar.hpp`,
  `bagwiz_slam/src/core/slam/progress_bar.cpp`); reuse that library, and the
  existing wrapper where it fits, so progress output stays visually
  consistent across commands and the build depends on a single,
  well-tested implementation.

### 4. Interactive TUIs

Applies to any bagwiz code that renders an interactive terminal UI —
today the `walk` command's views (the YAML pager, the image preview,
and their overlays and pickers) — and to any TUI added later.

- Keep every TUI simple, at introduction and as it evolves. The
  always-visible chrome (footer legends, info rows, status lines)
  shows only the working set of the current mode, ideally one row on
  a typical terminal; everything else belongs behind an on-demand
  reference (walk's `?` overlay) or in a transient status message
  that clears on the next action. When a new feature would widen the
  persistent chrome, move its reference material behind `?` or drop
  the hint rather than growing the footer.
- Give each key one meaning and keep that meaning identical across
  every command, view, and mode — a key must never do one thing in
  one TUI surface and something else in another. The established
  global bindings are: `?` opens the key reference, Esc backs out
  one level (close the overlay, leave the mode, leave the view;
  absorbed at the top so it cannot end the session),
  Ctrl-C / Ctrl-D terminate the session outright — from any screen,
  overlays and pickers included — and `j`/`k` scroll. Never rebind
  one of these locally. `q` is not a global binding: walk binds it
  only in the YAML view, where it quits walk; every other screen
  leaves it unbound (inert). When a view
  needs a new action, assign an unused key in `classify_key()`
  (`bagwiz_base/src/core/base/terminal_input.cpp`) — the single
  source of key bindings — instead of overloading a taken key with a
  context-dependent second meaning.

### 5. Documentation, Comment & Help Consistency

Applies whenever you change any bagwiz source code, regardless of
which part of the repository the change touches.

- After making any change to bagwiz's source code, investigate whether
  the change has introduced inconsistencies in the artifacts that
  describe that code — the related documentation (e.g. `README.md`),
  the comments embedded in the source itself, and the help text of
  every affected `bagwiz` command. Fix each inconsistency you find as
  part of the same change so these descriptions never drift from the
  actual behavior.
- Name every environment variable that bagwiz itself defines with a
  `BAGWIZ_` prefix, in `UPPER_SNAKE_CASE` — without exception, so that
  any variable appearing in a user's environment is immediately
  attributable to bagwiz and can never collide with an unrelated tool.
  The rule covers every variable bagwiz introduces regardless of who
  reads it: the `bagwiz` binary (`BAGWIZ_LOG_LEVEL`), the launcher,
  installer, and build scripts (`BAGWIZ_INSTALL_DIR`), the contributor
  tooling (`BAGWIZ_BENCH_RUNS` in `scripts/bench-rewrite.sh`), and the
  test suite (`BAGWIZ_REAL_BAG`). Where a group of variables belongs to
  one tool rather than to the CLI's runtime behavior, add an infix
  naming that tool (`BAGWIZ_BENCH_*`) so the two sets stay
  distinguishable; never claim a bare generic name like `BAGWIZ_BAG` or
  `BAGWIZ_ENV`, which reads as a global default for the CLI itself.
- Do **not** prefix a variable that bagwiz merely _consumes_ under
  someone else's published convention — `NO_COLOR` (no-color.org),
  `RCUTILS_COLORIZED_OUTPUT` (rcutils), `HOME` / `XDG_DATA_HOME` /
  `XDG_CONFIG_HOME` / `XDG_CACHE_HOME` (XDG), `AMENT_PREFIX_PATH` and
  `LD_LIBRARY_PATH` (ROS 2 / the loader), `PIXI_*`, `CONDA_PREFIX`,
  `CMAKE_*`, `CCACHE_*`. Renaming these would break the very convention
  that makes them useful, since the value is set by the ecosystem rather
  than by us. The distinction is ownership, not who calls `getenv`: if
  bagwiz decides the variable's name and meaning, it takes the prefix;
  if bagwiz is honoring a name someone else published, it keeps that
  name verbatim.
- When adding, renaming, or changing the behavior of an environment
  variable read by bagwiz code or its scripts (the `BAGWIZ_*` variables
  and conventional ones like `NO_COLOR`), update `docs/environment.md`
  in the same change — that file is the single inventory of every
  supported variable, and a variable missing from it is effectively
  undocumented.
- When adding, renaming, or changing a `bagwiz` command's flags,
  positionals, subcommands, or the message types a topic-valued flag
  accepts, sync the completion tables in
  `bagwiz/src/commands/completion.cpp` in the same change. They
  re-declare every command's flags and accepted types independently of
  `configure()`, so they are the easiest drift surface to forget. Verify
  with the `bagwiz_completion_test` suite, and for bag-driven value
  completion additionally with a live `bagwiz __complete <cword> ...`
  against a real bag.

### 6. Module Layout

Applies to the C++ source tree: which package a new source file,
header, or dependency belongs in, and which include/link directions
are allowed.

- The workspace consists of 11 ament_cmake packages, one per repo-root
  directory, each with its own `package.xml` and `CMakeLists.txt`: the
  libraries `bagwiz_base`, `bagwiz_io`, `bagwiz_msg`, `bagwiz_image`,
  `bagwiz_video`, `bagwiz_tf`, `bagwiz_pointcloud`, `bagwiz_tui`,
  `bagwiz_bag`, `bagwiz_slam`, and the `bagwiz` package (the CLI
  executable and its command implementations).
- Layering is one-way: a package may include and link only packages
  below it in the stack — base → io → msg → image/tf/bag →
  pointcloud/tui → slam → CLI. `bagwiz_video` sits outside the stack:
  it depends on no other bagwiz library and only the CLI links it.
- The package DAG is enforced by colcon's topological build: a
  dependency cycle fails the build, so no separate cycle check is
  needed.
- The layering rule constrains what a package's _libraries_ link. A
  test executable may additionally compile a single `.cpp` from
  another package directly, as an extra translation unit, when the
  code it must assert on sits in a package that already depends on
  this one — linking that package would close a cycle the DAG
  forbids. No test does this today. Keep such cases rare and comment
  the CMake block with the cycle it avoids; when the assertions are
  really about the other package's own behavior, put the test there
  instead.

### 7. Numerical Reproducibility

Applies to any bagwiz code whose output is a number a user consumes —
map points, trajectory poses, colors, weights, normals — and to the
tests that assert on those numbers.

- Bit-exact agreement of numeric output is not a requirement. The same
  input may produce slightly different numbers on the CPU and the CUDA
  backend, and at different `-j, --threads` values. The difference is
  acceptable while it stays inside the tolerance class for that
  quantity. The values live in
  `bagwiz_base/include/bagwiz/core/base/tolerance.hpp`, one `constexpr`
  per class, each documented with the reason for its magnitude. Cite
  the class, do not re-derive a number at the call site.
- Pick the class by what the number is: `kPointMeters` for coordinates
  and translations, `kRotationRadians` for rotations,
  `kUnitVectorComponent` for normals and other unit vectors,
  `kScalarRelative` for general real scalars (ratios, scores,
  residuals), `kColorChannelLsb` and `kQuantizedWeightLsb` for values
  already quantized to `uint8`.
- Three things stay strict and are outside this relaxation. Message
  order inside a bag is downstream semantics rather than error —
  reordering has broken Foxglove before. The byte identity of a bag
  written by the parallel mcap write path across `--threads` values
  (`mcap_parallel_chunk_writer`) underpins output caching and
  diff-based checks; that covers this writer only, not every output
  file — `map.pcd`, for one, is not byte-identical across `--threads`.
  Discrete decisions — keep or drop, ok or error, which topic was
  selected — are behavior, so a flip is a bug, not drift.
- Counts and point totals are compared exactly in unit tests over
  synthetic input. A count that moves with the thread count there means
  the input sits on a tolerance boundary: fix the input, not the
  assertion. Only end-to-end comparisons over real bag data may use
  `kCountRelative`.
- Never trade performance for exact agreement. That a static chunking
  or a fixed merge order makes the result independent of the thread
  count is not by itself a reason to keep it; choose the fastest
  schedule and let the numbers land inside tolerance.
- A test may assert exact agreement only where that agreement is
  algorithmically guaranteed, never merely observed. Four shapes
  qualify: a commutative and associative reduction (`min` over floats,
  a monotone OR, integer addition); a per-element computation that
  reads only immutable input; a function of the fully materialized
  result set that does not depend on the order of materialization (a
  median taken after a sort); and a pass over shared mutable state
  whose writes are provably invisible to concurrent readers (a value
  written into a state that no reader accepts as input). Anything
  else — in particular a floating-point accumulation whose order
  follows the work split — is asserted within tolerance instead. When
  a test does assert exact agreement, state in the test which shape
  justifies it, so a later change that breaks the premise is visible
  in review.

### 8. Benchmarking

Applies whenever you measure the runtime performance of a bagwiz
command — an existing one or one being added — whether through
`scripts/bench-rewrite.sh`, the `BAGWIZ_PROFILE` stage report, or an
ad-hoc timing run. It does not govern correctness tests, which remain
free to build their input synthetically.

- Measure on a real recorded rosbag whenever one is available. A
  synthetic bag does not reproduce the message-size distribution,
  topic mix, chunk layout, compression ratio, and timestamp jitter of
  real sensor data, so a figure measured on one can point at a
  bottleneck the command does not actually have.
  `scripts/bench-rewrite.sh` is written around a real multi-GB bag for
  exactly this reason.
- When no real bag has been supplied for the work at hand, ask the
  developer for one before measuring, and state what the measurement
  needs: rough size, the topics and message types that must be
  present, and the recording length. Do not quietly settle for a
  synthetic bag because asking would take longer.
- Fall back to a synthetic bag only once it is established that no real
  bag is available. Then say so wherever the numbers are reported — the
  benchmark output, the commit message, the pull request description —
  along with how the bag was generated, so no one reads the figures as
  real-data measurements. Keep the generator and the bag itself outside
  the working tree, per General Rule 9.

## Maintaining These Guidelines

- Keep the rules in this file free of duplication. Each topic should
  have a single source of truth: if two or more bullets or sections
  cover the same ground, consolidate them into one authoritative rule
  rather than restating the same guidance in multiple places.
- Before adding a new rule, scan the existing sections first. If the
  new guidance fits an existing rule, extend that rule in place;
  otherwise, place it under the section whose scope it most clearly
  belongs to instead of creating a parallel rule elsewhere. When the
  new rule does not yet fit any section, decide first whether it is
  general or bagwiz-specific and add it under the matching top-level
  group.
- When editing this file, also check whether the change makes any
  pre-existing rule redundant. If it does, update or remove the
  now-overlapping text in the same change so the ruleset stays
  minimal and non-redundant.
- Keep the rules in this file mutually consistent. Before adding or
  modifying a rule, read the surrounding sections to make sure the
  new wording does not contradict any existing rule. If a genuine
  conflict is unavoidable (for example, the new rule is meant to
  supersede an older one), resolve it in the same change by updating
  or removing the conflicting rule rather than leaving both in place.
