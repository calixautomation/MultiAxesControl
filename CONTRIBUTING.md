# Contributing to MultiAxesControl

Thank you for your interest in contributing to the MultiAxesControl project. This guide covers how to get set up and the workflow we follow.

## Getting Started

### 1. Fork the Repository

Go to [https://github.com/calixautomation/MultiAxesControl](https://github.com/calixautomation/MultiAxesControl) and click **Fork** in the top-right corner to create your own copy.

### 2. Clone Your Fork

```bash
git clone https://github.com/<your-username>/MultiAxesControl.git
cd MultiAxesControl
```

### 3. Add the Upstream Remote

This lets you pull in changes from the main repository:

```bash
git remote add upstream https://github.com/calixautomation/MultiAxesControl.git
```

Verify your remotes:

```bash
git remote -v
# origin    https://github.com/<your-username>/MultiAxesControl.git (fetch)
# origin    https://github.com/<your-username>/MultiAxesControl.git (push)
# upstream  https://github.com/calixautomation/MultiAxesControl.git (fetch)
# upstream  https://github.com/calixautomation/MultiAxesControl.git (push)
```

### 4. Set Up Prerequisites

See the [software README](software/README.md) for the full list of tools you need (STM32CubeCLT, CMake, Ninja, Python, VS Code extensions).

## Contribution Workflow

### 1. Create a Feature Branch

Always branch off the latest `main`:

```bash
git checkout main
git fetch upstream
git rebase upstream/main
git checkout -b your-branch-name
```

Use a descriptive branch name, e.g. `add-motor3-support` or `fix-uart-parity`.

### 2. Make Your Changes

- Follow the existing code style and project structure
- Keep commits focused — one logical change per commit
- Write clear commit messages:

```bash
git add <files>
git commit -m "Short summary of the change"
```

### 3. Keep Your Branch Up to Date (Rebase)

We use **rebase** instead of merge to keep a clean, linear history. Before pushing or opening a pull request, rebase onto the latest `main`:

```bash
git fetch upstream
git rebase upstream/main
```

If there are conflicts during rebase:

1. Git will pause and tell you which files have conflicts
2. Open the conflicted files and resolve the conflicts manually
3. Stage the resolved files and continue:

```bash
git add <resolved-files>
git rebase --continue
```

4. If you need to abort and start over:

```bash
git rebase --abort
```

> **Important:** Do not use `git pull` — it creates unnecessary merge commits. Always use `git fetch` + `git rebase` instead.

### 4. Push Your Branch

If this is your first push for the branch:

```bash
git push -u origin your-branch-name
```

If you've already pushed and then rebased, you'll need to force-push:

```bash
git push --force-with-lease
```

Use `--force-with-lease` instead of `--force` as a safety measure — it will refuse to push if someone else has pushed to the same branch.

### 5. Open a Pull Request

1. Go to your fork on GitHub
2. Click **Compare & pull request**
3. Set the base branch to `main` on `calixautomation/MultiAxesControl`
4. Write a clear description of what your changes do and why
5. Reference any related issues (e.g. `Fixes #12`)

## Quick Reference

| Task | Command |
|------|---------|
| Sync with upstream | `git fetch upstream && git rebase upstream/main` |
| Create a branch | `git checkout -b branch-name` |
| Stage changes | `git add <files>` |
| Commit | `git commit -m "message"` |
| Push new branch | `git push -u origin branch-name` |
| Push after rebase | `git push --force-with-lease` |

## Guidelines

- **Do not use `git pull`** — always fetch and rebase to keep history linear
- **Do not commit directly to `main`** — always work on a feature branch
- **Build and test before pushing** — run `python scripts/build/build_firmware.py` to verify your changes compile
- Keep pull requests focused on a single change or feature
- Update documentation if your changes affect setup steps or the public API
