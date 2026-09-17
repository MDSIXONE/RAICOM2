# GitHub Commit & Pull Request Guidelines

## Commit Workflow

1. Analyze the diff: `git diff --staged` for staged changes, `git diff` for unstaged, plus `git status --porcelain`.
2. Stage as needed (`git add` or `git add -p` to group). **Never commit secrets** (.env, credentials.json, private keys, etc.).
3. Determine type (Chinese, e.g. 功能/修复/文档), optional scope, and a concise Chinese description from the diff (≤72 characters).
4. Commit: `git commit -m "✨ 功能: 新增深色模式切换"`; for multi-line messages use a heredoc with body/footer (`Closes #123`, `Refs #456`).

## Pull Request Guidelines

Pull requests should explain:

- affected sections
- safety implications
- validation
- linked issues
- layout screenshots when needed

## Git Safety Protocol

- Never modify git config.
- Never run destructive commands (--force, hard reset) unless explicitly requested.
- Never skip hooks (--no-verify) unless the user asks.
- Never force-push to main/master.
- When a commit fails due to hooks, fix the issue and create a **new commit** (do not amend).
