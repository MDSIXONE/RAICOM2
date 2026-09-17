# Universal Rules

- Always answer in Chinese unless the context requires otherwise.
- Do not write defensive or fallback code; it does not solve the root problem. Prefer full exposure: let failures surface clearly (explicit errors, exceptions, logs, failing tests) so bugs are visible and can be fixed at the root cause.
- When editing existing code: if you notice unrelated dead code, mention it - don't delete it.

# Repository Guidelines

## Commit & Pull Request Guidelines

Follow the `github-commit` skill for commit message and pull request conventions. Use one coherent change per commit. Do not rewrite published history merely to restyle messages.

## Security & Configuration

Never commit keys, credentials, private prompts, or production data.

## Agent-Specific Instructions

Any task that writes or modifies source code may use a sub-agent to complete the task.

## AI Work Records

Use the repository `project-memory-records` skill for all project work.

# Windows PowerShell 防错（本机 pwsh 7.x）

在 Windows 上执行终端命令前，先对照以下最容易犯的错：

- 比较用 `-eq`/`-ne`（不是 `==`）；赋值是 `=`，`==` 在 PowerShell 中不合法。
- `curl`/`wget`/`ls`/`cat` 都是别名（实为 Invoke-WebRequest / Get-ChildItem / Get-Content），行为与 Linux 不同；需要真实 curl 时用 `curl.exe`。
- 转义符是反引号 `` ` ``，不是 `\`；双引号内 `$var` 会插值，需要字面量用单引号或 `` `${var} ``。
- cmdlet/原生命令失败不中止脚本：执行原生 exe 后必须检查 `$LASTEXITCODE`（或 `$?`）；需要严格时用 `$ErrorActionPreference = "Stop"`。
- 调用路径含空格的 exe 用调用运算符：`& "C:\Program Files\app.exe" args`。
- 管道传的是对象不是文本；`2>&1` 是把错误流并入成功流，不是 stderr 纯文本。
- 中文输出乱码时先设 `[Console]::OutputEncoding = [Text.Encoding]::UTF8`，读写文件用 `-Encoding UTF8`。
