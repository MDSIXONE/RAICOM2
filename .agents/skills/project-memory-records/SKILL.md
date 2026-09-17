---
name: project-memory-records
description: Maintain project-scoped AI change logs, mistake records, and failed-approach records. Use at the beginning of a new project conversation, before changing source code, after completing code changes, when an error yields a reusable lesson, or when considering, abandoning, or replacing an implementation approach.
---

# Project Memory Records

Maintain durable project learning records in `docs/ai-records/`. Create the folder and the files below from their repository templates if they are absent. Do not fabricate historical events.

## Read Before Work

At the start of every new conversation, read:

1. `docs/ai-records/CHANGE_LOG.md` to understand work in progress and completed changes.
2. `docs/ai-records/MISTAKE_INDEX.md` to avoid repeating known mistakes, then read only the day files the index points to that are relevant to the current task.

Before proposing or selecting a new implementation approach, also read `docs/ai-records/FAILED_APPROACHES.md`.

Follow repository-level instructions such as `AGENTS.md` after reading the records. If the task changes source code, satisfy any required sub-agent workflow before editing.

## Change Log

For every source-code change, update `CHANGE_LOG.md` twice when practical: create or mark the work unit `进行中` before implementation, then update it immediately after the change to `改动完成`. Use only those two status values. Record the date, goal, affected files, concise result, validation, and unresolved risks. Never add a third status such as “blocked” or “abandoned”; explain uncertainty in the notes instead.

Do not add an entry for documentation-only changes unless the repository explicitly requires it.

If `CHANGE_LOG.md` grows beyond a single-read limit, split it the same way as the mistake log: a small index plus date-bucketed files under `docs/ai-records/changes/`.

## Mistake Log

Store mistake entries as **a small index file plus date-bucketed entry files** (one file per day). The index stays small enough to read fully in one pass, and each day file stays small enough to read fully in one pass.

Directory layout under `docs/ai-records/`:

```
docs/ai-records/
├── CHANGE_LOG.md
├── FAILED_APPROACHES.md
├── MISTAKE_INDEX.md        # small index: topic index + date index
└── mistakes/
    └── YYYY-MM-DD.md       # one file per day; all entries of that day
```

### Index file: MISTAKE_INDEX.md

Keep it small (well under one read limit). Two sections:

1. `## 主题索引` (topic index): one line per recurring topic, listing the day files containing related entries, e.g. `- 跨 Shell 引号/变量展开：2026-08-09.md、2026-08-10.md`. Topics are free-form keywords; reuse existing topics instead of creating near-duplicates.
2. `## 按日期` (date index): one `### YYYY-MM-DD` section per day file, listing each entry title of that day.

The topic index exists to defeat grep synonym gaps: an entry described as “多层转义” is found under topic “跨 Shell 引号” even when the search keyword differs.

### Entry files: mistakes/YYYY-MM-DD.md

One file per day; append entries chronologically. Every entry uses the four-part template:

### 标题（必须包含可搜索的技术关键词，如 CRLF、NaN、nounset）

- **现象**：
- **原因**：
- **修复**：
- **预防**：

Titles must contain grep-able technical keywords so both the topic index and full-text search can locate the entry.

### Recording workflow

When an error, incorrect assumption, failed command, regression, or review finding produces a reusable lesson:

1. Append one concise entry to today's `docs/ai-records/mistakes/YYYY-MM-DD.md` (create the file if absent; use the current local date).
2. Update `MISTAKE_INDEX.md` in both places: add the entry title under the matching `### YYYY-MM-DD` date section, and add or update the topic line under `## 主题索引`.

Record confirmed lessons, not routine experimentation or speculation.

### Retrieval workflow

Before changing source code:

1. Read `MISTAKE_INDEX.md` fully — it is small and gives a complete overview.
2. Select relevant day files via the topic index, or `grep` the `mistakes/` folder for keywords.
3. Read only the selected day files; each is small enough for one full read.

Do not read every day file by default.

## Failed Approaches

Append to `FAILED_APPROACHES.md` only when the user explicitly identifies an approach as failed, or the work must switch to a different approach. State the original approach, why it was stopped, evidence, and the replacement or follow-up. Do not record a failed approach merely because an attempt was imperfect or incomplete.

## Finish

Before handing off code work, confirm that the change log is updated, relevant reusable mistakes are captured (entry + index updated), and any required failed-approach entry exists. Mention the record updates with the implementation summary.
