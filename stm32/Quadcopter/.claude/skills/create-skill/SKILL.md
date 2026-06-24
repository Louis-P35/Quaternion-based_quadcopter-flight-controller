---
description: >
  Instructions for writing a new skill in this project. Use when the user asks
  to create a skill, write a slash command, or document a procedure so Claude
  remembers it automatically in future sessions.
when_to_use: >
  Trigger on: "create a skill", "write a skill", "new skill", "slash command",
  "make Claude remember", "document this procedure", "/create-skill".
disable-model-invocation: false
---

# How to Write a Skill in This Project

## File Layout

Each skill lives in its own directory under `.claude/skills/`:

```
stm32/Quadcopter/.claude/skills/<skill-name>/
└── SKILL.md          ← required, this is the skill
```

The directory name becomes the slash command: `skills/scheduler-tasks/` → `/scheduler-tasks`.

## SKILL.md Structure

```markdown
---
description: >
  One or two sentences. What the skill does and when Claude should load it.
  Put the key trigger concept first — this text is what Claude matches against
  the conversation to decide whether to auto-invoke the skill.
when_to_use: >
  Comma-separated trigger phrases. Be specific to this codebase.
  Example: "Trigger on: addTask, frequency slot, SPI collision, timing budget."
disable-model-invocation: false   # true = only user can invoke with /name
---

# Skill Title

Body: plain markdown. Rules, tables, code snippets, anti-patterns.
No prose padding — every line Claude reads costs tokens each time the skill loads.
```

## Frontmatter Fields Used in This Project

| Field | When to set |
|---|---|
| `description` | Always. One clear paragraph, key trigger word first. |
| `when_to_use` | Always. Explicit "Trigger on: …" list improves auto-detection. |
| `disable-model-invocation: true` | For procedural skills with side effects (deploy, commit). |
| `user-invocable: false` | For background knowledge not meant to be invoked directly. |

All other fields (model, effort, context, allowed-tools) are optional and rarely needed.

## Body Guidelines

- **State rules, not explanations.** Claude knows how to code; it needs the
  project-specific constraints it cannot derive from the source files.
- **Use tables and code blocks** for reference material — faster to scan than prose.
- **Include anti-patterns.** The most valuable section is often "never do X because Y".
- **Keep it under 500 lines.** Move large reference tables to a sibling file and
  link to it from SKILL.md.
- **English only** — all skill content is in English (same rule as code comments).

## What Belongs in a Skill vs. CLAUDE.md

| Content type | Where it goes |
|---|---|
| Always-on rules (comment language, earth frame convention) | `CLAUDE.md` |
| Deep domain knowledge loaded only when relevant | Skill |
| Step-by-step procedures invoked on demand | Skill with `disable-model-invocation: true` |
| Ephemeral task state, in-progress work | TaskCreate / conversation only |

If a CLAUDE.md section has grown into a multi-step procedure, extract it into a skill.

## Existing Skills in This Project

| Skill | Command | Auto-invoked when… |
|---|---|---|
| `scheduler-tasks` | `/scheduler-tasks` | User discusses tasks, SPI, timing, addTask, priorities |
| `create-skill` | `/create-skill` | User asks to create a skill or slash command |

## Checklist for a New Skill

1. Identify what Claude keeps getting wrong or what you keep re-explaining.
2. Choose a name: lowercase, hyphen-separated, describes the domain (`spi-protocol`, `pid-tuning`).
3. Create `stm32/Quadcopter/.claude/skills/<name>/SKILL.md`.
4. Write `description` + `when_to_use` frontmatter first — these drive auto-invocation.
5. Body: rules first, anti-patterns last, no narrative filler.
6. Test: start a new session and ask something that should trigger it. Run `/skills` to confirm it appears.
