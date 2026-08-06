# Spiderbot Project — Claude Code Rules

## Scope discipline
- Only implement what is explicitly scoped in the task prompt for this session.
- Do NOT move to the next checkpoint, add features, or refactor unrelated code
  without being explicitly asked.
- If a task is ambiguous or requires a design decision not already specified,
  STOP and ask — do not assume.

## Source of truth
- progress_log.md is user-maintained. Do not edit it — read it for context only.
- Treat the "Key Decisions" and "Roadmap" sections as fixed constraints.

## Core logic ownership
- Core algorithms (IK solver, gait phase generator, pose controller math) are
  designed by the user in a separate design chat before being handed to you as
  an explicit spec. Implement exactly what's specified — don't invent the
  algorithm or change its structure.

## Verification
- Do not guess Isaac Sim/OmniGraph node names or ROS2 message types if unsure —
  flag the uncertainty explicitly and point to official docs instead of
  proceeding on a guess.
