Read AGENTS.md.

Two things in it are easy to miss and expensive to get wrong:

- **The "Cross-app agent format" section.** Every policy trained from here must use the 131-observation /
  28-action role-ordered interface, take its numbers from `RobotControl/configs/contract.py`, export a plain
  MLP with no normalisation in the graph, and pass `RobotControl/training/policy_guard.py` at the end of
  export. The authoritative rules are `PoPunkouterSoftware/docs/unite/RULES.md`.
- **Legacy policies are grandfathered.** The 23/7, 9/2, 6/1 and 83/24 interfaces listed in
  `agents/registry.json` stay runnable and are never migrated or promoted. Do not "fix" them, and do not
  reorder observations so an old brain fits a new loader.

If a creature has no contract in `PoPunkouterSoftware/docs/unite/creatures/`, stop and report it. Do not invent one.
