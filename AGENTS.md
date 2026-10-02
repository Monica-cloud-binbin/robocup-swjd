# Project Workflow Notes

- The user's primary development workspace is `D:\350 competetion\EE\robocup\my_ws`.
- The user regularly synchronizes this workspace with `my_ws` on the onboard computer. Do not assume synchronization is automatic or has already happened.
- Unless explicitly asked otherwise, make and inspect code changes in the primary workspace on the host computer. Clearly distinguish host-side edits from files deployed to the onboard computer, and never claim an onboard update without confirmation.
- When preparing changes for onboard testing, keep deployment scope focused on changed source and launch files when possible. Call out any required package, dependency, configuration, message, or other files that must also be synchronized, and remind that compilation must use the onboard computer's own ROS workspace/environment.

# Bug Diagnosis and Records

- Whenever the user asks for help diagnosing or fixing a bug, track the observed symptom/error, the confirmed root cause (separate confirmed facts from hypotheses), the fix, and useful prevention/check steps.
- After the user confirms the bug is resolved, append a concise entry to the project-root `BUG_FIX_LOG.md` with the symptom, cause, fix, prevention, and verification result. Preserve existing entries; do not rewrite or remove prior records.
- If it is unclear whether the fix worked, or which fix was actually applied, ask the user to confirm before recording the issue as resolved. Do not claim verification that did not happen; unresolved cases may be recorded as pending verification when useful.
- Keep deployment and verification scope explicit: distinguish the host workspace, virtual machine, and onboard computer, and record which environment was actually tested.
- Keep `BUG_FIX_LOG.md` concise, readable, and accurate. Do not repeat the cause in the fix; make the fix section only reusable edits or commands. Omit prevention notes unless they add a necessary, actionable check. Separate functional symptoms with no explicit error from terminal/node errors, and preserve relevant verification status.
