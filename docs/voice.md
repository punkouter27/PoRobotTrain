# Voice commands and spoken replies

Say what you want; the web app fills in the selection and answers out loud. (The Unreal app is the
simulation-only scenario lab, docs/scenario_lab.md, and has no voice input.)

| | Web app (Blazor) |
|---|---|
| Listen | mic button in the "Ask in plain words" box: Web Speech API (Chrome/Edge), else Whisper tiny.en via Transformers.js, local |
| Understand | MiniLM matcher in the browser (`assistant.js`); the service matcher `POST /assistant/interpret` (`robotcontrol/behaviours/voice_intent.py`) returns the same fields |
| Answer | `speechSynthesis` |

Both matchers return the same suggestion fields and reply templates (`docs/service_api.md` section 1).

## Safety: voice only suggests

- A behaviour request selects the behaviour and sweep size, exactly like clicking them. Running it still needs
  Step 1 (virtual), Step 2 (read-only check) and the confirm dialog.
- A request about one joint ("move the right elbow 40 degrees") points at that joint's slider. It is never turned
  into a multi-joint behaviour, and legs and waist answer "locked on the stand".
- Arm behaviours stay blocked until "Unlock arm motions" is ticked.
- The only thing voice drives directly is the **walk sandbox** (simulation only): "walk forward", "turn left",
  "step right", "faster", "slower", "stop".
- Voice is not an emergency stop. The remote (L2+B for 5 s) is.

## Examples

| say | result |
|---|---|
| "nod the head twenty degrees" | Neck Nod, 20° selected |
| "shake your head" | Neck Turn selected |
| "wave hello" | Hand Wave (needs arm motions unlocked) |
| "move the right elbow 40 degrees" | use the Right elbow slider (40°) |
| "bend the left knee" | Left knee is locked on the stand |
| in the walk sandbox: "walk forward" / "turn left" / "stop" | sets the sandbox velocity command |

Tests: `RobotControl/tests/test_voice_intent.py`.
