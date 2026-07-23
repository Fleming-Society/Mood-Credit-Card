# Firmware

The current learner starter sketch is at
[`student/mood_credit_card/`](student/mood_credit_card/).

Recovered I2C examples from Session 2 are under
[`examples/session-02/`](examples/session-02/). Historical prototype ZIP
snapshots are retained under
[`archive/2024/prototypes/`](archive/2024/prototypes/) for comparison, not as
the primary learner code.

Before the next workshop:

1. Confirm the exact ESP32-C3 board target.
2. Pin the Arduino ESP32 core and DevXplained MAX3010x library versions.
3. Compile from a fresh environment.
4. Verify that `readBPM()` is called frequently enough to sample a heartbeat;
   it should not run only once per second.
5. Resolve or document the current 3200 SPS sensor setting versus the 400 Hz
   filter sampling-frequency constant.
6. Test behaviour with no finger, poor contact and sensor disconnection.

Keep learner starter code separate from any worked solution. If solutions
must remain restricted, store them outside this public repository.
