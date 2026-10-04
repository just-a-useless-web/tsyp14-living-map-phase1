# How to Evaluate The Living Map Simulation

1. **Map the Zone:** Run `python src/writer.py`. The simulation will open. You may press `2` or `3` to increase simulation speed. Allow the blue robot to explore, drop beacons, and return to the start to generate the ONA briefing.
2. **Execute the Mission:** Run `python src/executor.py`. The green robot will read the generated briefing and navigate point-to-point via simulated BLE packets.
3. **Trigger Failures:** While the Executor is running, use the keyboard to test system robustness:
   * `A`: Force beacons to age to STALE (requires live sensor verification).
   * `C`: Remove hazards (forces a SUSPECT contradiction).
   * `M`: Delete the next beacon in the chain (forces blind-search recovery).
   * `X`: Corrupt the BLE packet CRC.