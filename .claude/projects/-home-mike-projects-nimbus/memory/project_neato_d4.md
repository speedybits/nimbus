---
name: Neato D4 hardware
description: The robot uses a Neato D4 (not XV series) - affects protocol key names and hardware specs
type: project
---

The physical Neato vacuum is a D4 model, not an XV series.
**Why:** Protocol references in code comments say "XV-series" but the hardware is D4. The serial protocol is the same (ASCII over USB serial at 115200), and GetDigitalSensors returns the same key names (LSIDEBIT, LFRONTBIT, RSIDEBIT, RFRONTBIT).
**How to apply:** Use "D4" in comments/docs. Don't assume XV-specific features. Battery is 14.4V Li-ion (part 205-0011 per GetVersion).
