# Learner KiCad starter

This is the recovered 2024 learner PCB template. It contains the ESP32-C3 and
MAX30102 circuit, with the two main footprints locked, but no routed tracks or
learner-designed LED circuit.

The source set now includes:

- `project_x_pcb.kicad_pro`
- `project_x_pcb.kicad_sch`
- `project_x_pcb.kicad_pcb`
- local library tables that point to the shared custom libraries one directory
  above

The files were recovered from the full `project_x.zip` source archive. They
have not yet been opened together in the intended KiCad version or checked
with ERC and DRC, so treat them as recovered historical source rather than a
validated release.

The board's 3D model references point to `../3d_models/`, but the component
models are deliberately absent pending licence confirmation. See
[`../3d_models/README.md`](../3d_models/README.md).
