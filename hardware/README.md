# Hardware

## Editable PCB sources

[`kicad/`](kicad/) contains the completed KiCad exemplar, custom symbols and
custom footprints. [`kicad/learner/`](kicad/learner/) contains the recovered
2024 learner starter project with its board, schematic and project file.

The board files use `${KIPRJMOD}`-relative references. The supplied component
STEP models are withheld from the public tree until their redistribution
rights are confirmed; see
[`kicad/3d_models/README.md`](kicad/3d_models/README.md).

## Enclosure CAD

[`enclosure/`](enclosure/) contains recovered SolidWorks source and STL
exports for the enclosure. Several back variants survive, and no preferred
release has been confirmed.

## Historical revisions

[`archive/2024/`](archive/2024/) contains seven labelled PCB development
stages recovered from the source ZIP. They are retained for traceability, not
as alternatives to the current exemplar or learner project.

## Manufacturing outputs

[`manufacturing/gerbers/`](manufacturing/gerbers/) contains the previously
selected historical Gerber and drill files. Additional revision-specific sets
are retained with their matching source under `archive/2024/`.

Do not send any set to manufacture until the schematic and PCB pass ERC, DRC
and a human design review against an approved fabricated board.

## Bill of materials

[`bom/2024/bom.xlsx`](bom/2024/bom.xlsx) is the supplied 2024 bill of
materials. Prices, stock, supplier links and part substitutions have not yet
been reviewed for a future delivery.

## Legacy source

[`legacy/`](legacy/) contains an older binary `.dip` design file. Retain it for
traceability until its application, version and relationship to the KiCad
design are documented.

## Repository hygiene

KiCad caches, user preferences and automatic backups are not public project
source. The untouched `project_x.zip` source drop is retained under ignored
`private/source-archive/` as a lossless fallback. Unverified third-party
models and libraries are held separately under `private/licensing-review/`
and are not covered by the project licence.

Before the next release:

- convert the BOM into a supplier-neutral master list with optional supplier
  columns;
- confirm board dimensions and connector pinout;
- choose and validate an enclosure revision;
- add assembly instructions and known design limitations; and
- record the KiCad version used for final ERC and DRC.
