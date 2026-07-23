# Contributing

Thank you for improving a Fleming Society project. These conventions are
designed to make repositories familiar to students, instructors and future
committees.

## Where files belong

| Content | Location |
| --- | --- |
| Learner handouts and tutorials | `docs/learner/` |
| Instructor preparation and delivery notes | `docs/instructor/` |
| Firmware and software | `firmware/` or `software/` |
| Editable CAD, schematic and PCB source | `hardware/` |
| Generated fabrication files | `hardware/manufacturing/` |
| Older source formats retained for reference | `hardware/legacy/` |
| Third-party licence texts | `third_party/` |

Use lower-case, descriptive, hyphen-separated names for new folders and
documents. Follow tool requirements where filenames are significant; for
example, an Arduino sketch directory must match its `.ino` filename.

## Privacy

Never commit attendance registers, names, personal email addresses, health
readings, consent forms or other participant records. Store operational data
in an approved UCL or Students' Union system with appropriate access controls.

The local `private/` directory is ignored as an additional safeguard, but it
is not a substitute for approved secure storage.

## Licensing

By contributing original work, you agree that it may be distributed under the
repository's [MIT License](LICENSE). Only contribute material that you created
or have permission to redistribute.

Keep third-party copyright notices and licence terms intact. Add the source,
author and licence to [NOTICE.md](NOTICE.md), and preserve the licence text
under `third_party/` when material is vendored into the repository.

## Engineering source and exports
- Commit editable source files, not only PDFs, screenshots or fabrication
  outputs.
- Keep generated outputs separate from source.
- Do not commit tool caches, automatic backups or user-specific preferences.
- Record the application and version last used to validate each design.
- Run relevant compile, ERC and DRC checks before marking an output as ready.
- Publish approved manufacturing bundles as versioned releases where possible.

## Teaching changes

When changing a session:

1. State the intended audience, prerequisites, duration and learning outcomes.
2. Update both learner and instructor materials when the delivery changes.
3. Check links and instructions from a fresh checkout.
4. Record hardware, dependency and software versions.
5. Include safety, accessibility and troubleshooting guidance.

## Pull requests

Use a focused branch and pull request. Explain:

- what changed and why;
- which session or hardware revision is affected;
- how the change was tested;
- whether manufacturing files were regenerated; and
- any remaining safety or technical limitations.
