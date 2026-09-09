# Journey 6 license and redistribution boundary

The repository's Apache-2.0 license applies to project-authored source,
manifests, tests, and deployment tooling. It does not relicense third-party
weights, datasets, model architectures, OpenExplorer, HUCP/DNN runtime, BSP,
sysroots, or vendor samples.

Every pretrained candidate must record the source URI, immutable revision,
filename, artifact SHA-256, model-card digest, declared license, architecture
license, training-code license, dataset terms, weight redistribution terms, and
the intended distribution mode. A model-card `apache-2.0` tag is evidence to
review, not sufficient proof that every layer can be redistributed.

Statuses are:

- `competition_open_source`: terms cover the intended competition release;
- `commercial_permissive`: redistribution and use are explicitly compatible;
- `research_only`: usable for local research but not a shipped bundle;
- `blocked_license`: missing or incompatible terms.

No candidate in the current pretrained manifest is shipped by Git. Downloaded
weights remain in `.workspace/models` or another operator-selected artifact
root. A release bundle may install a weight only after the license audit is
complete and the selected distribution mode is allowed.

Official Journey 6 SDK packages, images, HBM sanity models, headers, libraries,
and documentation remain outside Git and are used under Horizon's supplied
terms. Hashes and non-proprietary inventory metadata may be recorded as
evidence; the proprietary payload is not copied into the repository or release
ZIP.
