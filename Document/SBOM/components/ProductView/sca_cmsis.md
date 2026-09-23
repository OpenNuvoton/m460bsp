# CMSIS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS component under `Library/CMSIS`.

## 1) Component Identity

- Component name (`name`): `CMSIS`
- Component type (`type`): `library`
- Manufacturer (`manufacturer.name`): `Arm Limited`
- Supplier (`supplier.name`): `Arm Limited`
- Version: `5.1.1`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS`
- Included top-level paths: `Core`, `Core_A`, `DAP`, `Documentation`, `Driver`, `DSP_Lib`, `Include`, `Lib`, `Pack`, `RTOS`, `RTOS2`, `SVD`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Library/CMSIS/Documentation/General/html/index.html`
  - General CMSIS documentation shows package `Version 5.1.1`
- `Library/CMSIS/Documentation/General/html/cm_revisionHistory.html`
  - Revision history shows package `5.1.1`

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = Apache-2.0`

Also preserve upstream copyright and license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `CMSIS`
- `version`: `5.1.1`
- `scope`: `required`
- `author`: `Arm Limited`
- `manufacturer.name`: `Arm Limited`
- `supplier.name`: `Arm Limited`
- `description`: `Arm CMSIS vendored in Library/CMSIS, including CMSIS-Core(M), CMSIS-Core(A), CMSIS-DSP Library, CMSIS-Driver, CMSIS-RTOS and CMSIS-RTOS2 API headers, CMSIS-DAP firmware sources, SVD files, Pack metadata/examples, templates, libraries, and documentation.`
- `licenses[0].license.id`: `Apache-2.0`
- `properties` (recommended custom properties):
  - `src_path = Library/CMSIS`
  - `integration = vendored_source`
  - `cmsis_included_top_level_paths = Core; Core_A; DAP; Documentation; Driver; DSP_Lib; Include; Lib; Pack; RTOS; RTOS2; SVD`
  - `cmsis_package_version = 5.1.1`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = Arm CMSIS`
  - `bsp:evidence-file = Document/SBOM/cmsis/sca_cmsis.json`
  - `bsp:evidence-path = Library/CMSIS`
  - `bsp:version-evidence = Library/CMSIS/Documentation/General/html/index.html; Library/CMSIS/Documentation/General/html/cm_revisionHistory.html`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/cmsis@5.1.1?source=vendored&path=Library/CMSIS`
- `purl`: `pkg:generic/cmsis@5.1.1`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/cmsis@5.1.1?source=vendored&path=Library/CMSIS",
  "name": "CMSIS",
  "version": "5.1.1",
  "scope": "required",
  "author": "Arm Limited",
  "manufacturer": {
    "name": "Arm Limited"
  },
  "supplier": {
    "name": "Arm Limited"
  },
  "purl": "pkg:generic/cmsis@5.1.1",
  "description": "Arm CMSIS vendored in Library/CMSIS, including CMSIS-Core(M), CMSIS-Core(A), CMSIS-DSP Library, CMSIS-Driver, CMSIS-RTOS and CMSIS-RTOS2 API headers, CMSIS-DAP firmware sources, SVD files, Pack metadata/examples, templates, libraries, and documentation.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "Library/CMSIS" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "cmsis_included_top_level_paths", "value": "Core; Core_A; DAP; Documentation; Driver; DSP_Lib; Include; Lib; Pack; RTOS; RTOS2; SVD" },
    { "name": "cmsis_package_version", "value": "5.1.1" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Arm CMSIS" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/cmsis/sca_cmsis.json" },
    { "name": "bsp:evidence-path", "value": "Library/CMSIS" },
    { "name": "bsp:version-evidence", "value": "Library/CMSIS/Documentation/General/html/index.html; Library/CMSIS/Documentation/General/html/cm_revisionHistory.html" }
  ]
}
```

## 7) Compliance Notes

- `manufacturer` identifies Arm Limited as the organization that created CMSIS; `supplier` records Arm Limited as the upstream supplier in this component description.
- The component-level `manufacturer` field in this example uses the CycloneDX 1.6 schema.
- Keep original upstream copyright/license notices.
- Use `5.1.1` as the CMSIS package version for the top-level SBOM component.
- CMSIS in this BSP contains multiple CMSIS subareas, including Core(M), Core(A), DSP Library, Driver, RTOS, RTOS2, DAP firmware sources, SVD files, Pack metadata/examples, templates, libraries, and documentation.
- The current `Library/CMSIS` tree does not include the upstream `Utilities` directory.
- If producing a finer-grained SBOM, represent major CMSIS subcomponents separately when product policy requires component-level version tracking.
