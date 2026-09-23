# SFUD Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored SFUD component under `ThirdParty/SFUD`.

## 1) Component Identity

- Component name (`name`): `SFUD`
- Component type (`type`): `library`
- Manufacturer (`manufacturer.name`): `Armink`
- Supplier (`supplier.name`): `Armink`
- Version: `1.1.0`
- License: `MIT` (SPDX)
- Evidence path: `ThirdParty/SFUD`
- Included top-level paths: `inc`, `port`, `src`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/SFUD/inc/sfud_def.h`
  - Identifies the component as part of the Serial Flash Universal Driver Library.
  - Defines `SFUD_SW_VERSION` as `1.1.0`.
  - Contains the MIT license text in the file header.
- `ThirdParty/SFUD/src/sfud.c`
  - Logs initialization with `SFUD_SW_VERSION`.
  - Contains the MIT license text in the file header.
- `ThirdParty/SFUD/inc/sfud.h`
  - Identifies the component as part of the Serial Flash Universal Driver Library.
  - Contains the MIT license text in the file header.
- `ThirdParty/SFUD/port/sfud_port.c`
  - Contains the platform port implementation.
  - Contains the MIT license text in the file header.

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = MIT`

This vendored copy does not include a separate top-level license file, so preserve the source-file license headers as the license evidence.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `SFUD`
- `version`: `1.1.0`
- `scope`: `required`
- `author`: `Armink`
- `manufacturer.name`: `Armink`
- `supplier.name`: `Armink`
- `description`: `Serial Flash Universal Driver Library vendored in ThirdParty/SFUD, including public headers, serial flash driver source, SFDP support, and platform port source.`
- `licenses[0].license.id`: `MIT`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/SFUD`
  - `integration = vendored_source`
  - `sfud_version = 1.1.0`
  - `sfud_included_top_level_paths = inc; port; src`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = Serial Flash Universal Driver Library`
  - `bsp:evidence-file = Document/SBOM/components/sca_sfud.json`
  - `bsp:evidence-path = ThirdParty/SFUD`
  - `bsp:version-evidence = ThirdParty/SFUD/inc/sfud_def.h; ThirdParty/SFUD/src/sfud.c`
  - `bsp:license-evidence = ThirdParty/SFUD/inc/sfud.h; ThirdParty/SFUD/inc/sfud_def.h; ThirdParty/SFUD/src/sfud.c; ThirdParty/SFUD/port/sfud_port.c`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/sfud@1.1.0?source=vendored&path=ThirdParty/SFUD`
- `purl`: `pkg:generic/sfud@1.1.0`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/sfud@1.1.0?source=vendored&path=ThirdParty/SFUD",
  "name": "SFUD",
  "version": "1.1.0",
  "scope": "required",
  "author": "Armink",
  "manufacturer": {
    "name": "Armink"
  },
  "supplier": {
    "name": "Armink"
  },
  "purl": "pkg:generic/sfud@1.1.0",
  "description": "Serial Flash Universal Driver Library vendored in ThirdParty/SFUD, including public headers, serial flash driver source, SFDP support, and platform port source.",
  "licenses": [
    {
      "license": {
        "id": "MIT"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/SFUD" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "sfud_version", "value": "1.1.0" },
    { "name": "sfud_included_top_level_paths", "value": "inc; port; src" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Serial Flash Universal Driver Library" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_sfud.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/SFUD" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/SFUD/inc/sfud_def.h; ThirdParty/SFUD/src/sfud.c" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/SFUD/inc/sfud.h; ThirdParty/SFUD/inc/sfud_def.h; ThirdParty/SFUD/src/sfud.c; ThirdParty/SFUD/port/sfud_port.c" }
  ]
}
```

## 7) Compliance Notes

- `manufacturer` identifies Armink as the author/developer of SFUD; `supplier` records Armink as the upstream supplier.
- Keep original upstream copyright/license notices.
- Version evidence is taken from `inc/sfud_def.h`, with usage cross-checked in `src/sfud.c`.
- License evidence is taken from the MIT license headers in the vendored source files.
