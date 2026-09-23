# FreeRTOS-Kernel Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored FreeRTOS component under `ThirdParty/FreeRTOS`.

## 1) Component Identity

- Component name (`name`): `FreeRTOS-Kernel`
- Component type (`type`): `library`
- Manufacturer (`manufacturer.name`): `Amazon.com, Inc.`
- Supplier (`supplier.name`): `Amazon.com, Inc.`
- Author / project: `Amazon.com, Inc. (FreeRTOS project)`
- Version: `10.0.0`
- License: `MIT` (SPDX)
- Evidence path: `ThirdParty/FreeRTOS`
- Included top-level paths: `Demo`, `License`, `Source`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`
  - Header shows `FreeRTOS Kernel V10.0.0`
- `ThirdParty/FreeRTOS/Source/tasks.c`
  - Header shows `FreeRTOS Kernel V10.0.0`
- `ThirdParty/FreeRTOS/Source/queue.c`
  - Header shows `FreeRTOS Kernel V10.0.0`
- `ThirdParty/FreeRTOS/Source/list.c`
  - Header shows `FreeRTOS Kernel V10.0.0`
- `ThirdParty/FreeRTOS/Source/timers.c`
  - Header shows `FreeRTOS Kernel V10.0.0`
- `ThirdParty/FreeRTOS/License/license.txt`
  - License file states that the FreeRTOS kernel is released under the MIT open source license.

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = MIT`

Also preserve upstream license headers and the vendored license text.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `FreeRTOS-Kernel`
- `version`: `10.0.0`
- `scope`: `required`
- `author`: `Amazon.com, Inc. (FreeRTOS project)`
- `manufacturer.name`: `Amazon.com, Inc.`
- `supplier.name`: `Amazon.com, Inc.`
- `description`: `FreeRTOS Kernel vendored in ThirdParty/FreeRTOS, including kernel source, public headers, GCC/IAR/RVDS portable layers, memory managers, license text, and demo/common example files.`
- `licenses[0].license.id`: `MIT`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/FreeRTOS`
  - `integration = vendored_source`
  - `freertos_kernel_version = 10.0.0`
  - `freertos_included_top_level_paths = Demo; License; Source`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = FreeRTOS Kernel (Amazon FreeRTOS project)`
  - `bsp:evidence-file = Document/SBOM/components/sca_freertos.json`
  - `bsp:evidence-path = ThirdParty/FreeRTOS`
  - `bsp:version-evidence = ThirdParty/FreeRTOS/Source/include/FreeRTOS.h; ThirdParty/FreeRTOS/Source/tasks.c; ThirdParty/FreeRTOS/Source/queue.c; ThirdParty/FreeRTOS/Source/list.c; ThirdParty/FreeRTOS/Source/timers.c`
  - `bsp:license-evidence = ThirdParty/FreeRTOS/License/license.txt`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/freertos-kernel@10.0.0?source=vendored&path=ThirdParty/FreeRTOS`
- `purl`: `pkg:generic/freertos-kernel@10.0.0`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/freertos-kernel@10.0.0?source=vendored&path=ThirdParty/FreeRTOS",
  "name": "FreeRTOS-Kernel",
  "version": "10.0.0",
  "scope": "required",
  "author": "Amazon.com, Inc. (FreeRTOS project)",
  "manufacturer": {
    "name": "Amazon.com, Inc."
  },
  "supplier": {
    "name": "Amazon.com, Inc."
  },
  "purl": "pkg:generic/freertos-kernel@10.0.0",
  "description": "FreeRTOS Kernel vendored in ThirdParty/FreeRTOS, including kernel source, public headers, GCC/IAR/RVDS portable layers, memory managers, license text, and demo/common example files.",
  "licenses": [
    {
      "license": {
        "id": "MIT"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/FreeRTOS" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "freertos_kernel_version", "value": "10.0.0" },
    { "name": "freertos_included_top_level_paths", "value": "Demo; License; Source" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "FreeRTOS Kernel (Amazon FreeRTOS project)" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_freertos.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/FreeRTOS" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/FreeRTOS/Source/include/FreeRTOS.h; ThirdParty/FreeRTOS/Source/tasks.c; ThirdParty/FreeRTOS/Source/queue.c; ThirdParty/FreeRTOS/Source/list.c; ThirdParty/FreeRTOS/Source/timers.c" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/FreeRTOS/License/license.txt" }
  ]
}
```

## 7) Compliance Notes

- `manufacturer` identifies Amazon.com, Inc. as the organization maintaining and publishing FreeRTOS; `supplier` records Amazon.com, Inc. as the upstream supplier.
- Keep original upstream copyright/license notices.
- Version evidence is taken from the vendored kernel headers and source files.
- License evidence is taken from the vendored `License/license.txt` file.
- If producing a finer-grained SBOM, represent major units as separate components and declare dependencies between them.
