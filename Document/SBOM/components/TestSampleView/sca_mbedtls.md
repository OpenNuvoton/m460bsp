# Mbed TLS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Mbed TLS component under `ThirdParty/mbedtls-3.1.0`.

## 1) Component Identity

- Component name (`name`): `Mbed TLS`
- Component type (`type`): `library`
- Manufacturer (`manufacturer.name`): `Arm Limited`
- Supplier (`supplier.name`): `Arm Limited`
- Author (`author`): `Mbed TLS Contributors`
- Version: `3.1.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `ThirdParty/mbedtls-3.1.0`
- Included top-level paths: `.github`, `3rdparty`, `ChangeLog.d`, `cmake`, `configs`, `docs`, `doxygen`, `include`, `library`, `programs`, `scripts`, `tests`, `visualc`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/mbedtls-3.1.0/include/mbedtls/build_info.h`
  - Defines `MBEDTLS_VERSION_MAJOR` as `3`.
  - Defines `MBEDTLS_VERSION_MINOR` as `1`.
  - Defines `MBEDTLS_VERSION_PATCH` as `0`.
  - Defines `MBEDTLS_VERSION_NUMBER` as `0x03010000`.
  - Defines `MBEDTLS_VERSION_STRING` as `3.1.0`.
  - Defines `MBEDTLS_VERSION_STRING_FULL` as `mbed TLS 3.1.0`.
  - Contains SPDX license identifier `Apache-2.0`.
- `ThirdParty/mbedtls-3.1.0/ChangeLog`
  - Contains the `mbed TLS 3.1.0` release entry.
- `ThirdParty/mbedtls-3.1.0/CMakeLists.txt`
  - Declares project version `3.1.0`.
- `ThirdParty/mbedtls-3.1.0/LICENSE`
  - Contains the Apache License, Version 2.0 text.
- `ThirdParty/mbedtls-3.1.0/README.md`
  - States that, unless indicated otherwise, files are provided under the Apache-2.0 license.
- `ThirdParty/mbedtls-3.1.0/include/mbedtls/version.h`
  - Contains SPDX license identifier `Apache-2.0`.

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = Apache-2.0`

Also preserve the upstream `LICENSE` file and source-file copyright/license notices.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `Mbed TLS`
- `version`: `3.1.0`
- `scope`: `required`
- `author`: `Mbed TLS Contributors`
- `manufacturer.name`: `Arm Limited`
- `supplier.name`: `Arm Limited`
- `description`: `Mbed TLS cryptographic and TLS library vendored in ThirdParty/mbedtls-3.1.0, including core library sources, public headers, configurations, documentation, programs, tests, scripts, build metadata, and bundled third-party code.`
- `licenses[0].license.id`: `Apache-2.0`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/mbedtls-3.1.0`
  - `integration = vendored_source`
  - `mbedtls_version = 3.1.0`
  - `mbedtls_version_number = 0x03010000`
  - `mbedtls_included_top_level_paths = .github; 3rdparty; ChangeLog.d; cmake; configs; docs; doxygen; include; library; programs; scripts; tests; visualc`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = Mbed TLS`
  - `bsp:evidence-file = Document/SBOM/components/sca_mbedtls.json`
  - `bsp:evidence-path = ThirdParty/mbedtls-3.1.0`
  - `bsp:version-evidence = ThirdParty/mbedtls-3.1.0/include/mbedtls/build_info.h; ThirdParty/mbedtls-3.1.0/ChangeLog; ThirdParty/mbedtls-3.1.0/CMakeLists.txt`
  - `bsp:license-evidence = ThirdParty/mbedtls-3.1.0/LICENSE; ThirdParty/mbedtls-3.1.0/README.md; ThirdParty/mbedtls-3.1.0/include/mbedtls/version.h`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/mbedtls@3.1.0?source=vendored&path=ThirdParty/mbedtls-3.1.0`
- `purl`: `pkg:generic/mbedtls@3.1.0`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/mbedtls@3.1.0?source=vendored&path=ThirdParty/mbedtls-3.1.0",
  "name": "Mbed TLS",
  "version": "3.1.0",
  "scope": "required",
  "author": "Mbed TLS Contributors",
  "manufacturer": {
    "name": "Arm Limited"
  },
  "supplier": {
    "name": "Arm Limited"
  },
  "purl": "pkg:generic/mbedtls@3.1.0",
  "description": "Mbed TLS cryptographic and TLS library vendored in ThirdParty/mbedtls-3.1.0, including core library sources, public headers, configurations, documentation, programs, tests, scripts, build metadata, and bundled third-party code.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/mbedtls-3.1.0" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "mbedtls_version", "value": "3.1.0" },
    { "name": "mbedtls_version_number", "value": "0x03010000" },
    { "name": "mbedtls_included_top_level_paths", "value": ".github; 3rdparty; ChangeLog.d; cmake; configs; docs; doxygen; include; library; programs; scripts; tests; visualc" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Mbed TLS" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/sca_mbedtls.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/mbedtls-3.1.0" },
    { "name": "bsp:version-evidence", "value": "ThirdParty/mbedtls-3.1.0/include/mbedtls/build_info.h; ThirdParty/mbedtls-3.1.0/ChangeLog; ThirdParty/mbedtls-3.1.0/CMakeLists.txt" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/mbedtls-3.1.0/LICENSE; ThirdParty/mbedtls-3.1.0/README.md; ThirdParty/mbedtls-3.1.0/include/mbedtls/version.h" }
  ]
}
```

## 7) Compliance Notes

- `manufacturer` identifies Arm Limited as the organization maintaining and releasing Mbed TLS 3.1.0; `supplier` records Arm Limited as the upstream supplier (with contributions from Mbed TLS Contributors).
- Keep original upstream copyright/license notices.
- Version evidence is taken from `include/mbedtls/build_info.h`, with cross-checks in `ChangeLog` and `CMakeLists.txt`.
- License evidence is taken from the vendored `LICENSE` file, the license note in `README.md`, and Apache-2.0 SPDX headers in the source tree.
