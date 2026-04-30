# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0](https://github.com/jettify/uf-dps3xx/compare/v0.1.1...v0.2.0) - 2026-04-29

### Added

- Simplify API significantly, expose only two states. ([#17](https://github.com/jettify/uf-dps3xx/issues/17))

### Fixed

- [**breaking**] Make state transition recoverable. ([#15](https://github.com/jettify/uf-dps3xx/issues/15))

## [0.1.1](https://github.com/jettify/uf-dps3xx/compare/v0.1.0...v0.1.1) - 2026-04-05

### Other

- Improve README information.
- More badges to the readme.
- release v0.1.0 ([#12](https://github.com/jettify/uf-dps3xx/issues/12))

## [0.1.0](https://github.com/jettify/uf-dps3xx/releases/tag/v0.1.0) - 2026-04-04

### Added

- Add initialization timeout. ([#9](https://github.com/jettify/uf-dps3xx/issues/9))
- Add helper method to do init and calibration. ([#6](https://github.com/jettify/uf-dps3xx/issues/6))
- Add proper defmt support. ([#4](https://github.com/jettify/uf-dps3xx/issues/4))
- Rework polling API.
- Rework init logic, apply chip correction.
- Rename crate ([#3](https://github.com/jettify/uf-dps3xx/issues/3))
- Add CI ([#2](https://github.com/jettify/uf-dps3xx/issues/2))
- Always read scale register similar to arduio library and mintor clenaup. ([#1](https://github.com/jettify/uf-dps3xx/issues/1))
- Add README.mc
- Initial implementation.
- Initial project setup.

### Fixed

- Added silicon mitigation properly and proper wait for coef ready. ([#5](https://github.com/jettify/uf-dps3xx/issues/5))
- Add missing bus module.

### Other

- Fix Cargo.toml keywords.
- Prepare package for release. ([#11](https://github.com/jettify/uf-dps3xx/issues/11))
- [**breaking**] Idiomatic naming of constants.
- Improve code coverage with integraton test
- Improve code coverage with easy tests
- Idiomantic, infallable type casting
- Stricter linter rules aand improve cargo inclusion list. ([#10](https://github.com/jettify/uf-dps3xx/issues/10))
- Add simple example to show case API. ([#8](https://github.com/jettify/uf-dps3xx/issues/8))
- Update readme with usage exampels. ([#7](https://github.com/jettify/uf-dps3xx/issues/7))
- Port nicer justfile.
- Refactoring, move function across modules.
- Add license file.
- Move code around.
