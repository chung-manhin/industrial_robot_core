# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- CI smoke script and matrix workflow validating build/tests/benchmark/demo.
- IK regression tests for joint limits and near-singular stability.
- Benchmark seed-mode comparison (`--single-seed` / `--multi-seed`).

### Changed
- README quick start and CI documentation.

## [0.1.0] - 2026-02-08

### Added
- Initial C++17 robot motion core (FK/Jacobian/DLS IK).
- CTest suite and IK benchmark.
- Python bindings and demo that renders a pose image.
