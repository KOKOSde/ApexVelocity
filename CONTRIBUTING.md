# Contributing to ApexVelocity

Thank you for your interest in contributing! We welcome contributions from the community.

## Code of Conduct

Be respectful, professional, and collaborative. We value diverse perspectives and constructive feedback.

## How to Report Bugs

Use GitHub Issues with the bug report template. Include:
- Environment (OS, Python version, compiler)
- Steps to reproduce
- Expected vs actual behavior
- Error messages and logs

## How to Submit Changes

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes with clear commit messages
4. Write or update tests
5. Ensure all tests pass: `pytest python/tests/ && ctest`
6. Run linters: `black . && isort . && flake8`
7. Submit a pull request

## Development Setup

### C++ Core

```bash
cd core && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DAPEXVELOCITY_BUILD_TESTS=ON
make -j"$(nproc)"
ctest --output-on-failure
```

### Python Package

```bash
pip install -r requirements.txt
pip install -r requirements-dev.txt
pip install -e python/
pytest python/tests/ -v
```

### Go Server

```bash
cd server
go build ./cmd/apex-server
go test ./...
```

## Code Style

- **C++**: Follow C++20 Core Guidelines, use clang-format
- **Python**: PEP 8, use black and isort
- **Go**: Use gofmt and golangci-lint

## Testing Requirements

All PRs must:
- Pass existing tests
- Add tests for new features
- Maintain >80% code coverage
- Pass linting checks

## Pull Request Review Process

1. Automated CI must pass
2. At least one maintainer approval required
3. Address all review comments
4. Squash commits before merge

## Questions?

Open a GitHub Discussion or reach out to maintainers.


