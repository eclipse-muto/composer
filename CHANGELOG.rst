^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package muto_composer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial release preparation for packages.ros.org
* Renamed package from ``composer`` to ``muto_composer`` to avoid name collisions
* Added pipeline-based stack deployment and orchestration engine
* Added pluggable stack handlers for archive, JSON, and Ditto content types
* Added provision, compose, and launch plugin architecture
* Added process monitoring with crash detection and event handling
* Added rollback support with persistent state management
* Added launch file introspection and composable node support
* Added managed (lifecycle) node support
* Added watchdog for deployment health monitoring
* Standardized copyright headers to SPDX EPL-2.0 format
* Added ruff and mypy linting configuration
* Added pre-commit hooks for code quality enforcement
* Set python_requires>=3.10 for Humble/Jazzy compatibility
* Contributors: Alp Sarıca, Deniz Memis, Ibrahim Sel, Naci Dai, Nazli Eker, Samet Karabulut
