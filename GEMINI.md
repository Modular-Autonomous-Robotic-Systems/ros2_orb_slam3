# SLAM Package Context

For a detailed overview of the software architecture, class hierarchy, and design patterns used in this package, please refer to [ARCHITECTURE.md](ARCHITECTURE.md).

## Python Coding Standards

This applies to all new python code. When writing Python code (especially launch files), only import top-level packages (e.g., `import launch`, `import launch_ros`, `import ament_index_python`) and use fully qualified module paths in the code instead of using `from ... import ...`. This helps make the code structure visible everywhere.
