"""
Generate the stubs of the compiled `dqrobotics._dqrobotics` module, to be shipped inside the `dqrobotics` package.

The public modules of dqrobotics (e.g. `dqrobotics.robot_modeling`) are Python files that do
`from .._dqrobotics._robot_modeling import *`, so type checkers read them directly. Only the compiled module
needs stubs, which pybind11-stubgen writes next to it, as below. Together with `dqrobotics/py.typed`, this makes
dqrobotics an inline-typed package (PEP 561).

dqrobotics
├── py.typed
├── __init__.py
├── robot_modeling
│   └── __init__.py
├── ...
├── _dqrobotics.<platform>.so   (compiled module)
└── _dqrobotics                 (generated stubs)
    ├── __init__.pyi
    ├── _robot_modeling.pyi
    └── ...

Usage (with dqrobotics and pybind11-stubgen installed in the current environment)
    python regenerate_stubs.py

Author: Murilo M. Marinho
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

OUTPUT_DIR: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dqrobotics", "_dqrobotics")
MODULE: str = "dqrobotics._dqrobotics"


def remove_keyword_identifiers(stub_file: str) -> None:
    """
    Remove the attributes named `None` from a stub file.
    `ControlObjective.None` is valid at runtime, through getattr(), but `None: ...` is a syntax error in a stub.
    :param stub_file: The path to the stub file.
    """
    with open(stub_file) as f:
        content = f.read()
    content = re.sub(r"^[ \t]*None: .*\n", "", content, flags=re.MULTILINE)
    content = content.replace("'None', ", "")
    with open(stub_file, "w") as f:
        f.write(content)


def replace_once(content: str, old: str, new: str) -> str:
    """
    Replace a snippet of generated stub that must appear exactly once, so that changes in pybind11-stubgen's output are
    noticed instead of silently ignored.
    """
    if content.count(old) != 1:
        raise RuntimeError(f"Expected exactly one occurrence of:\n{old}")
    return content.replace(old, new)


def complete_dq_comparison(stub_file: str) -> None:
    """
    Make the stub of `DQ` consistent with `object`, as expected by type checkers.
    - `__eq__` and `__ne__` also accept any other object at runtime, for which pybind11 returns NotImplemented and Python
      falls back to the default comparison. That last overload cannot be expressed in the bindings with a `bool` result.
    - `__hash__` is None because `DQ` defines `__eq__`. This is declared as in typeshed (e.g. `list`), which needs
      the ignore because `object.__hash__` is a method.
    :param stub_file: The path to the stub file of `dqrobotics._dqrobotics`.
    """
    with open(stub_file) as f:
        content = f.read()
    content = replace_once(content,
                           "    __hash__: typing.ClassVar[None] = None\n",
                           "    __hash__: typing.ClassVar[None] = None  # type: ignore[assignment]\n")
    for method, result in (("__eq__", "False"), ("__ne__", "True")):
        last_overload = re.findall(rf'    def {method}\(self, arg0: typing\.SupportsFloat \| typing\.SupportsIndex\) -> bool:\n'
                                   rf'        """\n.*?\n        """\n', content, flags=re.DOTALL)
        if len(last_overload) != 1:
            raise RuntimeError(f"Expected exactly one scalar overload of DQ.{method}")
        content = replace_once(content, last_overload[0], last_overload[0] +
                               "    @typing.overload\n"
                               f"    def {method}(self, other: object) -> bool:\n"
                               '        """\n'
                               f"        Returns {result} for any other object, through Python's default comparison.\n"
                               '        """\n')
    with open(stub_file, "w") as f:
        f.write(content)


def normalize_empty_all(stub_dir: str) -> None:
    """
    Write the empty `__all__` of the generated stubs as `[]`. pybind11-stubgen writes `list()`, which type checkers
    do not evaluate, so they cannot tell which names the module exports.
    :param stub_dir: The directory with the stub files.
    """
    for root, _, files in os.walk(stub_dir):
        for name in files:
            if name.endswith(".pyi"):
                stub_file = os.path.join(root, name)
                with open(stub_file) as f:
                    content = f.read()
                with open(stub_file, "w") as f:
                    f.write(content.replace("__all__: list[str] = list()", "__all__: list[str] = []"))


def main() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        # Run from temp_dir so that the source folder `dqrobotics`, which lacks the compiled module, does not shadow
        # the installed package.
        subprocess.check_call([sys.executable, "-m", "pybind11_stubgen", MODULE, "--output-dir", temp_dir,
                               "--exit-code"],
                              cwd=temp_dir)
        shutil.rmtree(OUTPUT_DIR, ignore_errors=True)
        shutil.copytree(os.path.join(temp_dir, *MODULE.split(".")), OUTPUT_DIR)
    normalize_empty_all(OUTPUT_DIR)
    remove_keyword_identifiers(os.path.join(OUTPUT_DIR, "_robot_control.pyi"))
    complete_dq_comparison(os.path.join(OUTPUT_DIR, "__init__.pyi"))


if __name__ == "__main__":
    main()
