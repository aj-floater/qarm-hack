# Python installation guide

This hackathon project relies on Python versions between 3.10 and 3.13
(inclusive). Both PyBullet and Panda3D publish wheels in that range, so aim for
the newest release available on your platform.

## Windows

1. Download the latest 3.x installer from [python.org/downloads](https://www.python.org/downloads/windows/).
2. Run the installer, check **Add python.exe to PATH**, and pick **Customize
   installation** if you need to change the install location.
3. When the installer finishes, open **Command Prompt** and verify:
   ```powershell
   python --version
   ```
   It should report a version between 3.10.x and 3.13.x.
4. (Optional) Install the [VS Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
   if you skipped them earlier. PyBullet will compile extension modules without
   them, but having the toolchain avoids linker issues on some systems.

## macOS

1. Install the Xcode Command Line Tools if you have not already:
   ```bash
   xcode-select --install
   ```
2. Pick a Python 3 release via either:
   - [python.org/macOS installer](https://www.python.org/downloads/macos/)
   - `brew install python@3.11` (or similar) if you use Homebrew
3. Confirm the shell is running the expected interpreter:
   ```bash
   python3 --version
   ```
   If the version is outside the supported range, adjust your PATH or symlinks
   so `python3` resolves to the correct install.
4. When using Homebrew, remember that macOS ships its own Python. Always use
   the full path (e.g. `/opt/homebrew/bin/python3`) when creating the virtual
   environment so packages end up in the right place.

## Verifying pip

Once Python is installed, ensure `pip` matches the interpreter:
```bash
python -m pip --version
```
If this fails, reinstall Python or run `python -m ensurepip --upgrade`.

Keeping the interpreter within the supported version range and using the
interpreter’s `pip` command avoids module mismatch errors when you later run
`pip install -e .` inside the project virtual environment.
