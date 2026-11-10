"""Temporary build directory with bounded retries for Windows executable locks."""
from contextlib import contextmanager
from pathlib import Path
import shutil
import tempfile
import time
import warnings


@contextmanager
def test_directory(prefix):
    root = Path(tempfile.gettempdir()).resolve()
    directory = Path(tempfile.mkdtemp(prefix=prefix, dir=root)).resolve()
    # Cleanup can only target the newly created child, never the temp root.
    if directory.parent != root or directory == root:
        raise RuntimeError(f"Unexpected temporary directory: {directory}")
    try:
        yield str(directory)
    finally:
        for attempt in range(5):
            try:
                shutil.rmtree(directory)
                break
            except FileNotFoundError:
                break
            except OSError as error:
                if attempt == 4:
                    # Do not mask a compiler/test exception or change a passed
                    # test into a failure because antivirus still holds the exe.
                    warnings.warn(f"Temporary files could not be removed: {directory}: {error}",
                                  RuntimeWarning, stacklevel=2)
                else:
                    time.sleep(0.2 * (attempt + 1))
