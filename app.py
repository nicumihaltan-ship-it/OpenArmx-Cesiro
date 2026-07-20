"""Entry point for the OpenArmX RobStride configurator.

    python app.py [--debug]
"""

from __future__ import annotations

import argparse
import logging
import sys

from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug", action="store_true",
                        help="verbose logging, including per-frame decode errors")
    parser.add_argument("--selftest", action="store_true",
                        help="verify this build works, then exit")
    args = parser.parse_args()

    if args.selftest:
        from selftest import run
        return run()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    app = QApplication(sys.argv)
    app.setApplicationName("OpenArmX RobStride Configurator")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
