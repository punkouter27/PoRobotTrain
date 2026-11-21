"""Compatibility entry point: the head-only sweep is now the ``neck_turn`` behaviour.

    ./run.sh robotcontrol.tools.head_sweep --target real --go
is the same as
    ./run.sh robotcontrol.behaviours.runner neck_turn --target real --go
"""

import sys

from ..behaviours.runner import main

if __name__ == "__main__":
    sys.exit(main(["neck_turn"] + sys.argv[1:]))
