"""
NetworkX
========

NetworkX is a Python package for the creation, manipulation, and study of the
structure, dynamics, and functions of complex networks.

See https://networkx.org for complete documentation.
"""

__version__ = "3.4.2"


# These are imported in order as listed
from ai_src.vendor.networkx.networkx.lazy_imports import _lazy_import

from ai_src.vendor.networkx.networkx.exception import *

from ai_src.vendor.networkx.networkx import utils
from ai_src.vendor.networkx.networkx.utils import _clear_cache, _dispatchable

# load_and_call entry_points, set configs
config = utils.backends._set_configs_from_environment()
utils.config = utils.configs.config = config  # type: ignore[attr-defined]

from ai_src.vendor.networkx.networkx import classes
from ai_src.vendor.networkx.networkx.classes import filters
from ai_src.vendor.networkx.networkx.classes import *

from ai_src.vendor.networkx.networkx import convert
from ai_src.vendor.networkx.networkx.convert import *

from ai_src.vendor.networkx.networkx import convert_matrix
from ai_src.vendor.networkx.networkx.convert_matrix import *

from ai_src.vendor.networkx.networkx import relabel
from ai_src.vendor.networkx.networkx.relabel import *

from ai_src.vendor.networkx.networkx import generators
from ai_src.vendor.networkx.networkx.generators import *

from ai_src.vendor.networkx.networkx import readwrite
from ai_src.vendor.networkx.networkx.readwrite import *

# Need to test with SciPy, when available
from ai_src.vendor.networkx.networkx import algorithms
from ai_src.vendor.networkx.networkx.algorithms import *

from ai_src.vendor.networkx.networkx import linalg
from ai_src.vendor.networkx.networkx.linalg import *

from ai_src.vendor.networkx.networkx import drawing
from ai_src.vendor.networkx.networkx.drawing import *
