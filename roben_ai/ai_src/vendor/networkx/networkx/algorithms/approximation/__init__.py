"""Approximations of graph properties and Heuristic methods for optimization.

The functions in this class are not imported into the top-level ``networkx``
namespace so the easiest way to use them is with::

    >>> from ai_src.vendor.networkx.networkx.algorithms import approximation

Another option is to import the specific function with
``from ai_src.vendor.networkx.networkx.algorithms.approximation import function_name``.

"""

from ai_src.vendor.networkx.networkx.algorithms.approximation.clustering_coefficient import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.clique import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.connectivity import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.distance_measures import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.dominating_set import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.kcomponents import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.matching import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.ramsey import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.steinertree import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.traveling_salesman import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.treewidth import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.vertex_cover import *
from ai_src.vendor.networkx.networkx.algorithms.approximation.maxcut import *
