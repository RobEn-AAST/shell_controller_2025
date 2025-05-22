import sys
import os

# Add vendor directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "vendor"))

# Import all dependencies
__all__ = ["networkx", "transforms3d", "shapely", "ortools"]