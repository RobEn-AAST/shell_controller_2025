import pytest


def test_utils_namespace():
    """Ensure objects are not unintentionally exposed in utils namespace."""
    with pytest.raises(ImportError):
        from ai_src.vendor.networkx.networkx.utils import nx
    with pytest.raises(ImportError):
        from ai_src.vendor.networkx.networkx.utils import sys
    with pytest.raises(ImportError):
        from ai_src.vendor.networkx.networkx.utils import defaultdict, deque
