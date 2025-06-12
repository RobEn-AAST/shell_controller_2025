import pytest


def test_namespace_alias():
    with pytest.raises(ImportError):
        from ai_src.vendor.networkx.networkx import nx


def test_namespace_nesting():
    with pytest.raises(ImportError):
        from ai_src.vendor.networkx.networkx import networkx
