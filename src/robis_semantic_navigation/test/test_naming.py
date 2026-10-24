from robis_semantic_navigation.naming import normalize_query, requires_vlm


def test_alias_and_whitespace_normalization():
    assert normalize_query("  Cadeira  ", {"cadeira": "chair"}) == "chair"


def test_vlm_only_for_open_or_qualified_queries():
    assert not requires_vlm("chair", "chair")
    assert not requires_vlm("cadeira", "chair")
    assert requires_vlm("red chair", "red chair")
