import pytest

import planning_service_client.native.types as types

# ──────────────────────────────────────────────────────────────────────────────
# Value
# ──────────────────────────────────────────────────────────────────────────────


class TestValueConstruction:
    def test_default_is_none(self):
        v = types.Value()
        assert v.is_none
        assert not v.is_string
        assert not v.is_double
        assert not v.is_bool
        assert not v.is_list
        assert not v.is_state

    def test_string_value(self):
        v = types.Value("hello")
        assert v.is_string
        assert not v.is_none
        assert v.as_string == "hello"

    def test_double_value(self):
        v = types.Value(3.14)
        assert v.is_double
        assert v.as_double == pytest.approx(3.14)

    def test_bool_value_true(self):
        v = types.Value(True)
        assert v.is_bool
        assert v.as_bool is True

    def test_bool_value_false(self):
        v = types.Value(False)
        assert v.is_bool
        assert v.as_bool is False

    def test_list_value(self):
        v = types.Value([types.Value(1.0), types.Value(2.0)])
        assert v.is_list
        items = v.as_list
        assert len(items) == 2
        assert items[0].as_double == pytest.approx(1.0)
        assert items[1].as_double == pytest.approx(2.0)

    def test_empty_list_value(self):
        v = types.Value([])
        assert v.is_list
        assert v.as_list == []

    def test_state_value(self):
        s = types.State()
        s.add("x", 42.0)
        v = types.Value(s)
        assert v.is_state
        assert v.as_state["x"].as_double == pytest.approx(42.0)


class TestValueAccessorErrors:
    def test_wrong_accessor_raises(self):
        v = types.Value(1.0)
        with pytest.raises((RuntimeError, Exception)):
            _ = v.as_string

    def test_none_accessor_raises(self):
        v = types.Value()
        with pytest.raises((RuntimeError, Exception)):
            _ = v.as_double


class TestValueIndex:
    # variant index matches order in Value::value_type:
    # monostate=0, string=1, double=2, bool=3, vector=4, State=5
    def test_none_index(self):
        assert types.Value().index == 0

    def test_string_index(self):
        assert types.Value("x").index == 1

    def test_double_index(self):
        assert types.Value(0.0).index == 2

    def test_bool_index(self):
        assert types.Value(False).index == 3

    def test_list_index(self):
        assert types.Value([]).index == 4

    def test_state_index(self):
        assert types.Value(types.State()).index == 5


class TestValueCompactString:
    def test_returns_string(self):
        v = types.Value(7.5)
        s = v.to_compact_string()
        assert isinstance(s, str)
        assert len(s) > 0


class TestValueProtoRoundtrip:
    @pytest.mark.parametrize(
        "value",
        [
            types.Value(),
            types.Value("round_trip"),
            types.Value(2.718),
            types.Value(True),
            types.Value(False),
        ],
    )
    def test_scalar_roundtrip(self, value):
        recovered = types.Value.from_proto_bytes(value.to_proto_bytes())
        assert recovered.index == value.index

    def test_string_content_preserved(self):
        v = types.Value("preserved")
        r = types.Value.from_proto_bytes(v.to_proto_bytes())
        assert r.as_string == "preserved"

    def test_double_content_preserved(self):
        v = types.Value(1.23456789)
        r = types.Value.from_proto_bytes(v.to_proto_bytes())
        assert r.as_double == pytest.approx(1.23456789)

    def test_bool_content_preserved(self):
        for b in (True, False):
            v = types.Value(b)
            r = types.Value.from_proto_bytes(v.to_proto_bytes())
            assert r.as_bool == b

    def test_list_content_preserved(self):
        v = types.Value([types.Value(10.0), types.Value("item")])
        r = types.Value.from_proto_bytes(v.to_proto_bytes())
        assert r.is_list
        items = r.as_list
        assert items[0].as_double == pytest.approx(10.0)
        assert items[1].as_string == "item"

    def test_nested_state_roundtrip(self):
        s = types.State()
        s.add("k", 99.0)
        v = types.Value(s)
        r = types.Value.from_proto_bytes(v.to_proto_bytes())
        assert r.is_state
        assert r.as_state["k"].as_double == pytest.approx(99.0)


# ──────────────────────────────────────────────────────────────────────────────
# State
# ──────────────────────────────────────────────────────────────────────────────


class TestStateConstruction:
    def test_default_is_empty(self):
        s = types.State()
        assert s.keys() == []
        assert s.values() == []
        assert s.items() == []


class TestStateAddAndAccess:
    def test_add_and_contains(self):
        s = types.State()
        s.add("foo", 1.0)
        assert s.contains("foo")
        assert not s.contains("bar")

    def test_add_with_in_operator(self):
        s = types.State()
        s.add("x", "hello")
        assert "x" in s
        assert "y" not in s

    def test_getitem(self):
        s = types.State()
        s.add("pi", 3.14159)
        assert s["pi"].as_double == pytest.approx(3.14159)

    def test_setitem(self):
        s = types.State()
        s["key"] = "value"
        assert s["key"].as_string == "value"

    def test_setitem_overwrites(self):
        s = types.State()
        s["k"] = 1.0
        s["k"] = 2.0
        assert s["k"].as_double == pytest.approx(2.0)

    def test_getitem_missing_raises(self):
        s = types.State()
        with pytest.raises((KeyError, RuntimeError, Exception)):
            _ = s["missing"]

    def test_erase(self):
        s = types.State()
        s.add("to_remove", True)
        assert s.contains("to_remove")
        s.erase("to_remove")
        assert not s.contains("to_remove")


class TestStateIterationMethods:
    @pytest.fixture
    def populated(self):
        s = types.State()
        s.add("a", 1.0)
        s.add("b", "two")
        s.add("c", True)
        return s

    def test_keys(self, populated):
        assert sorted(populated.keys()) == ["a", "b", "c"]

    def test_values_count(self, populated):
        assert len(populated.values()) == 3

    def test_items_count(self, populated):
        assert len(populated.items()) == 3

    def test_items_are_pairs(self, populated):
        for key, val in populated.items():
            assert isinstance(key, str)
            assert isinstance(val, types.Value)


class TestStateCalcHash:
    def test_same_state_same_hash(self):
        s1 = types.State()
        s1.add("x", 1.0)
        s2 = types.State()
        s2.add("x", 1.0)
        assert s1.calc_hash() == s2.calc_hash()

    def test_different_state_different_hash(self):
        s1 = types.State()
        s1.add("x", 1.0)
        s2 = types.State()
        s2.add("x", 2.0)
        assert s1.calc_hash() != s2.calc_hash()

    def test_hash_is_integer(self):
        s = types.State()
        s.add("k", 0.0)
        assert isinstance(s.calc_hash(), int)


class TestStateToCompactString:
    def test_returns_string(self):
        s = types.State()
        s.add("k", 1.0)
        result = s.to_compact_string()
        assert isinstance(result, str)
        assert len(result) > 0

    def test_with_hash(self):
        s = types.State()
        s.add("k", 1.0)
        without = s.to_compact_string(include_hash=False)
        with_hash = s.to_compact_string(include_hash=True)
        # The version with the hash should be longer
        assert len(with_hash) > len(without)


class TestStateIsSubsetOf:
    def test_empty_is_subset_of_anything(self):
        empty = types.State()
        full = types.State()
        full.add("x", 1.0)
        assert empty.is_subset_of(full)

    def test_equal_states_are_subsets_of_each_other(self):
        s1 = types.State()
        s1.add("x", 1.0)
        s2 = types.State()
        s2.add("x", 1.0)
        assert s1.is_subset_of(s2)
        assert s2.is_subset_of(s1)

    def test_proper_subset(self):
        sub = types.State()
        sub.add("x", 1.0)
        full = types.State()
        full.add("x", 1.0)
        full.add("y", 2.0)
        assert sub.is_subset_of(full)
        assert not full.is_subset_of(sub)

    def test_disjoint_states_not_subsets(self):
        s1 = types.State()
        s1.add("a", 1.0)
        s2 = types.State()
        s2.add("b", 1.0)
        assert not s1.is_subset_of(s2)
        assert not s2.is_subset_of(s1)

    def test_same_key_different_value_not_subset(self):
        s1 = types.State()
        s1.add("x", 1.0)
        s2 = types.State()
        s2.add("x", 9.0)
        assert not s1.is_subset_of(s2)


class TestStateProtoRoundtrip:
    def test_empty_state_roundtrip(self):
        s = types.State()
        r = types.State.from_proto_bytes(s.to_proto_bytes())
        assert r.keys() == []

    def test_scalar_values_preserved(self):
        s = types.State()
        s.add("d", 2.718)
        s.add("s", "hello")
        s.add("b", True)
        r = types.State.from_proto_bytes(s.to_proto_bytes())
        assert r["d"].as_double == pytest.approx(2.718)
        assert r["s"].as_string == "hello"
        assert r["b"].as_bool is True

    def test_list_value_preserved(self):
        s = types.State()
        s.add("lst", [1.0, 2.0])
        r = types.State.from_proto_bytes(s.to_proto_bytes())
        items = r["lst"].as_list
        assert len(items) == 2
        assert items[0].as_double == pytest.approx(1.0)
        assert items[1].as_double == pytest.approx(2.0)

    def test_nested_state_preserved(self):
        inner = types.State()
        inner.add("inner_key", "inner_val")
        outer = types.State()
        outer.add("nested", inner)
        r = types.State.from_proto_bytes(outer.to_proto_bytes())
        assert r["nested"].is_state
        assert r["nested"].as_state["inner_key"].as_string == "inner_val"


class TestStateFileIO:
    def test_roundtrip_via_file(self, tmp_path):
        s = types.State()
        s.add("x", 42.0)
        s.add("label", "test")
        path = str(tmp_path / "state.pb")
        s.save_to_file(path)
        r = types.State.load_from_file(path)
        assert r["x"].as_double == pytest.approx(42.0)
        assert r["label"].as_string == "test"


# ──────────────────────────────────────────────────────────────────────────────
# __str__ / __repr__ for Value and State
# ──────────────────────────────────────────────────────────────────────────────


class TestValueStringMethods:
    def test_str_returns_string(self):
        v = types.Value(3.14)
        assert isinstance(str(v), str)
        assert len(str(v)) > 0

    def test_repr_contains_class_name(self):
        v = types.Value("hello")
        assert repr(v).startswith("Value(")

    def test_repr_ends_with_paren(self):
        v = types.Value(True)
        assert repr(v).endswith(")")


class TestStateStringMethods:
    def test_str_returns_string(self):
        s = types.State()
        s.add("x", 1.0)
        assert isinstance(str(s), str)
        assert len(str(s)) > 0

    def test_repr_contains_class_name(self):
        s = types.State()
        s.add("x", 1.0)
        assert repr(s).startswith("State(")

    def test_repr_ends_with_paren(self):
        s = types.State()
        assert repr(s).endswith(")")
