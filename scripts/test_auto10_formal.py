from auto10_formal import percentile, rejected_cases, valid_cases


def test_formal_matrix_size_and_categories():
    cases = valid_cases() + rejected_cases()
    assert len(cases) >= 1000
    assert {"normal", "synonym", "missing", "conflict", "unsafe", "bilingual", "asr_noisy"} <= {
        item["category"] for item in cases
    }


def test_percentile_is_bounded():
    assert percentile([1, 2, 3, 4], 0.95) == 3
