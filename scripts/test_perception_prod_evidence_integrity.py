from perception_prod_verify_evidence import parse_index


def test_parse_final_evidence_index_row():
    rows = parse_index(
        "| x3 | `" + "a" * 64 + "` | 28542 | `x3/x3_full_static_report.json` |\n"
    )
    assert rows == [
        {
            "path": "x3/x3_full_static_report.json",
            "sha256": "a" * 64,
            "bytes": 28542,
        }
    ]
