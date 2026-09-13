import json
import math
from pathlib import Path
import pytest
from competition_mapping_budget import area_gate,budget
from sanitation_formal_campus_integration.scan_self_filter_core import filter_ranges

def test_95_percent_is_not_20000():
    assert not area_gate(19000);assert not area_gate(19999.999);assert area_gate(20000)
    assert not area_gate(float('nan'))

def test_physical_hits_and_nan_not_filled_by_29m_candidate():
    filtered,_,_=filter_ranges(angle_min=0.,angle_increment=.1,ranges=[math.inf,math.nan,5.,29.5],masks=[],range_max=30.,normalize_positive_infinity=True,no_return_replacement_m=29.)
    assert filtered[0]==29.;assert math.isnan(filtered[1]);assert filtered[2:]==[5.,29.5]
    # At exactly the raster cutoff, Karto's strict endpoint test is false.
    assert not filtered[0]<29.-1e-6

def test_measured_rtf_budget_and_no_implicit_map_pass():
    c=json.loads((Path(__file__).resolve().parents[1]/'config/competition_mapping_29m_candidate.json').read_text())
    r=budget(c,.084,120);assert r['expected_wall_minutes'][1]>400
    assert not r['budget_sufficient_for_upper_estimate'];assert r['map_gate']=='NOT_MEASURED'
    with pytest.raises(ValueError):budget(c,0,120)
