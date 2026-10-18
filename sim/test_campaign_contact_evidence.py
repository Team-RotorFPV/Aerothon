"""A native fixture contact may not be hidden by the layout-distance proxy."""
import pytest
from campaign_record import navigation_grade


def grade(contacts):
    layout=dict(home_world=[0.,0.],gate=[2.,2.,0.],
        corridors=dict(outbound=[2.,2.,0.,10.,3.5,3.6],
                       **{'return':[12.,-2.,3.14159,10.,3.5,3.6]}),
        geofence_rect=[-10.,60.,-40.,40.],delivery_zone_rect=[33.,0.,40.,30.],
        pads={'a':[21.,10.]},red_zones={},
        obstacles=[dict(h=4.,poly=[[20.,20.],[21.,20.],[21.,21.],[20.,21.]])])
    base=dict(x=2.,y=2.,z=3.2,armed=True,state='GATE_CROSSING',roll=0.,pitch=0.,mode='GUIDED')
    track=[dict(base,t=0.),dict(base,t=1.,x=0.,y=0.,z=.05,armed=False,state='COMPLETED')]
    return navigation_grade(track,layout,'a',independent_contacts=contacts)


def test_board_contact_overrides_clear_baffle_geometry():
    result=grade([[.25,'board',2.,2.,3.2]])
    assert result['checks']['no obstacle contact'] is False
    assert result['pass_navigation'] is False


def test_explicit_empty_contact_record_preserves_geometry_verdict():
    assert grade([])['checks']['no obstacle contact'] is True


@pytest.mark.parametrize('contacts',[None,{},[['bad','board',2.,2.,3.2]],[[.25,'board',2.,float('nan'),3.2]]])
def test_absent_or_malformed_contact_record_cannot_qualify(contacts):
    assert grade(contacts)['checks']['no obstacle contact'] is False
