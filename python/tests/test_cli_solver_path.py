import io
import contextlib


def make_dummy_path(n=5):
    pts = []
    for i in range(n):
        pts.append({
            'lat': 0.0,
            'lon': 0.0,
            'x_m': float(i * 10),
            'y_m': 0.0,
            'z_m': 0.0,
            'curvature': 0.0 if i in (0, n-1) else 0.01,
            'distance_along_m': float(i * 10),
            'surface_type': 'asphalt',
        })
    return pts


def test_core_path_used(monkeypatch):
    import apexvelocity.cli as cli

    # Stub core API solve to simulate native path
    class DummyResult:
        def __init__(self, N):
            self.velocity_profile_mps = [10.0 + i for i in range(N)]
            self.energy_joules = [i * 100.0 for i in range(N)]

    def stub_solve(path, surfaces, vehicle, condition, initial_speed=0.0, final_speed=0.0):
        return DummyResult(len(path))

    import apexvelocity.api as api
    monkeypatch.setattr(api, 'solve', stub_solve)

    pts = make_dummy_path(6)
    stderr = io.StringIO()
    with contextlib.redirect_stderr(stderr):
        cli._solve_with_core_or_fallback(pts, vehicle_name='tesla_model_3', condition='dry')

    # No warning expected
    assert '[WARN]' not in stderr.getvalue()
    # Speeds set from core stub
    assert pts[0]['speed_mps'] == 10.0
    assert pts[-1]['speed_mps'] == 15.0
    # Energy mapped
    assert pts[-1]['energy_kwh'] == pts[-1]['energy_joules'] / 3.6e6


def test_fallback_when_core_missing(monkeypatch):
    import apexvelocity.cli as cli

    # Make api.solve raise to trigger fallback
    import apexvelocity.api as api
    def boom(*args, **kwargs):
        raise ImportError('no native')
    monkeypatch.setattr(api, 'solve', boom, raising=True)

    pts = make_dummy_path(5)
    stderr = io.StringIO()
    with contextlib.redirect_stderr(stderr):
        cli._solve_with_core_or_fallback(pts, vehicle_name='default', condition='dry', vehicle_params={'mass_kg':1500,'drag_coeff':0.3,'frontal_area':2.2,'rolling_res':0.015})

    # Warning printed
    assert '[WARN]' in stderr.getvalue()
    # Fallback assigns speeds and energy cumulatively
    assert all('speed_mps' in p for p in pts)
    assert all('energy_kwh' in p for p in pts)

