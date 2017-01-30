from robot import rescale_js

def test_scaling_deadzone():
    # Deadzone
    assert rescale_js(0.0, deadzone=0.1) == 0.0
    assert rescale_js(0.1, deadzone=0.1) == 0.0
    assert rescale_js(-0.1, deadzone=0.1) == 0.0
    assert rescale_js(1.0, deadzone=0.1) == 1.0
    assert rescale_js(-1.0, deadzone=0.1) == -1.0
    # Remaining range should be scaled evenly to avoid jumps around the deadzone value
    assert abs(rescale_js(0.6, deadzone=0.2) - 0.5) < 0.05
    assert abs(rescale_js(-0.6, deadzone=0.2) - -0.5) < 0.05

def test_scaling_expo():
    # Exponential
    assert rescale_js(0.0, exponential=0.5) == 0.0
    assert rescale_js(1.0, exponential=0.5) == 1.0
    assert rescale_js(-1.0, exponential=0.5) == -1.0
    # Check for strictly increasing function
    assert rescale_js(0.5, exponential=0.5) < rescale_js(0.6, exponential=0.5)
    assert rescale_js(-0.5, exponential=0.5) > rescale_js(-0.6, exponential=0.5)
    # Check that more expo damps the centre more
    assert rescale_js(0.5, exponential=0.5) > rescale_js(0.5, exponential=0.6)
    assert rescale_js(-0.5, exponential=0.5) < rescale_js(-0.5, exponential=0.6)
    # Check symmetrical
    assert rescale_js(-0.5, exponential=0.5) == -rescale_js(0.5, exponential=0.5)

def test_scaling_rates():
    # Check rates
    assert rescale_js(0.0, rate=0.5) == 0.0
    assert rescale_js(1.0, rate=0.5) == 0.5
    assert rescale_js(-1.0, rate=0.5) == -0.5
    assert rescale_js(0.5, rate=0.5) == 0.25
    assert rescale_js(-0.5, rate=0.5) == -0.25

def test_scaling_combo():
    # Check combination
    assert rescale_js(0.0, deadzone=0.2, exponential=0.2, rate=0.9) == 0.0
    assert (rescale_js(1.0, deadzone=0.2, exponential=0.2, rate=0.9) - 0.9) < 0.05
    assert (rescale_js(-1.0, deadzone=0.2, exponential=0.2, rate=0.9) - -0.9) < 0.05
    # Test halfway point
    assert rescale_js(0.6, deadzone=0.2, exponential=0.2, rate=0.9) < 0.45
    assert rescale_js(-0.6, deadzone=0.2, exponential=0.2, rate=0.9) > -0.45
    assert rescale_js(0.6, deadzone=0.2, exponential=0.2, rate=0.9) == -rescale_js(-0.6, deadzone=0.2, exponential=0.2, rate=0.9)
