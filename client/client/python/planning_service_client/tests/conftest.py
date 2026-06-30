import pytest
from planning_service_client.api.base_client import ClientOptions


@pytest.fixture
def test_options():
    options = ClientOptions()
    options.name = "TestClient"
    options.monitor_health = False
    return options
