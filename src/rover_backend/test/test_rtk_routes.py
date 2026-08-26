"""HTTP contract tests for authenticated RTK routes."""

from __future__ import annotations

from pathlib import Path

import pytest

from fastapi import FastAPI
from fastapi.testclient import TestClient

from rover_backend.auth import (
    AuthenticatedSession,
    require_auth,
)
from rover_backend.rtk_control_service import (
    RtkControlService,
)
from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_profile_store import (
    RtkProfileStore,
)
from rover_backend.rtk_routes import (
    RtkProfileCreateRequest,
    clear_rtk_control_service,
    get_rtk_control_service,
    install_rtk_control_service,
    rtk_router,
)
from rover_backend.rtk_runtime_service import (
    RtkRuntimeServiceSnapshot,
)


SECRET = "ROUTE_SECRET_3819"


class FakeRuntime:
    def __init__(self):
        self.start_calls = 0
        self.stop_calls = 0

        self.fail_start = False
        self.fail_stop = False

        self._snapshot = (
            RtkRuntimeServiceSnapshot(
                running=True,
                shutdown_requested=False,
                owner_thread_id=123,
                mavros_ready=False,
                last_error_code=None,
                runtime=None,
            )
        )

    @property
    def snapshot(self):
        return self._snapshot

    def request_start(self):
        self.start_calls += 1

        if self.fail_start:
            raise RuntimeError(
                "synthetic start failure"
            )

    def request_stop(self):
        self.stop_calls += 1

        if self.fail_stop:
            raise RuntimeError(
                "synthetic stop failure"
            )


def authenticated_session(
) -> AuthenticatedSession:
    return AuthenticatedSession(
        session_id="test-session",
        username="operator",
        created_at=(
            "2026-08-26T00:00:00Z"
        ),
        expires_at=(
            "2026-08-27T00:00:00Z"
        ),
        client_ip="127.0.0.1",
        user_agent="pytest",
    )


@pytest.fixture
def api(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "rtk.sqlite3"
    )

    store.initialize()

    runtime = FakeRuntime()

    control = RtkControlService(
        store,
        runtime,
    )

    app = FastAPI()

    app.include_router(
        rtk_router
    )

    app.dependency_overrides[
        require_auth
    ] = authenticated_session

    app.dependency_overrides[
        get_rtk_control_service
    ] = lambda: control

    with TestClient(app) as client:
        yield (
            client,
            control,
            store,
            runtime,
        )


def profile_body(
    *,
    name: str = "Office Base",
    password: str = SECRET,
) -> dict:
    return {
        "name": name,
        "caster_host": "caster.test",
        "caster_port": 2101,
        "mountpoint": "MOUNT",
        "username": "rover",
        "password": password,
    }


def create_profile(
    client: TestClient,
    *,
    name="Office Base",
    password=SECRET,
) -> int:
    response = client.post(
        "/api/rtk/profiles",
        json=profile_body(
            name=name,
            password=password,
        ),
    )

    assert response.status_code == 201

    return int(
        response.json()[
            "profile"
        ][
            "id"
        ]
    )


def activate_profile(
    client: TestClient,
    profile_id: int,
) -> None:
    response = client.post(
        (
            "/api/rtk/profiles/"
            f"{profile_id}/activate"
        )
    )

    assert response.status_code == 200


def test_password_request_repr_is_redacted():
    body = RtkProfileCreateRequest(
        **profile_body()
    )

    assert SECRET not in repr(body)


def test_routes_require_auth(
    tmp_path: Path,
):
    store = RtkProfileStore(
        tmp_path / "auth.sqlite3"
    )
    store.initialize()

    runtime = FakeRuntime()

    control = RtkControlService(
        store,
        runtime,
    )

    app = FastAPI()
    app.include_router(
        rtk_router
    )

    app.dependency_overrides[
        get_rtk_control_service
    ] = lambda: control

    with TestClient(app) as client:
        response = client.get(
            "/api/rtk/status"
        )

    assert response.status_code == 401


def test_missing_control_service_returns_503():
    clear_rtk_control_service()

    app = FastAPI()
    app.include_router(
        rtk_router
    )

    app.dependency_overrides[
        require_auth
    ] = authenticated_session

    with TestClient(app) as client:
        response = client.get(
            "/api/rtk/status"
        )

    assert response.status_code == 503

    assert (
        response.json()["detail"]["code"]
        == "RTK_CONTROL_UNAVAILABLE"
    )


def test_create_profile_is_secret_free(
    api,
):
    client, _, _, _ = api

    response = client.post(
        "/api/rtk/profiles",
        json=profile_body(),
    )

    assert response.status_code == 201

    payload = response.json()

    assert (
        payload["profile"][
            "password_configured"
        ]
        is True
    )

    assert SECRET not in response.text

    assert "password" not in (
        payload["profile"]
    )


def test_list_and_get_profiles_are_secret_free(
    api,
):
    client, _, _, _ = api

    profile_id = create_profile(
        client
    )

    listing = client.get(
        "/api/rtk/profiles"
    )

    assert listing.status_code == 200
    assert listing.json()["count"] == 1
    assert SECRET not in listing.text

    detail = client.get(
        f"/api/rtk/profiles/{profile_id}"
    )

    assert detail.status_code == 200
    assert SECRET not in detail.text


def test_duplicate_profile_name_returns_409(
    api,
):
    client, _, _, _ = api

    create_profile(
        client,
        name="Office Base",
    )

    response = client.post(
        "/api/rtk/profiles",
        json=profile_body(
            name="office base"
        ),
    )

    assert response.status_code == 409

    assert (
        response.json()["detail"]["code"]
        == "RTK_PROFILE_CONFLICT"
    )


def test_invalid_profile_returns_422(
    api,
):
    client, _, _, _ = api

    body = profile_body()
    body["caster_port"] = 0

    response = client.post(
        "/api/rtk/profiles",
        json=body,
    )

    assert response.status_code == 422

    assert (
        response.json()["detail"]["code"]
        == "RTK_PROFILE_INVALID"
    )


def test_extra_create_field_is_rejected(
    api,
):
    client, _, _, _ = api

    body = profile_body()
    body["unexpected_secret"] = (
        "do-not-accept"
    )

    response = client.post(
        "/api/rtk/profiles",
        json=body,
    )

    assert response.status_code == 422


def test_activate_then_start_persists_and_forwards(
    api,
):
    client, _, store, runtime = api

    profile_id = create_profile(
        client
    )

    activate_profile(
        client,
        profile_id,
    )

    response = client.post(
        "/api/rtk/start"
    )

    assert response.status_code == 200

    assert (
        response.json()[
            "persisted"
        ][
            "desired_state"
        ]
        == "RUNNING"
    )

    assert (
        store.runtime_state()
        .desired_state
        is DesiredState.RUNNING
    )

    assert runtime.start_calls == 1


def test_start_without_active_profile_returns_409(
    api,
):
    client, _, _, runtime = api

    response = client.post(
        "/api/rtk/start"
    )

    assert response.status_code == 409

    assert runtime.start_calls == 0


def test_stop_failure_returns_503_but_stays_persisted_stopped(
    api,
):
    client, _, store, runtime = api

    profile_id = create_profile(
        client
    )
    activate_profile(
        client,
        profile_id,
    )

    assert (
        client.post(
            "/api/rtk/start"
        ).status_code
        == 200
    )

    runtime.fail_stop = True

    response = client.post(
        "/api/rtk/stop"
    )

    assert response.status_code == 503

    assert (
        response.json()["detail"]["code"]
        == "RTK_RUNTIME_UNAVAILABLE"
    )

    assert (
        store.runtime_state()
        .desired_state
        is DesiredState.STOPPED
    )


def test_patch_runtime_field_forces_stop(
    api,
):
    client, _, store, runtime = api

    profile_id = create_profile(
        client
    )
    activate_profile(
        client,
        profile_id,
    )

    client.post(
        "/api/rtk/start"
    )

    response = client.patch(
        f"/api/rtk/profiles/{profile_id}",
        json={
            "caster_host": (
                "new-caster.test"
            )
        },
    )

    assert response.status_code == 200

    assert (
        response.json()["profile"][
            "caster_host"
        ]
        == "new-caster.test"
    )

    assert (
        store.runtime_state()
        .desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_patch_password_never_returns_secret(
    api,
):
    client, _, store, _ = api

    profile_id = create_profile(
        client
    )

    activate_profile(
        client,
        profile_id,
    )

    new_secret = (
        "NEW_ROUTE_SECRET_9982"
    )

    response = client.patch(
        f"/api/rtk/profiles/{profile_id}",
        json={
            "password": new_secret
        },
    )

    assert response.status_code == 200

    assert new_secret not in response.text
    assert SECRET not in response.text

    config = (
        store.build_active_worker_config(
            "run-secret-check"
        )
    )

    assert (
        config.password
        == new_secret
    )


def test_delete_missing_profile_returns_404(
    api,
):
    client, _, _, _ = api

    response = client.delete(
        "/api/rtk/profiles/999"
    )

    assert response.status_code == 404


def test_delete_active_profile_while_running_forwards_stop(
    api,
):
    client, _, store, runtime = api

    profile_id = create_profile(
        client
    )
    activate_profile(
        client,
        profile_id,
    )

    client.post(
        "/api/rtk/start"
    )

    response = client.delete(
        f"/api/rtk/profiles/{profile_id}"
    )

    assert response.status_code == 200

    state = store.runtime_state()

    assert state.active_profile_id is None

    assert (
        state.desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_clear_active_profile_forwards_stop(
    api,
):
    client, _, store, runtime = api

    profile_id = create_profile(
        client
    )
    activate_profile(
        client,
        profile_id,
    )

    client.post(
        "/api/rtk/start"
    )

    response = client.delete(
        "/api/rtk/active-profile"
    )

    assert response.status_code == 200

    assert (
        response.json()[
            "persisted"
        ][
            "active_profile_id"
        ]
        is None
    )

    assert (
        store.runtime_state()
        .desired_state
        is DesiredState.STOPPED
    )

    assert runtime.stop_calls == 1


def test_status_payload_is_credential_free(
    api,
):
    client, _, _, _ = api

    profile_id = create_profile(
        client
    )
    activate_profile(
        client,
        profile_id,
    )

    client.post(
        "/api/rtk/start"
    )

    response = client.get(
        "/api/rtk/status"
    )

    assert response.status_code == 200

    payload = response.json()[
        "status"
    ]

    assert (
        payload["persisted"][
            "desired_state"
        ]
        == "RUNNING"
    )

    assert (
        payload["active_profile"][
            "password_configured"
        ]
        is True
    )

    assert (
        payload["runtime"][
            "supervisor"
        ][
            "running"
        ]
        is True
    )

    assert SECRET not in response.text
    assert "password_secret" not in response.text


def test_registry_install_is_idempotent_and_clear(
    tmp_path: Path,
):
    clear_rtk_control_service()

    store = RtkProfileStore(
        tmp_path / "registry.sqlite3"
    )
    store.initialize()

    service = RtkControlService(
        store,
        FakeRuntime(),
    )

    try:
        install_rtk_control_service(
            service
        )

        install_rtk_control_service(
            service
        )

        assert (
            get_rtk_control_service()
            is service
        )

    finally:
        clear_rtk_control_service(
            service
        )

    app = FastAPI()
    app.include_router(
        rtk_router
    )

    app.dependency_overrides[
        require_auth
    ] = authenticated_session

    with TestClient(app) as client:
        response = client.get(
            "/api/rtk/status"
        )

    assert response.status_code == 503
