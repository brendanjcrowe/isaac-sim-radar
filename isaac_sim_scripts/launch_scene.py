"""Load an urban scene with robot, radar, and LiDAR in Isaac Sim.

Run inside Isaac Sim's Script Editor or via:
    ./isaac-sim.sh --exec launch_scene.py

This script:
1. Loads a base USD stage (or creates a ground plane)
2. Creates a procedural urban environment (buildings, walls, pillars)
3. Spawns the robot (iw.hub or placeholder)
4. Attaches RTX Radar sensor with WpmDmatApproxRadar config
5. Attaches RTX Lidar sensor
6. Enables ROS2 bridge extension
7. Sets up OmniRadar replicator annotator for UDP output
8. Sets up LiDAR → ROS2 OmniGraph publishing
"""

import os
import yaml
import carb
import omni.usd
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

# Paths (adjust for your Isaac Sim installation)
ISAAC_NUCLEUS = "omniverse://localhost/NVIDIA/Assets/Isaac/5.1"
ROBOT_USD = f"{ISAAC_NUCLEUS}/Isaac/Robots/Idealworks/iw_hub_sensors.usd"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
CONFIG_DIR = os.path.join(PROJECT_DIR, "config")


def load_config(name: str) -> dict:
    """Load a YAML config file from the config directory."""
    path = os.path.join(CONFIG_DIR, name)
    if os.path.exists(path):
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    carb.log_warn(f"Config not found: {path}")
    return {}


def _make_material(stage, prim_path: str, label: str, metallic: float) -> Sdf.Path:
    """Create a simple OmniPBR material with a metallic value for radar reflectivity.

    WpmDmatApproxRadar reads material metallic/roughness from the bound shader.
    Concrete:  metallic≈0.0, roughness≈0.8  → low radar reflectivity
    Metal:     metallic≈1.0, roughness≈0.2  → high radar reflectivity

    For M6 (material tuning), replace with full MDL asset references from Nucleus.
    """
    mat_path = Sdf.Path(f"/World/Materials/{label}")
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path.AppendChild("Shader"))
    shader.CreateIdAttr("OmniPBR")
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0 - metallic * 0.6)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return mat_path


def _bind_material(stage, prim_path: str, mat_path: Sdf.Path):
    """Apply a material binding to the given prim."""
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
        mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
        binding_api.Bind(mat)


def _add_collision(stage, prim_path: str):
    """Apply UsdPhysics collision to the prim at prim_path."""
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        UsdPhysics.CollisionAPI.Apply(prim)


def _set_xform(stage, prim_path: str, translate=None, scale=None):
    """Set translate and/or scale on a prim via UsdGeom.Xformable.

    In Isaac Sim 5.1, xformOp attributes cannot be set through the
    CreatePrimCommand ``attributes`` dict — they must be applied via the
    Xformable API after the prim has been created.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    if translate is not None:
        xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if scale is not None:
        # PrecisionDouble matches the 'double3' attribute left by CreatePrimCommand
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))


def create_urban_environment(stage):
    """Create a procedural urban block for radar detection testing.

    All geometry is placed within the WpmDmatApproxRadar FOV (±75° azimuth, 100m max range).
    Robot is at the origin facing +X.

    Layout (approximate, all positions in meters):
      Buildings (concrete, high radar cross-section at sharp corners):
        A: x=20, y=10   — 12m wide, 8m deep, 15m tall
        B: x=30, y=-8   — 15m wide, 10m deep, 10m tall
        C: x=42, y=6    — 10m wide, 10m deep, 20m tall
        D: x=45, y=-16  — 20m wide, 12m deep, 8m tall
      Walls/barriers (concrete):
        W1: x=10, y=0   — 10m long, 0.3m thick, 3m tall (road divider)
        W2: x=12, y=-12 — 8m long,  0.3m thick, 2m tall (fence)
      Pillars (metal, strong radar return):
        P1: x=8,  y=-3
        P2: x=9,  y=5
        P3: x=7,  y=-6
    """
    # Create materials once
    concrete_mat = _make_material(stage, "/World/Materials/Concrete", "Concrete", metallic=0.05)
    metal_mat    = _make_material(stage, "/World/Materials/Metal",    "Metal",    metallic=0.95)

    # ── Buildings ─────────────────────────────────────────────────────────────
    buildings = [
        ("/World/Urban/Building_A", (20.0,  10.0, 7.5),  (12.0, 8.0,  15.0)),
        ("/World/Urban/Building_B", (30.0,  -8.0, 5.0),  (15.0, 10.0, 10.0)),
        ("/World/Urban/Building_C", (42.0,   6.0, 10.0), (10.0, 10.0, 20.0)),
        ("/World/Urban/Building_D", (45.0, -16.0, 4.0),  (20.0, 12.0, 8.0)),
    ]
    for path, (tx, ty, tz), (sx, sy, sz) in buildings:
        omni.kit.commands.execute("CreatePrimCommand", prim_type="Cube", prim_path=path)
        _set_xform(stage, path, translate=(tx, ty, tz), scale=(sx / 2, sy / 2, sz / 2))
        _add_collision(stage, path)
        _bind_material(stage, path, concrete_mat)

    # ── Walls / barriers ──────────────────────────────────────────────────────
    walls = [
        ("/World/Urban/Wall_1", (10.0,  0.0, 1.5), (10.0, 0.3, 3.0)),
        ("/World/Urban/Wall_2", (12.0, -12.0, 1.0), (8.0,  0.3, 2.0)),
    ]
    for path, (tx, ty, tz), (sx, sy, sz) in walls:
        omni.kit.commands.execute("CreatePrimCommand", prim_type="Cube", prim_path=path)
        _set_xform(stage, path, translate=(tx, ty, tz), scale=(sx / 2, sy / 2, sz / 2))
        _add_collision(stage, path)
        _bind_material(stage, path, concrete_mat)

    # ── Metal pillars (strong radar return) ───────────────────────────────────
    pillars = [
        ("/World/Urban/Pillar_1", ( 8.0, -3.0, 2.0)),
        ("/World/Urban/Pillar_2", ( 9.0,  5.0, 2.0)),
        ("/World/Urban/Pillar_3", ( 7.0, -6.0, 2.0)),
    ]
    for path, (tx, ty, tz) in pillars:
        omni.kit.commands.execute("CreatePrimCommand", prim_type="Cylinder", prim_path=path)
        _set_xform(stage, path, translate=(tx, ty, tz), scale=(0.3, 0.3, 2.0))
        _add_collision(stage, path)
        _bind_material(stage, path, metal_mat)

    carb.log_info("Urban environment created: 4 buildings, 2 walls, 3 pillars.")


def create_ground_plane(stage):
    """Create a simple ground plane for testing."""
    omni.kit.commands.execute("CreatePrimCommand", prim_type="Plane", prim_path="/World/GroundPlane")
    _set_xform(stage, "/World/GroundPlane", scale=(100.0, 100.0, 1.0))
    # Add collision
    prim = stage.GetPrimAtPath("/World/GroundPlane")
    if prim.IsValid():
        UsdPhysics.CollisionAPI.Apply(prim)


def spawn_robot(stage, robot_usd: str = ROBOT_USD):
    """Reference the robot USD into the stage, falling back to a placeholder cube.

    CreateReferenceCommand does not raise on unreachable Nucleus URLs — it silently
    creates a prim with a dangling reference, which blocks sensor attachment later.
    We pre-check with omni.client.stat() so the fallback triggers immediately.
    """
    robot_path = "/World/Robot"

    # Check Nucleus reachability without a blocking network call.
    # omni.client.stat() times out after ~2 minutes when the server is unreachable.
    # Only probe if ISAAC_NUCLEUS_URI env var is set (operator explicitly enabled Nucleus).
    # Use an alias so that 'omni' stays bound to the module-level global throughout
    # this function (a bare 'import omni.client' would shadow it as an unbound local).
    nucleus_ok = False
    if os.environ.get("ISAAC_NUCLEUS_URI"):
        try:
            import omni.client as _oc
            result, _ = _oc.stat(robot_usd)
            nucleus_ok = (result == _oc.Result.OK)
        except Exception:
            pass

    if nucleus_ok:
        try:
            omni.kit.commands.execute(
                "CreateReferenceCommand",
                usd_context=omni.usd.get_context(),
                path_to=robot_path,
                asset_path=robot_usd,
            )
            carb.log_info(f"Loaded robot from {robot_usd}")
            return robot_path
        except Exception as e:
            carb.log_warn(f"CreateReferenceCommand failed ({e}). Using placeholder.")

    carb.log_warn("Nucleus not available — using placeholder Xform for robot.")
    # Use Xform (not Cube) so that OmniRadar/OmniLidar sensor children can be
    # nested under it.  USD forbids nesting Gprims inside other Gprims, so a
    # Cube parent would block OmniRadar creation.
    omni.kit.commands.execute("CreatePrimCommand", prim_type="Xform", prim_path=robot_path)
    _set_xform(stage, robot_path, translate=(0.0, 0.0, 0.2))
    return robot_path


def attach_radar_sensor(stage, parent_path: str):
    """Create an RTX Radar sensor (OmniRadar prim) attached to the robot.

    Uses IsaacSensorCreateRtxRadar which applies all required schemas:
      OmniSensorGenericRadarWpmDmatAPI, OmniSensorAPI, OmniSensorEncryptionAPI,
      OmniSensorGenericRadarWpmDmatScanCfgAPI:s001

    IMPORTANT: IsaacSensorCreateRtxRadar strips the leading "/" from the path
    and renames any path containing "/" separators (e.g. World/Robot/RadarSensor
    becomes World_Robot_RadarSensor via Tf.MakeValidIdentifier).  To avoid this,
    use a top-level path "/Radar" with parent=None and position the sensor at
    the absolute world coordinates (robot position + mount offset).
    """
    radar_config = load_config("radar_params.yaml")
    radar_path = "/Radar"

    # Compute absolute world position: parent prim translation + mount offset
    mount = radar_config.get("mount_offset", {})
    parent_prim = stage.GetPrimAtPath(parent_path)
    parent_translate = Gf.Vec3d(0.0, 0.0, 0.0)
    if parent_prim.IsValid():
        xf = UsdGeom.Xformable(parent_prim)
        ops = xf.GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                parent_translate = Gf.Vec3d(v[0], v[1], v[2])
                break
    translation = Gf.Vec3d(
        parent_translate[0] + mount.get("x", 0.3),
        parent_translate[1] + mount.get("y", 0.0),
        parent_translate[2] + mount.get("z", 0.4),
    )

    # Build scan config kwargs from radar_params.yaml.
    # These apply OmniSensorGenericRadarWpmDmatScanCfgAPI:s001 — without them
    # the sensor has no scan profile and produces empty annotator data.
    _scan = radar_config.get("scan", {})
    _scan_defaults = {
        "rBins": 64, "vBins": 32, "azBins": 64, "elBins": 4,
        "maxRangeM": 100.0, "raysPerDeg": 2.0,
        "maxAzAngDeg": 75.0, "maxElAngDeg": 10.0,
    }
    scan_kwargs = {
        f"omni:sensor:WpmDmat:scan:s001:{k}": _scan.get(k, v)
        for k, v in _scan_defaults.items()
    }

    try:
        success, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            path=radar_path,
            parent=None,
            translation=translation,
            orientation=Gf.Quatd(1, 0, 0, 0),
            **scan_kwargs,
        )
    except Exception as e:
        carb.log_error(f"Failed to create RTX Radar sensor: {e}")
        return None

    if not success or sensor is None:
        carb.log_error(
            f"IsaacSensorCreateRtxRadar returned success={success}, sensor={sensor!r}"
        )
        return None

    actual_path = str(sensor.GetPath())
    carb.log_info(
        f"RTX Radar sensor (WpmDMAT) created at {actual_path} "
        f"(position={translation})"
    )
    return actual_path


def attach_lidar_sensor(stage, parent_path: str):
    """Create an RTX Lidar sensor attached to the robot.

    Returns the path to the actual OmniLidar sensor prim (not the parent Xform).
    IsaacSensorCreateRtxLidar creates:
      <parent_path>/LidarSensor        — Xform (for mount offset)
      <parent_path>/LidarSensor/sensor — OmniLidar prim (actual sensor)

    The OmniLidar path is what rep.create.render_product() and ROS2RtxLidarHelper need.
    """
    lidar_config = load_config("lidar_params.yaml")
    lidar_xform_path = f"{parent_path}/LidarSensor"

    # config="OS1" matches the USD at /Isaac/Sensors/Ouster/OS1/OS1.usd on NVIDIA's CDN.
    # variant selects the specific emitter layout within that USD.
    # Isaac Sim resolves get_assets_root_path() to NVIDIA's public CDN — no local Nucleus needed.
    _, sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_xform_path,
        config=lidar_config.get("config", "OS1"),
        variant=lidar_config.get("variant", "OS1_REV7_128ch10hz1024res"),
    )

    xform_prim = stage.GetPrimAtPath(lidar_xform_path)
    if not xform_prim.IsValid():
        carb.log_error("Failed to create RTX Lidar sensor")
        return None

    # Apply mount offset to the Xform parent
    mount = lidar_config.get("mount_offset", {})
    xform = UsdGeom.Xformable(xform_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(
        mount.get("x", 0.0),
        mount.get("y", 0.0),
        mount.get("z", 0.5),
    ))

    # Return the OmniLidar sensor prim path (required by rep.create.render_product)
    if sensor_prim is not None and sensor_prim.IsValid():
        sensor_path = str(sensor_prim.GetPath())
    else:
        # Fallback: sensor child is always named 'sensor'
        sensor_path = lidar_xform_path + "/sensor"
        if not stage.GetPrimAtPath(sensor_path).IsValid():
            carb.log_warn(
                f"OmniLidar prim not found at {sensor_path}; "
                "setup_lidar_ros2_publisher may fail"
            )
            sensor_path = lidar_xform_path

    carb.log_info(f"RTX Lidar sensor created at {sensor_path}")
    return sensor_path


def setup_radar_annotator(radar_path: str):
    """Attach a replicator GenericModelOutput annotator to the OmniRadar prim.

    The correct pipeline for Isaac Sim 5.1 RTX radar data:
      1. Look up the prim and use prim.GetPath() (SdfPath, not string) when
         creating the render product — passing a string path causes Replicator
         to strip the leading "/" and replace "/" with "_", breaking lookup.
      2. render_vars=["GenericModelOutput"] is the correct render var for RTX
         sensor data; "RtxSensorCpu" is a generic GPU readback var that returns
         empty data when attached to an OmniRadar render product.
      3. annotator.attach([render_product.path]) uses the render product's own
         path string (not the prim path string, and not the render product object).

    Call annotator.get_data() each frame to get the raw GMO numpy buffer.
    Use isaacsim.sensors.rtx.get_gmo_data(buf) to parse the structured output.

    Returns:
        annotator handle, or None on failure.
    """
    try:
        import omni.replicator.core as rep

        stage = omni.usd.get_context().get_stage()
        radar_prim = stage.GetPrimAtPath(radar_path)
        if not radar_prim.IsValid():
            carb.log_error(f"setup_radar_annotator: prim not valid at {radar_path}")
            return None

        # Use SdfPath (not string) to avoid Replicator path-stripping bug.
        # RtxSensorMetadata must be included alongside GenericModelOutput for
        # the sensor pipeline to activate (matches NVIDIA's inspect_radar_metadata.py).
        render_product = rep.create.render_product(
            radar_prim.GetPath(),
            [1, 1],
            render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        )
        annotator = rep.AnnotatorRegistry.get_annotator("GenericModelOutput")
        annotator.attach([render_product.path])
        carb.log_info(f"Radar GMO annotator attached to {radar_path} "
                      f"(render_product={render_product.path})")
        return annotator
    except Exception as e:
        carb.log_warn(
            f"Could not set up radar annotator ({e}). "
            "Radar GMO data will not be sent via UDP."
        )
        return None


def setup_lidar_ros2_publisher(stage, lidar_sensor_path: str):
    """Attach a GenericModelOutput annotator to the OmniLidar prim's render product.

    The OmniGraph-based ROS2RtxLidarHelper approach (IsaacCreateRTXLidarScanBuffer)
    does not successfully receive data from the OS1 CDN lidar model — the scan buffer
    annotator returns no data even though the render product IS producing OMGN bytes
    (confirmed by GenericModelOutput annotator getting data on every frame).

    Instead, we use the same pipeline as radar:
      1. Create a render product on the OmniLidar prim with GenericModelOutput.
      2. Attach a GenericModelOutput annotator.
      3. In the Python update loop, call get_gmo_data() and send LDR2 UDP packets.
      4. The ros2-bridge UDP listener publishes PointCloud2 on /lidar/point_cloud.

    Returns the GenericModelOutput annotator, or None on failure.
    """
    import omni.replicator.core as rep

    sensor_prim = stage.GetPrimAtPath(lidar_sensor_path)
    if not sensor_prim.IsValid():
        carb.log_error(
            f"setup_lidar_annotator: OmniLidar prim not valid at {lidar_sensor_path}"
        )
        return None

    lidar_rp = rep.create.render_product(
        sensor_prim.GetPath(),
        resolution=(1, 1),
        render_vars=["GenericModelOutput", "RtxSensorMetadata"],
    )
    lidar_rp_path = lidar_rp.path
    carb.log_info(f"LiDAR render product created: {lidar_rp_path}")

    try:
        ann = rep.AnnotatorRegistry.get_annotator("GenericModelOutput")
        ann.attach([lidar_rp_path])
        carb.log_info(f"LiDAR GenericModelOutput annotator attached to {lidar_rp_path}")
        return ann
    except Exception as _e:
        carb.log_warn(f"LiDAR GenericModelOutput annotator failed: {_e}")
        return None


# ── Scenario Variations ────────────────────────────────────────────────────

_FOG_PRESETS = {
    "light": {"start": 20.0, "end": 80.0},
    "dense": {"start":  5.0, "end": 30.0},
}

_PEDESTRIAN_DEFS = [
    {"path": "/World/Dynamic/Ped_0",
     "start": (15.0,  4.0, 0.9), "end": (15.0, -4.0, 0.9)},
    {"path": "/World/Dynamic/Ped_1",
     "start": (25.0,  2.0, 0.9), "end": (35.0,  2.0, 0.9)},
    {"path": "/World/Dynamic/Ped_2",
     "start": ( 8.0, -2.0, 0.9), "end": ( 8.0,  2.0, 0.9)},
]

_VEHICLE_DEFS = [
    {"path": "/World/Dynamic/Vehicle_0",
     "start": (-5.0, -5.0, 0.75), "end": (60.0, -5.0, 0.75)},
    {"path": "/World/Dynamic/Vehicle_1",
     "start": (60.0,  8.0, 0.75), "end": (-5.0,  8.0, 0.75)},
]


def add_weather_effects(stage, weather_type: str = "fog", fog_preset: str = "light"):
    """Add weather effects to the scene.

    Args:
        weather_type: "fog", "rain", or "clear".
          fog  — enables RTX atmospheric fog (degrades LiDAR effective range;
                 WpmDmatApproxRadar is unaffected by fog).
          rain — scatters metallic spherical prims throughout the scene volume,
                 simulating radar clutter from precipitation.
          clear — no effect applied.
        fog_preset: "light" or "dense" (only used when weather_type="fog").
    """
    if weather_type == "fog":
        params = _FOG_PRESETS.get(fog_preset, _FOG_PRESETS["light"])
        try:
            import carb.settings
            s = carb.settings.get_settings()
            s.set("/rtx/fog/fogMode",      1)               # exponential
            s.set("/rtx/fog/fogStartDist", params["start"])
            s.set("/rtx/fog/fogEndDist",   params["end"])
            s.set("/rtx/fog/fogColor",     [0.85, 0.85, 0.90, 1.0])
            carb.log_info(
                f"Weather: {fog_preset} fog — "
                f"start={params['start']}m, end={params['end']}m"
            )
        except Exception as exc:
            carb.log_warn(f"Could not apply fog settings ({exc}); continuing.")

    elif weather_type == "rain":
        import random
        random.seed(42)
        rain_mat = _make_material(stage, "", "RainClutter", metallic=0.9)
        root = "/World/Weather/Rain"
        count = 60
        for i in range(count):
            x = random.uniform( 0.0, 55.0)
            y = random.uniform(-20.0, 20.0)
            z = random.uniform( 0.3,  4.0)
            path = f"{root}/drop_{i:03d}"
            omni.kit.commands.execute(
                "CreatePrimCommand",
                prim_type="Sphere",
                prim_path=path,
                attributes={"radius": 0.04},
            )
            _set_xform(stage, path, translate=(x, y, z))
            _bind_material(stage, path, rain_mat)
        carb.log_info(f"Weather: rain — {count} metallic clutter prims added.")

    else:
        carb.log_info("Weather: clear — no effects applied.")


def add_dynamic_objects(
    stage,
    num_pedestrians: int = 2,
    num_vehicles:    int = 1,
    num_frames:      int = 600,
):
    """Add time-sampled animated dynamic objects to the scene.

    Animation is baked into USD time samples so it plays automatically
    as the timeline advances — no changes to the update loop required.

    Pedestrians: vertical cylinders (~0.4 m dia, 1.8 m tall) that
      oscillate back and forth along a linear path within the radar FOV.
    Vehicles: car-sized boxes (4.4 m x 1.8 m x 1.5 m) that travel one-way
      across the scene at constant speed.

    Args:
        num_pedestrians: number of pedestrian prims (max 3).
        num_vehicles: number of vehicle prims (max 2).
        num_frames: total animation frames to bake (default 600 = 10 s at 60 fps).
    """
    ped_mat = _make_material(stage, "", "PedBody", metallic=0.25)
    car_mat = _make_material(stage, "", "CarBody",  metallic=0.85)

    # ── Pedestrians: triangle-wave oscillation ────────────────────────────────
    for cfg in _PEDESTRIAN_DEFS[:num_pedestrians]:
        omni.kit.commands.execute(
            "CreatePrimCommand",
            prim_type="Cylinder",
            prim_path=cfg["path"],
            attributes={"radius": 0.2, "height": 1.8},
        )
        _bind_material(stage, cfg["path"], ped_mat)

        prim = stage.GetPrimAtPath(cfg["path"])
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            op = xform.AddTranslateOp()
            sx, sy, sz = cfg["start"]
            ex, ey, ez = cfg["end"]
            for frame in range(num_frames + 1):
                phase = (frame / num_frames) * 2.0        # 0 → 2
                alpha = phase if phase <= 1.0 else 2.0 - phase  # triangle wave
                op.Set(
                    Gf.Vec3d(sx + alpha * (ex - sx), sy + alpha * (ey - sy), sz),
                    time=frame,
                )

    # ── Vehicles: linear one-way travel ──────────────────────────────────────
    for cfg in _VEHICLE_DEFS[:num_vehicles]:
        omni.kit.commands.execute("CreatePrimCommand", prim_type="Cube", prim_path=cfg["path"])
        _bind_material(stage, cfg["path"], car_mat)
        _add_collision(stage, cfg["path"])

        prim = stage.GetPrimAtPath(cfg["path"])
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(2.2, 0.9, 0.75))  # static car size
            op = xform.AddTranslateOp()
            sx, sy, sz = cfg["start"]
            ex, ey, ez = cfg["end"]
            for frame in range(num_frames + 1):
                alpha = frame / num_frames
                op.Set(Gf.Vec3d(sx + alpha * (ex - sx), sy, sz), time=frame)

    carb.log_info(
        f"Dynamic objects: {num_pedestrians} pedestrian(s), "
        f"{num_vehicles} vehicle(s), {num_frames} animation frames baked."
    )


def main():
    """Main entry point — set up the full scene."""
    # Enable required extensions first
    from enable_extensions import enable_extensions
    enable_extensions()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("No USD stage available")
        return

    # Set up stage defaults
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Build the scene
    create_ground_plane(stage)
    create_urban_environment(stage)
    robot_path = spawn_robot(stage)
    radar_path = attach_radar_sensor(stage, robot_path)
    lidar_path = attach_lidar_sensor(stage, robot_path)

    if radar_path:
        setup_radar_udp_output(stage, radar_path)
    if lidar_path:
        setup_lidar_ros2_publisher(stage, lidar_path)

    carb.log_info("Scene setup complete. Press Play to start simulation.")


if __name__ == "__main__":
    main()
