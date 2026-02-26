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
7. Configures TranscoderRadar OmniGraph node for UDP output
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
        omni.kit.commands.execute(
            "CreatePrimCommand",
            prim_type="Cube",
            prim_path=path,
            attributes={
                "xformOp:translate": Gf.Vec3d(tx, ty, tz),
                "xformOp:scale":     Gf.Vec3f(sx / 2, sy / 2, sz / 2),
            },
        )
        _add_collision(stage, path)
        _bind_material(stage, path, concrete_mat)

    # ── Walls / barriers ──────────────────────────────────────────────────────
    walls = [
        ("/World/Urban/Wall_1", (10.0,  0.0, 1.5), (10.0, 0.3, 3.0)),
        ("/World/Urban/Wall_2", (12.0, -12.0, 1.0), (8.0,  0.3, 2.0)),
    ]
    for path, (tx, ty, tz), (sx, sy, sz) in walls:
        omni.kit.commands.execute(
            "CreatePrimCommand",
            prim_type="Cube",
            prim_path=path,
            attributes={
                "xformOp:translate": Gf.Vec3d(tx, ty, tz),
                "xformOp:scale":     Gf.Vec3f(sx / 2, sy / 2, sz / 2),
            },
        )
        _add_collision(stage, path)
        _bind_material(stage, path, concrete_mat)

    # ── Metal pillars (strong radar return) ───────────────────────────────────
    pillars = [
        ("/World/Urban/Pillar_1", ( 8.0, -3.0, 2.0)),
        ("/World/Urban/Pillar_2", ( 9.0,  5.0, 2.0)),
        ("/World/Urban/Pillar_3", ( 7.0, -6.0, 2.0)),
    ]
    for path, (tx, ty, tz) in pillars:
        omni.kit.commands.execute(
            "CreatePrimCommand",
            prim_type="Cylinder",
            prim_path=path,
            attributes={
                "xformOp:translate": Gf.Vec3d(tx, ty, tz),
                "xformOp:scale":     Gf.Vec3f(0.3, 0.3, 2.0),
            },
        )
        _add_collision(stage, path)
        _bind_material(stage, path, metal_mat)

    carb.log_info("Urban environment created: 4 buildings, 2 walls, 3 pillars.")


def create_ground_plane(stage):
    """Create a simple ground plane for testing."""
    omni.kit.commands.execute(
        "CreatePrimCommand",
        prim_type="Plane",
        prim_path="/World/GroundPlane",
        attributes={
            "xformOp:scale": Gf.Vec3f(100.0, 100.0, 1.0),
        },
    )
    # Add collision
    prim = stage.GetPrimAtPath("/World/GroundPlane")
    if prim.IsValid():
        UsdPhysics.CollisionAPI.Apply(prim)


def spawn_robot(stage, robot_usd: str = ROBOT_USD):
    """Reference the robot USD into the stage."""
    robot_path = "/World/Robot"

    # Try loading from Nucleus; fall back to creating a placeholder
    try:
        omni.kit.commands.execute(
            "CreateReferenceCommand",
            usd_context=omni.usd.get_context(),
            path_to=robot_path,
            asset_path=robot_usd,
        )
        carb.log_info(f"Loaded robot from {robot_usd}")
    except Exception as e:
        carb.log_warn(f"Could not load robot USD ({e}). Creating placeholder cube.")
        omni.kit.commands.execute(
            "CreatePrimCommand",
            prim_type="Cube",
            prim_path=robot_path,
            attributes={
                "xformOp:scale": Gf.Vec3f(0.5, 0.3, 0.2),
                "xformOp:translate": Gf.Vec3d(0.0, 0.0, 0.2),
            },
        )

    return robot_path


def attach_radar_sensor(stage, parent_path: str):
    """Create an RTX Radar sensor attached to the robot."""
    radar_config = load_config("radar_params.yaml")
    radar_path = f"{parent_path}/RadarSensor"

    # Create the RTX Radar sensor prim
    omni.kit.commands.execute(
        "IsaacSensorCreateRtxRadar",
        path=radar_path,
    )

    radar_prim = stage.GetPrimAtPath(radar_path)
    if not radar_prim.IsValid():
        carb.log_error("Failed to create RTX Radar sensor")
        return None

    # Set radar model to WpmDmatApproxRadar.
    # In Isaac Sim 5.x, IsaacSensorCreateRtxRadar creates an OmniRadar prim
    # (not a Camera prim). The model config attribute is the same but guarded
    # defensively — WpmDmatApprox is the default model so this is a no-op if
    # the attribute is absent.
    for attr_name in ("rtxsensor:modelConfig", "omni:sensor:config"):
        if radar_prim.HasAttribute(attr_name):
            radar_prim.GetAttribute(attr_name).Set("WpmDmatApproxRadar")
            break
    else:
        carb.log_info("Radar model attribute not found; using sensor default (WpmDmatApprox).")

    # Apply mount offset from config
    mount = radar_config.get("mount_offset", {})
    xform = UsdGeom.Xformable(radar_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(
        mount.get("x", 0.3),
        mount.get("y", 0.0),
        mount.get("z", 0.4),
    ))

    carb.log_info(f"RTX Radar sensor created at {radar_path}")
    return radar_path


def attach_lidar_sensor(stage, parent_path: str):
    """Create an RTX Lidar sensor attached to the robot."""
    lidar_config = load_config("lidar_params.yaml")
    lidar_path = f"{parent_path}/LidarSensor"

    omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_path,
        config=lidar_config.get("config_name", "OS1_64"),
    )

    lidar_prim = stage.GetPrimAtPath(lidar_path)
    if not lidar_prim.IsValid():
        carb.log_error("Failed to create RTX Lidar sensor")
        return None

    # Apply mount offset
    mount = lidar_config.get("mount_offset", {})
    xform = UsdGeom.Xformable(lidar_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(
        mount.get("x", 0.0),
        mount.get("y", 0.0),
        mount.get("z", 0.5),
    ))

    carb.log_info(f"RTX Lidar sensor created at {lidar_path}")
    return lidar_path


def setup_radar_udp_output(stage, radar_path: str):
    """Configure the TranscoderRadar OmniGraph node for UDP output."""
    radar_config = load_config("radar_params.yaml")
    udp = radar_config.get("udp", {})

    # Create an OmniGraph action graph for radar transcoding
    graph_path = "/World/RadarGraph"

    try:
        import omni.graph.core as og

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("TranscoderRadar", "omni.sensors.nv.radar.TranscoderRadar"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "TranscoderRadar.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath",
                     "TranscoderRadar.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", radar_path),
                    ("TranscoderRadar.inputs:remoteIP",
                     udp.get("remote_ip", "239.0.0.1")),
                    ("TranscoderRadar.inputs:remotePort",
                     udp.get("remote_port", 10001)),
                    ("TranscoderRadar.inputs:interfaceIP",
                     udp.get("interface_ip", "0.0.0.0")),
                ],
            },
        )
        carb.log_info(f"Radar UDP output graph created at {graph_path}")
    except Exception as e:
        carb.log_warn(
            f"Could not create radar OmniGraph ({e}). "
            "You may need to set up the TranscoderRadar node manually."
        )


def setup_lidar_ros2_publisher(stage, lidar_path: str):
    """Configure OmniGraph to publish LiDAR data to ROS2."""
    lidar_config = load_config("lidar_params.yaml")
    graph_path = "/World/LidarROS2Graph"

    try:
        import omni.graph.core as og

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ROS2Publisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "ROS2Publisher.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath",
                     "ROS2Publisher.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", lidar_path),
                    ("ROS2Publisher.inputs:topicName",
                     lidar_config.get("ros2_topic", "/lidar/point_cloud")),
                    ("ROS2Publisher.inputs:frameId",
                     lidar_config.get("frame_id", "lidar_link")),
                    ("ROS2Publisher.inputs:type", "point_cloud"),
                ],
            },
        )
        carb.log_info(f"LiDAR ROS2 publisher graph created at {graph_path}")
    except Exception as e:
        carb.log_warn(
            f"Could not create LiDAR ROS2 graph ({e}). "
            "Use Tools > Robotics > ROS2 OmniGraphs > RTX LiDAR instead."
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
