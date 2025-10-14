
import argparse
from isaaclab.app import AppLauncher
# -----------------------
# CLI
# -----------------------
parser = argparse.ArgumentParser(description="Import astribot s1.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# -----------------------
# Launch Omniverse
# -----------------------
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

S1_CONFIG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/S1",
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=False,
        asset_path=f"/home/zxw/software/isaacgym_wbc_rl/resources/robots/s1/urdf/astribot_s1_whole_body_full.urdf",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            'astribot_torso_joint_1': 0.33,
            'astribot_torso_joint_2': -0.67,
            'astribot_torso_joint_3': 0.455,
            'astribot_torso_joint_4': 0.,
            'astribot_arm_left_joint_1': 0.58,
            'astribot_arm_left_joint_2': 0.1,
            'astribot_arm_left_joint_3': -1.52,
            'astribot_arm_left_joint_4': 1.8,
            'astribot_arm_left_joint_5': -0.0,
            'astribot_arm_left_joint_6': 0.55,
            'astribot_arm_left_joint_7': -0.1,
            'astribot_arm_right_joint_1': 0.5,
            'astribot_arm_right_joint_2': -0.34,
            'astribot_arm_right_joint_3': 0.77,
            'astribot_arm_right_joint_4': 1.57,
            'astribot_arm_right_joint_5': 0.1,
            'astribot_arm_right_joint_6': -0.45,
            'astribot_arm_right_joint_7': 0.8,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "torso": ImplicitActuatorCfg(
            joint_names_expr=[
                "astribot_torso_joint_1",
                "astribot_torso_joint_2",
                "astribot_torso_joint_3",
                "astribot_torso_joint_4",
            ],
            effort_limit_sim={
                "astribot_torso_joint_1": 88.0,
                "astribot_torso_joint_2": 139.0,
                "astribot_torso_joint_3": 88.0,
                "astribot_torso_joint_4": 139.0,
            },
            velocity_limit_sim={
                "astribot_torso_joint_1": 32.0,
                "astribot_torso_joint_2": 20.0,
                "astribot_torso_joint_3": 32.0,
                "astribot_torso_joint_4": 20.0,
            },
            stiffness={
                "astribot_torso_joint_1": 250,
                "astribot_torso_joint_2": 150,
                "astribot_torso_joint_3": 150,
                "astribot_torso_joint_4": 100,
            },
            damping={
                "astribot_torso_joint_1": 12.5,
                "astribot_torso_joint_2": 8,
                "astribot_torso_joint_3": 8,
                "astribot_torso_joint_4": 6,
            },
            armature={
                "astribot_torso_joint_1": 0,
                "astribot_torso_joint_2": 0,
                "astribot_torso_joint_3": 0,
                "astribot_torso_joint_4": 0,
            },
        ),
        "arm_left": ImplicitActuatorCfg(
            joint_names_expr=[
            "astribot_arm_left_joint_1",
            "astribot_arm_left_joint_2",
            "astribot_arm_left_joint_3",
            "astribot_arm_left_joint_4",
            "astribot_arm_left_joint_5",
            "astribot_arm_left_joint_6",
            "astribot_arm_left_joint_7",

            ],
            effort_limit_sim={
            'astribot_arm_left_joint_1': 500.0,
            'astribot_arm_left_joint_2': 500.0,
            'astribot_arm_left_joint_3': 300.0,
            'astribot_arm_left_joint_4': 200.0,
            'astribot_arm_left_joint_5': 60.0,
            'astribot_arm_left_joint_6': 30.0,
            'astribot_arm_left_joint_7': 30.0,
            },
            velocity_limit_sim={
            'astribot_arm_left_joint_1': 500.0,
            'astribot_arm_left_joint_2': 500.0,
            'astribot_arm_left_joint_3': 300.0,
            'astribot_arm_left_joint_4': 200.0,
            'astribot_arm_left_joint_5': 60.0,
            'astribot_arm_left_joint_6': 30.0,
            'astribot_arm_left_joint_7': 30.0,
            },
            stiffness={
            'astribot_arm_left_joint_1': 500.0,
            'astribot_arm_left_joint_2': 500.0,
            'astribot_arm_left_joint_3': 300.0,
            'astribot_arm_left_joint_4': 200.0,
            'astribot_arm_left_joint_5': 60.0,
            'astribot_arm_left_joint_6': 30.0,
            'astribot_arm_left_joint_7': 30.0,
            },
            damping={
            'astribot_arm_left_joint_1': 500.0,
            'astribot_arm_left_joint_2': 500.0,
            'astribot_arm_left_joint_3': 300.0,
            'astribot_arm_left_joint_4': 200.0,
            'astribot_arm_left_joint_5': 60.0,
            'astribot_arm_left_joint_6': 30.0,
            'astribot_arm_left_joint_7': 30.0,
            },
            armature={
            'astribot_arm_left_joint_1': 500.0,
            'astribot_arm_left_joint_2': 500.0,
            'astribot_arm_left_joint_3': 300.0,
            'astribot_arm_left_joint_4': 200.0,
            'astribot_arm_left_joint_5': 60.0,
            'astribot_arm_left_joint_6': 30.0,
            'astribot_arm_left_joint_7': 30.0,
            },
        ),
        "arm_right": ImplicitActuatorCfg(
            joint_names_expr=[
            "astribot_arm_right_joint_1",
            "astribot_arm_right_joint_2",
            "astribot_arm_right_joint_3",
            "astribot_arm_right_joint_4",
            "astribot_arm_right_joint_5",
            "astribot_arm_right_joint_6",
            "astribot_arm_right_joint_7",

            ],
            effort_limit_sim={
            'astribot_arm_right_joint_1': 500.0,
            'astribot_arm_right_joint_2': 500.0,
            'astribot_arm_right_joint_3': 300.0,
            'astribot_arm_right_joint_4': 200.0,
            'astribot_arm_right_joint_5': 60.0,
            'astribot_arm_right_joint_6': 30.0,
            'astribot_arm_right_joint_7': 30.0,
            },
            velocity_limit_sim={
            'astribot_arm_right_joint_1': 500.0,
            'astribot_arm_right_joint_2': 500.0,
            'astribot_arm_right_joint_3': 300.0,
            'astribot_arm_right_joint_4': 200.0,
            'astribot_arm_right_joint_5': 60.0,
            'astribot_arm_right_joint_6': 30.0,
            'astribot_arm_right_joint_7': 30.0,
            },
            stiffness={
            'astribot_arm_right_joint_1': 500.0,
            'astribot_arm_right_joint_2': 500.0,
            'astribot_arm_right_joint_3': 300.0,
            'astribot_arm_right_joint_4': 200.0,
            'astribot_arm_right_joint_5': 60.0,
            'astribot_arm_right_joint_6': 30.0,
            'astribot_arm_right_joint_7': 30.0,
            },
            damping={
            'astribot_arm_right_joint_1': 500.0,
            'astribot_arm_right_joint_2': 500.0,
            'astribot_arm_right_joint_3': 300.0,
            'astribot_arm_right_joint_4': 200.0,
            'astribot_arm_right_joint_5': 60.0,
            'astribot_arm_right_joint_6': 30.0,
            'astribot_arm_right_joint_7': 30.0,
            },
            armature={
            'astribot_arm_right_joint_1': 500.0,
            'astribot_arm_right_joint_2': 500.0,
            'astribot_arm_right_joint_3': 300.0,
            'astribot_arm_right_joint_4': 200.0,
            'astribot_arm_right_joint_5': 60.0,
            'astribot_arm_right_joint_6': 30.0,
            'astribot_arm_right_joint_7': 30.0,
            },
        ),
    },
)

class S1SceneCfg(InteractiveSceneCfg):
    """只包含地面、光照和 Kaya。"""
    # ground
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    # light
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    # robot
    S1 = S1_CONFIG

def run(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    joint_names = list(scene["S1"].data.joint_names)


    num_envs = scene.num_envs
    device = sim.device

    while simulation_app.is_running():
        # 每 500 步复位一次
        if count % 500 == 0:
            count = 0
            # 复位到默认 root state，并按环境原点平移
            root_state = scene["S1"].data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            scene["S1"].write_root_pose_to_sim(root_state[:, :7])
            scene["S1"].write_root_velocity_to_sim(root_state[:, 7:])
            # 复位关节
            joint_pos = scene["S1"].data.default_joint_pos.clone()
            joint_vel = scene["S1"].data.default_joint_vel.clone()
            scene["S1"].write_joint_state_to_sim(joint_pos, joint_vel)
            scene.reset()
            print("[INFO] Resetting S1 state...")


        # 写入并步进
        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)

def main():
    # 仿真上下文
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # 视角
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # 场景
    scene_cfg = S1SceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # 开始
    sim.reset()
    print("[INFO]: Setup complete...")
    run(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()