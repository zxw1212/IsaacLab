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
# app_launcher = AppLauncher(args_cli)
app_launcher = AppLauncher(headless=False)
# app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg, IdealPDActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sim.spawners import materials


import torch

KAYA_CONFIG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/astribot_base",
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=False,
        asset_path=f"/home/zxw/Downloads/kaya/urdf/kaya.urdf",
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
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
            target_type="velocity",
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            '.*': 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "actuated_wheel_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "axle_0_joint",
                "axle_1_joint",
                "axle_2_joint",
            ],
            effort_limit_sim={
                ".*": 30000.0,
            },
            velocity_limit_sim={
                ".*": 1000.0,
            },
            stiffness={
                # P=0 for velocity control
                ".*": 0.0,
            },
            damping={
                # ".*": 0.0,
                ".*": 65.0,
            },
            armature={
                ".*": 0.001,
            },
            friction=None,# use default friction in usd
            # friction={".*": 0.1},
        ),
        # passive joints
        # "passive_rollers": ImplicitActuatorCfg(
        #     joint_names_expr=[
        #         "wheel_.*_roller_Joint_.*",
        #     ],
        #     stiffness={".*": 0.0},
        #     damping={".*": 0.0},
        #     armature={".*": 0.0},
        #     friction=None,# use default friction in usd
        #     # friction={".*": 0.1},
        # ),
    },
)

class S1SceneCfg(InteractiveSceneCfg):
    """只包含地面、光照和 Kaya。"""
    # ground
    # ground = AssetBaseCfg(
    #     prim_path="/World/Ground",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Terrains/rough_plane.usd",
    #         scale=(1, 1, 1),
    #     ),
    # )
    # ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            # physics_material=materials.RigidBodyMaterialCfg(static_friction=0, dynamic_friction=0, restitution=0.0, friction_combine_mode="max")
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                restitution = 0.0,
                static_friction=1.0,
                dynamic_friction=0.8,
            ),
        )
    )

    # light
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    # robot
    kaya = KAYA_CONFIG

def run(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    print(f"[INFO] Simulation dt: {sim_dt:.4f}")
    sim_time = 0.0
    count = 0
    joint_names = list(scene["kaya"].data.joint_names)
    print("[DEBUG] Loaded joint names:", joint_names) # all joints
    actutors_names = scene["kaya"].actuators.keys()
    print("[DEBUG] Loaded actuator names:", type(actutors_names), actutors_names) # class 'dict_keys' dict_keys(['actuated_wheel_joints'])
    acttuated_joints = scene["kaya"].actuators["actuated_wheel_joints"].joint_names
    print("[DEBUG] Actuated wheel joints:", acttuated_joints) # ['wheel_LF_Joint', 'wheel_LR_Joint', 'wheel_RF_Joint', 'wheel_RR_Joint']



    num_envs = scene.num_envs
    device = sim.device
    
    while simulation_app.is_running():
        # 每 500 步复位一次
        if count % 500 == 0:
            count = 0
            # 复位到默认 root state，并按环境原点平移
            root_state = scene["kaya"].data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            scene["kaya"].write_root_pose_to_sim(root_state[:, :7])
            scene["kaya"].write_root_velocity_to_sim(root_state[:, 7:])
            # 复位关节
            joint_pos = scene["kaya"].data.default_joint_pos.clone()
            joint_vel = scene["kaya"].data.default_joint_vel.clone()
            scene["kaya"].write_joint_state_to_sim(joint_pos, joint_vel)
            scene.reset()
            print("[INFO] Resetting kaya state...")

        # === 控到默认位置 ===
        # targets = scene["kaya"].data.default_joint_pos
        # # -- apply action to the robot
        # scene["kaya"].set_joint_position_target(targets)

        # # torque control using pd
        # # note to enable ImplicitActuatorCfg kd == 0.0
        # wheel_vel_cmd = [0.0, 4.0, -4.0]
        # kp = 0.7
        # kd = 0.00
        # current_wheel_vel = scene["kaya"].data.joint_vel[:, [joint_names.index(name) for name in acttuated_joints]] # shape (num_envs, 3)
        # current_wheel_acc = scene["kaya"].data.joint_acc[:, [joint_names.index(name) for name in acttuated_joints]] # shape (num_envs, 3)
        # # cal wheel tau cmd using: tau = k_p * (vel_cmd - vel_fb) - k_d * acc_fb
        # wheel_torque_cmd = kp * (torch.tensor(wheel_vel_cmd, device=device).unsqueeze(0).repeat(num_envs, 1) - current_wheel_vel) - kd * current_wheel_acc
        # # print wheel torque cmd
        # # if count % 10 == 0:
        # #     print(f"[DEBUG] Wheel torque command at sim time {sim_time:.2f}s:", wheel_torque_cmd[0].cpu().numpy())
        # # ===>直接设置这几个关节的力矩命令
        # scene["kaya"].set_joint_effort_target(
        #     target=wheel_torque_cmd,
        #     joint_ids=[joint_names.index(name) for name in acttuated_joints], # 只设置主动轮
        #     env_ids=None # 所有环境一起设置
        # )
        
        # # vel control
        # note to enable ImplicitActuatorCfg kd != 0.0
        # ===>直接设置主动轮的速度命令
        wheel_speed = torch.tensor([[1.0, 1.0, 1.0]], device=device).repeat(num_envs, 1)
        # 直接设置这几个关节的目标速度
        scene["kaya"].set_joint_velocity_target(
            target=wheel_speed,
            joint_ids=[joint_names.index(name) for name in acttuated_joints],   # 只设置主动轮
            env_ids=None           # 所有环境一起设置
        )

        # # print wheel joint vel feedback
        if count % 1 == 0:
            joint_vel_feedback = scene["kaya"].data.joint_vel.clone()
            wheel_vel_feedback = {name: joint_vel_feedback[0, joint_names.index(name)].item() for name in acttuated_joints}
            print(f"[DEBUG] Wheel joint velocity feedback at sim time {sim_time:.2f}s:", wheel_vel_feedback)

        # 写入并步进
        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)

def main():
    # 仿真上下文 # 大质量度场景建议用更小的 dt，比如 1/500
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 1.0/500.0, render_interval=1)
    # sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 1.0/60.0, render_interval=1)
    # sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 1.0 / 40.0) # sim fps * dt * render_interval == 1 :仿真速度等于真实, <1 :仿真慢于真实, >1 :仿真快于真实
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
    main()
    simulation_app.close()