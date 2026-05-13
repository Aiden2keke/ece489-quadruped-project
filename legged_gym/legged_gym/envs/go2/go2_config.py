from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class GO2BaseCfg(LeggedRobotCfg):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.42] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "Head_lower"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.5, 1.2]
        randomize_base_mass = True
        # Go2 URDF total mass is approximately 15.0 kg, so +/-3 kg is +/-20%.
        added_mass_range = [-3.0, 3.0]
        randomize_motor = True
        motor_strength_range = [0.9, 1.1]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.0
        randomize_Kp_factor = True
        Kp_factor_range = [0.8, 1.2]
        randomize_Kd_factor = True
        Kd_factor_range = [0.8, 1.2]
        randomize_action_delay = True
        delay_ms_range = [0, 10]

    class commands(LeggedRobotCfg.commands):
        curriculum = True
        max_curriculum = 2.0
        max_yaw = 2.0
        curriculum_lin_vel_step = 0.25
        curriculum_yaw_step = 0.15
        curriculum_tracking_reward_threshold = 0.85
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.25
        foot_clearance_target = 0.08
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -1.0
            tracking_lin_vel = 1.2
            orientation = -1.2
            torques = -0.0002
            dof_acc = -3.0e-7
            collision = -1.2
            action_rate = -0.012
            action_smoothness = -0.0012
            dof_pos_limits = -10.0
            feet_clearance = -0.5


class GO2FlatCfg(GO2BaseCfg):
    class terrain(GO2BaseCfg.terrain):
        mesh_type = 'plane'
        curriculum = False


class GO2RoughCfg(GO2BaseCfg):
    class terrain(GO2BaseCfg.terrain):
        mesh_type = 'trimesh'
        curriculum = True
        measure_heights = True
        max_init_terrain_level = 0
        curriculum_tracking_reward_up = 0.60
        curriculum_tracking_reward_down = 0.35
        curriculum_successes_before_level_up = 3
        curriculum_failures_before_level_down = 1
        curriculum_min_episode_fraction = 0.85
        curriculum_early_done_fraction = 0.60
        terrain_proportions = [0.2, 0.3, 0.1, 0.1, 0.3, 0.0, 0.0, 0.0]


class GO2FlatCfgPPO(LeggedRobotCfgPPO):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        max_iterations = 7500
        experiment_name = 'rough_go2'


class GO2RoughCfgPPO(GO2FlatCfgPPO):
    class runner(GO2FlatCfgPPO.runner):
        experiment_name = 'rough_go2_challenging'
