#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2026 İDA - ANA OTONOM LAUNCH

TF sahipliği:
    map -> odom        : bu launch içindeki static TF
    odom -> base_link  : MAVROS local_position TF
    base_link -> ZED   : static montaj TF

Not:
- LiDAR/scan tabanlı slam_toolbox bu sürümde başlatılmaz.
- Kamera tabanlı yerel engel haritası data_logger_node tarafından
  /camera_local_costmap (nav_msgs/OccupancyGrid) olarak yayınlanır.
- ZED positional tracking TF yayınlamaz; navigasyon pozisyonunun sahibi MAVROS'tur.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    usv_share = get_package_share_directory('usv_navigation')
    mavros_share = get_package_share_directory('mavros')
    zed_share = get_package_share_directory('zed_wrapper')

    mavros_config_yolu = os.path.join(
        usv_share, 'config', 'mavros_params.yaml'
    )
    apm_pluginlists = os.path.join(
        mavros_share, 'launch', 'apm_pluginlists.yaml'
    )
    apm_config = os.path.join(
        mavros_share, 'launch', 'apm_config.yaml'
    )
    zed_config = os.path.join(
        usv_share, 'config', 'zed.yaml'
    )
    model_default = os.path.join(
        usv_share, 'models', 'best.pt'
    )

    arg_hedef = DeclareLaunchArgument(
        'carpilacak_hedef',
        default_value='',
        description=(
            'Hedef duba rengi: kirmizi / yesil / siyah. '
            'Boşsa /carpilacakhedef topiği beklenir.'
        ),
    )

    arg_model = DeclareLaunchArgument(
        'model_path',
        default_value=model_default,
        description='YOLO modeli (.pt/.engine/.onnx)',
    )

    arg_fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:115200',
        description=(
            'MAVROS FCU seri bağlantısı. Gerçek aygıt yolu launch argümanıyla '
            'belirtilmelidir; /dev/ttyACM0 yalnızca yaygın bir örnektir.'
        ),
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        ros_arguments=['--log-level', 'WARN'],
        parameters=[
            apm_pluginlists,
            apm_config,
            mavros_config_yolu,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'local_position.frame_id': 'odom',
                'local_position.tf.send': True,
                'local_position.tf.frame_id': 'odom',
                'local_position.tf.child_frame_id': 'base_link',
                'global_position.tf.send': False,
            },
        ],
    )

    # YKİ -> RFD -> Cube -> MAVROS raw MAVLink -> /carpilacakhedef
    # Cube seri portunu yalnızca MAVROS açar; haberlesme node'u seri porta
    # doğrudan erişmez.
    haberlesme_node = Node(
        package='usv_navigation',
        executable='haberlesme',
        name='ida_hedef_renk_al',
        output='screen',
        parameters=[{
            # Yarış YKİ köprüsünün doğrulanmış MAVLink kimliği.
            # Başka bir MAVLink kaynağının hedef rengini yanlışlıkla kilitlemesini önler.
            'beklenen_yki_sistem_id': 201,
            'beklenen_yki_bilesen_id': 190,
        }],
    )

    # SLAM bu sürümde kapalı olduğundan map ve odom aynı başlangıç çerçevesidir.
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    tf_base_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_zed',
        arguments=[
            '--x', '0.40',
            '--y', '0.00',
            '--z', '0.25',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link',
        ],
    )

    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps',
        arguments=[
            '--x', '-0.15',
            '--y', '0.00',
            '--z', '0.45',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'gps_link',
        ],
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_share, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed',
            'ros_params_override_path': zed_config,
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_urdf': 'true',
            'node_log_type': 'screen',
        }.items(),
    )

    los_node = Node(
        package='usv_navigation',
        executable='los_node',
        name='los_node',
        output='screen',
        parameters=[{
            'acceptance_radius': 0.75,
            'start_waypoint_index': 1,
            'donus_esigi_derece': 30.0,
            'estop_timeout_sec': 1.5,
        }],
    )

    obstacle_avoidance_node = Node(
        package='usv_navigation',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'carpilacak_hedef': LaunchConfiguration(
                'carpilacak_hedef'
            ),
            'conf_threshold': 0.45,
            'guvenli_mesafe': 1.5,
            'geri_vites_tetik_mesafe': 0.5,
            'koridor_varsayilan_yaricap': 2.5,
            'pruva_koridor_yaricap': 0.60,
            'perception_timeout_sec': 0.8,
            'guidance_timeout_sec': 1.0,
            'rgb_depth_max_skew_sec': 0.35,
            'max_depth_m': 10.0,
            'min_valid_depth_pixels': 5,
            'final_charge_trigger_m': 0.55,
            'final_charge_duration_sec': 1.0,
        }],
    )

    pid_node = Node(
        package='usv_navigation',
        executable='pid_node',
        name='pid_node',
        output='screen',
        parameters=[{
            'Kp': 1.8,
            'Ki': 0.05,
            'Kd': 0.25,
            'normal_seyir_hizi': 1.3,
            'geri_vites_hizi': 0.5,
            'max_donus_hizi': 1.5,
            'integral_limit': 0.5,
            'estop_timeout_sec': 1.5,
        }],
    )

    data_logger_node = Node(
        package='usv_navigation',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[{
            'video_fps': 10.0,
            # /stop=0 bir güvenli duruş sinyali de olabildiğinden kayıt
            # yalnızca DISARM/shutdown ile kapanır.
            'seal_on_stop': False,
            'max_depth_m': 10.0,
            'map_publish_rate_hz': 2.0,
        }],
    )

    estop_node = Node(
        package='usv_navigation',
        executable='estop',
        name='estop',
        output='screen',
        # GPIO/RC düğümü geçici bir hata ile kapanırsa tekrar ayağa kalksın.
        # Yeniden başlayana kadar LOS ve PID heartbeat watchdog'u aracı durdurur.
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'gpio_pin': 7,
            'pwm_threshold': 1600,
            'rc_timeout_sec': 1.0,
        }],
    )

    return LaunchDescription([
        arg_hedef,
        arg_model,
        arg_fcu_url,
        estop_node,
        mavros_node,
        haberlesme_node,
        tf_map_to_odom,
        tf_base_to_zed,
        tf_base_to_gps,
        zed_launch,
        los_node,
        obstacle_avoidance_node,
        pid_node,
        data_logger_node,
    ])
