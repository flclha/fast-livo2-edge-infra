#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""This file generates shell code for the setup.SHELL scripts to set environment variables."""

from __future__ import print_function

import argparse
import copy
import errno
import os
import platform
import sys

CATKIN_MARKER_FILE = '.catkin'

system = platform.system()
IS_DARWIN = (system == 'Darwin')
IS_WINDOWS = (system == 'Windows')

PATH_TO_ADD_SUFFIX = ['bin']
if IS_WINDOWS:
    # while catkin recommends putting dll's into bin, 3rd party packages often put dll's into lib
    # since Windows finds dll's via the PATH variable, prepend it with path to lib
    PATH_TO_ADD_SUFFIX.extend([['lib', os.path.join('lib', 'x86_64-linux-gnu')]])

# subfolder of workspace prepended to CMAKE_PREFIX_PATH
ENV_VAR_SUBFOLDERS = {
    'CMAKE_PREFIX_PATH': '',
    'LD_LIBRARY_PATH' if not IS_DARWIN else 'DYLD_LIBRARY_PATH': ['lib', os.path.join('lib', 'x86_64-linux-gnu')],
    'PATH': PATH_TO_ADD_SUFFIX,
    'PKG_CONFIG_PATH': [os.path.join('lib', 'pkgconfig'), os.path.join('lib', 'x86_64-linux-gnu', 'pkgconfig')],
    'PYTHONPATH': 'lib/python3/dist-packages',
}


def rollback_env_variables(environ, env_var_subfolders):
    """
    Generate shell code to reset environment variables.

    by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH.
    This does not cover modifications performed by environment hooks.
    """
    lines = []
    unmodified_environ = copy.copy(environ)
    for key in sorted(env_var_subfolders.keys()):
        subfolders = env_var_subfolders[key]
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        value = _rollback_env_variable(unmodified_environ, key, subfolders)
        if value is not None:
            environ[key] = value
            lines.append(assignment(key, value))
    if lines:
        lines.insert(0, comment('reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH'))
    return lines


def _rollback_env_variable(environ, name, subfolders):
    """
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolders: list of str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    """
    value = environ[name] if name in environ else ''
    env_paths = [path for path in value.split(os.pathsep) if path]
    value_modified = False
    for subfolder in subfolders:
        if subfolder:
            if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
                subfolder = subfolder[1:]
            if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
                subfolder = subfolder[:-1]
        for ws_path in _get_workspaces(environ, include_fuerte=True, include_non_existing=True):
            path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
            path_to_remove = None
            for env_path in env_paths:
                env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
                if env_path_clean == path_to_find:
                    path_to_remove = env_path
                    break
            if path_to_remove:
                env_paths.remove(path_to_remove)
                value_modified = True
    new_value = os.pathsep.join(env_paths)
    return new_value if value_modified else None


def _get_workspaces(environ, include_fuerte=False, include_non_existing=False):
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces.

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    value = environ[env_name] if env_name in environ else ''
    paths = [path for path in value.split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte')) or (include_non_existing and not os.path.exists(path))]
    return workspaces


def prepend_env_variables(environ, env_var_subfolders, workspaces):
    """Generate shell code to prepend environment variables for the all workspaces."""
    lines = []
    lines.append(comment('prepend folders of workspaces to environment variables'))

    paths = [path for path in workspaces.split(os.pathsep) if path]

    prefix = _prefix_env_variable(environ, 'CMAKE_PREFIX_PATH', paths, '')
    lines.append(prepend(environ, 'CMAKE_PREFIX_PATH', prefix))

    for key in sorted(key for key in env_var_subfolders.keys() if key != 'CMAKE_PREFIX_PATH'):
        subfolder = env_var_subfolders[key]
        prefix = _prefix_env_variable(environ, key, paths, subfolder)
        lines.append(prepend(environ, key, prefix))
    return lines


def _prefix_env_variable(environ, name, paths, subfolders):
    """
    Return the prefix to prepend to the environment variable NAME.

    Adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    """
    value = environ[name] if name in environ else ''
    environ_paths = [path for path in value.split(os.pathsep) if path]
    checked_paths = []
    for path in paths:
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        for subfolder in subfolders:
            path_tmp = path
            if subfolder:
                path_tmp = os.path.join(path_tmp, subfolder)
            # skip nonexistent paths
            if not os.path.exists(path_tmp):
                continue
            # exclude any path already in env and any path we already added
            if path_tmp not in environ_paths and path_tmp not in checked_paths:
                checked_paths.append(path_tmp)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def assignment(key, value):
    if not IS_WINDOWS:
        return 'export %s="%s"' % (key, value)
    else:
        return 'set %s=%s' % (key, value)


def comment(msg):
    if not IS_WINDOWS:
        return '# %s' % msg
    else:
        return 'REM %s' % msg


def prepend(environ, key, prefix):
    if key not in environ or not environ[key]:
        return assignment(key, prefix)
    if not IS_WINDOWS:
        return 'export %s="%s$%s"' % (key, prefix, key)
    else:
        return 'set %s=%s%%%s%%' % (key, prefix, key)


def find_env_hooks(environ, cmake_prefix_path):
    """Generate shell code with found environment hooks for the all workspaces."""
    lines = []
    lines.append(comment('found environment hooks in workspaces'))

    generic_env_hooks = []
    generic_env_hooks_workspace = []
    specific_env_hooks = []
    specific_env_hooks_workspace = []
    generic_env_hooks_by_filename = {}
    specific_env_hooks_by_filename = {}
    generic_env_hook_ext = 'bat' if IS_WINDOWS else 'sh'
    specific_env_hook_ext = environ['CATKIN_SHELL'] if not IS_WINDOWS and 'CATKIN_SHELL' in environ and environ['CATKIN_SHELL'] else None
    # remove non-workspace paths
    workspaces = [path for path in cmake_prefix_path.split(os.pathsep) if path and os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    for workspace in reversed(workspaces):
        env_hook_dir = os.path.join(workspace, 'etc', 'catkin', 'profile.d')
        if os.path.isdir(env_hook_dir):
            for filename in sorted(os.listdir(env_hook_dir)):
                if filename.endswith('.%s' % generic_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in generic_env_hooks_by_filename:
                        i = generic_env_hooks.index(generic_env_hooks_by_filename[filename])
                        generic_env_hooks.pop(i)
                        generic_env_hooks_workspace.pop(i)
                    # append env hook
                    generic_env_hooks.append(os.path.join(env_hook_dir, filename))
                    generic_env_hooks_workspace.append(workspace)
                    generic_env_hooks_by_filename[filename] = generic_env_hooks[-1]
                elif specific_env_hook_ext is not None and filename.endswith('.%s' % specific_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in specific_env_hooks_by_filename:
                        i = specific_env_hooks.index(specific_env_hooks_by_filename[filename])
                        specific_env_hooks.pop(i)
                        specific_env_hooks_workspace.pop(i)
                    # append env hook
                    specific_env_hooks.append(os.path.join(env_hook_dir, filename))
                    specific_env_hooks_workspace.append(workspace)
                    specific_env_hooks_by_filename[filename] = specific_env_hooks[-1]
    env_hooks = generic_env_hooks + specific_env_hooks
    env_hooks_workspace = generic_env_hooks_workspace + specific_env_hooks_workspace
    count = len(env_hooks)
    lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_COUNT', count))
    for i in range(count):
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d' % i, env_hooks[i]))
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d_WORKSPACE' % i, env_hooks_workspace[i]))
    return lines


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    parser.add_argument('--extend', action='store_true', help='Skip unsetting previous environment variables to extend context')
    parser.add_argument('--local', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    return parser.parse_known_args(args=args)[0]


if __name__ == '__main__':
    try:
        try:
            args = _parse_arguments()
        except Exception as e:
            print(e, file=sys.stderr)
            sys.exit(1)

        if not args.local:
            # environment at generation time
            CMAKE_PREFIX_PATH = r'/home/nuc/catkin_ws/devel;/opt/ros/noetic;/home/nuc/geoinfo/install/geographic_info;/home/nuc/geoinfo/install/geodesy;/home/nuc/geoinfo/install/geographic_msgs;/home/nuc/geoinfo/install/angles;/home/nuc/ardu_ws/install/ardupilot_sitl;/home/nuc/ardu_ws/install/micro_ros_agent;/home/nuc/ardu_ws/install/ardupilot_msgs;/home/nuc/uros_ws/install/micro_ros_agent;/home/nuc/uros_ws/install/micro_ros_msgs;/home/nuc/ros2_humble/install/rosbag2_storage_mcap;/home/nuc/ros2_humble/install/rosbag2;/home/nuc/ros2_humble/install/rosbag2_compression_zstd;/home/nuc/ros2_humble/install/mcap_vendor;/home/nuc/ros2_humble/install/zstd_vendor;/home/nuc/ros2_humble/install/rviz_visual_testing_framework;/home/nuc/ros2_humble/install/rviz2;/home/nuc/ros2_humble/install/rviz_default_plugins;/home/nuc/ros2_humble/install/rviz_common;/home/nuc/ros2_humble/install/rosbag2_py;/home/nuc/ros2_humble/install/rosbag2_transport;/home/nuc/ros2_humble/install/rosbag2_storage_default_plugins;/home/nuc/ros2_humble/install/rosbag2_performance_benchmarking;/home/nuc/ros2_humble/install/rosbag2_compression;/home/nuc/ros2_humble/install/rosbag2_cpp;/home/nuc/ros2_humble/install/rosbag2_storage;/home/nuc/ros2_humble/install/image_common;/home/nuc/ros2_humble/install/camera_info_manager;/home/nuc/ros2_humble/install/camera_calibration_parsers;/home/nuc/ros2_humble/install/yaml_cpp_vendor;/home/nuc/ros2_humble/install/interactive_markers;/home/nuc/ros2_humble/install/common_interfaces;/home/nuc/ros2_humble/install/visualization_msgs;/home/nuc/ros2_humble/install/dummy_robot_bringup;/home/nuc/ros2_humble/install/robot_state_publisher;/home/nuc/ros2_humble/install/kdl_parser;/home/nuc/ros2_humble/install/urdf;/home/nuc/ros2_humble/install/urdfdom;/home/nuc/ros2_humble/install/urdf_parser_plugin;/home/nuc/ros2_humble/install/urdfdom_headers;/home/nuc/ros2_humble/install/turtlesim;/home/nuc/ros2_humble/install/geometry2;/home/nuc/ros2_humble/install/tf2_sensor_msgs;/home/nuc/ros2_humble/install/test_tf2;/home/nuc/ros2_humble/install/tf2_kdl;/home/nuc/ros2_humble/install/tf2_geometry_msgs;/home/nuc/ros2_humble/install/tf2_eigen;/home/nuc/ros2_humble/install/tf2_bullet;/home/nuc/ros2_humble/install/tf2_ros;/home/nuc/ros2_humble/install/tf2_py;/home/nuc/ros2_humble/install/tf2_msgs;/home/nuc/ros2_humble/install/test_msgs;/home/nuc/ros2_humble/install/sros2_cmake;/home/nuc/ros2_humble/install/ros2cli_common_extensions;/home/nuc/ros2_humble/install/rqt_py_common;/home/nuc/ros2_humble/install/rosbag2_storage_mcap_testdata;/home/nuc/ros2_humble/install/ros_testing;/home/nuc/ros2_humble/install/ros2cli_test_interfaces;/home/nuc/ros2_humble/install/quality_of_service_demo_cpp;/home/nuc/ros2_humble/install/image_transport;/home/nuc/ros2_humble/install/message_filters;/home/nuc/ros2_humble/install/demo_nodes_cpp;/home/nuc/ros2_humble/install/composition;/home/nuc/ros2_humble/install/laser_geometry;/home/nuc/ros2_humble/install/rclpy;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_action_server;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_action_client;/home/nuc/ros2_humble/install/action_tutorials_cpp;/home/nuc/ros2_humble/install/rclcpp_action;/home/nuc/ros2_humble/install/rcl_action;/home/nuc/ros2_humble/install/examples_rclcpp_wait_set;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_service;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_client;/home/nuc/ros2_humble/install/examples_rclcpp_async_client;/home/nuc/ros2_humble/install/example_interfaces;/home/nuc/ros2_humble/install/action_tutorials_interfaces;/home/nuc/ros2_humble/install/action_msgs;/home/nuc/ros2_humble/install/unique_identifier_msgs;/home/nuc/ros2_humble/install/ament_lint_common;/home/nuc/ros2_humble/install/ament_cmake_uncrustify;/home/nuc/ros2_humble/install/uncrustify_vendor;/home/nuc/ros2_humble/install/trajectory_msgs;/home/nuc/ros2_humble/install/topic_statistics_demo;/home/nuc/ros2_humble/install/pendulum_control;/home/nuc/ros2_humble/install/tlsf_cpp;/home/nuc/ros2_humble/install/test_tracetools;/home/nuc/ros2_humble/install/rqt_gui_cpp;/home/nuc/ros2_humble/install/rosbag2_test_common;/home/nuc/ros2_humble/install/ros2lifecycle_test_fixtures;/home/nuc/ros2_humble/install/lifecycle;/home/nuc/ros2_humble/install/rclcpp_lifecycle;/home/nuc/ros2_humble/install/logging_demo;/home/nuc/ros2_humble/install/image_tools;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_subscriber;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_composition;/home/nuc/ros2_humble/install/demo_nodes_cpp_native;/home/nuc/ros2_humble/install/rclcpp_components;/home/nuc/ros2_humble/install/intra_process_demo;/home/nuc/ros2_humble/install/examples_rclcpp_multithreaded_executor;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_timer;/home/nuc/ros2_humble/install/examples_rclcpp_minimal_publisher;/home/nuc/ros2_humble/install/examples_rclcpp_cbg_executor;/home/nuc/ros2_humble/install/dummy_sensors;/home/nuc/ros2_humble/install/dummy_map_server;/home/nuc/ros2_humble/install/rclcpp;/home/nuc/ros2_humble/install/rcl_lifecycle;/home/nuc/ros2_humble/install/libstatistics_collector;/home/nuc/ros2_humble/install/rcl;/home/nuc/ros2_humble/install/rmw_implementation;/home/nuc/ros2_humble/install/rmw_fastrtps_dynamic_cpp;/home/nuc/ros2_humble/install/rmw_fastrtps_cpp;/home/nuc/ros2_humble/install/rmw_fastrtps_shared_cpp;/home/nuc/ros2_humble/install/rmw_cyclonedds_cpp;/home/nuc/ros2_humble/install/tracetools;/home/nuc/ros2_humble/install/tlsf;/home/nuc/ros2_humble/install/tinyxml_vendor;/home/nuc/ros2_humble/install/qt_gui_core;/home/nuc/ros2_humble/install/qt_gui_cpp;/home/nuc/ros2_humble/install/pluginlib;/home/nuc/ros2_humble/install/tinyxml2_vendor;/home/nuc/ros2_humble/install/tf2_eigen_kdl;/home/nuc/ros2_humble/install/tf2;/home/nuc/ros2_humble/install/test_security;/home/nuc/ros2_humble/install/test_rmw_implementation;/home/nuc/ros2_humble/install/test_rclcpp;/home/nuc/ros2_humble/install/test_quality_of_service;/home/nuc/ros2_humble/install/test_launch_testing;/home/nuc/ros2_humble/install/test_interface_files;/home/nuc/ros2_humble/install/test_communication;/home/nuc/ros2_humble/install/test_cli_remapping;/home/nuc/ros2_humble/install/test_cli;/home/nuc/ros2_humble/install/qt_gui_app;/home/nuc/ros2_humble/install/qt_gui;/home/nuc/ros2_humble/install/tango_icons_vendor;/home/nuc/ros2_humble/install/stereo_msgs;/home/nuc/ros2_humble/install/std_srvs;/home/nuc/ros2_humble/install/shape_msgs;/home/nuc/ros2_humble/install/map_msgs;/home/nuc/ros2_humble/install/sensor_msgs;/home/nuc/ros2_humble/install/nav_msgs;/home/nuc/ros2_humble/install/diagnostic_msgs;/home/nuc/ros2_humble/install/geometry_msgs;/home/nuc/ros2_humble/install/actionlib_msgs;/home/nuc/ros2_humble/install/std_msgs;/home/nuc/ros2_humble/install/statistics_msgs;/home/nuc/ros2_humble/install/sqlite3_vendor;/home/nuc/ros2_humble/install/rcl_logging_spdlog;/home/nuc/ros2_humble/install/spdlog_vendor;/home/nuc/ros2_humble/install/shared_queues_vendor;/home/nuc/ros2_humble/install/rviz_rendering_tests;/home/nuc/ros2_humble/install/rviz_rendering;/home/nuc/ros2_humble/install/rviz_ogre_vendor;/home/nuc/ros2_humble/install/rviz_assimp_vendor;/home/nuc/ros2_humble/install/rttest;/home/nuc/ros2_humble/install/rmw_connextddsmicro;/home/nuc/ros2_humble/install/rmw_connextdds;/home/nuc/ros2_humble/install/rmw_connextdds_common;/home/nuc/ros2_humble/install/rti_connext_dds_cmake_module;/home/nuc/ros2_humble/install/rosgraph_msgs;/home/nuc/ros2_humble/install/rosbag2_interfaces;/home/nuc/ros2_humble/install/rmw_dds_common;/home/nuc/ros2_humble/install/composition_interfaces;/home/nuc/ros2_humble/install/rcl_interfaces;/home/nuc/ros2_humble/install/pendulum_msgs;/home/nuc/ros2_humble/install/lifecycle_msgs;/home/nuc/ros2_humble/install/builtin_interfaces;/home/nuc/ros2_humble/install/rosidl_default_runtime;/home/nuc/ros2_humble/install/rosidl_default_generators;/home/nuc/ros2_humble/install/rosidl_generator_py;/home/nuc/ros2_humble/install/rosidl_typesupport_introspection_tests;/home/nuc/ros2_humble/install/rosidl_typesupport_cpp;/home/nuc/ros2_humble/install/rosidl_typesupport_introspection_cpp;/home/nuc/ros2_humble/install/rosidl_typesupport_c;/home/nuc/ros2_humble/install/rosidl_typesupport_introspection_c;/home/nuc/ros2_humble/install/rosidl_typesupport_fastrtps_c;/home/nuc/ros2_humble/install/rosidl_typesupport_fastrtps_cpp;/home/nuc/ros2_humble/install/rosidl_generator_cpp;/home/nuc/ros2_humble/install/rosidl_runtime_cpp;/home/nuc/ros2_humble/install/rcl_yaml_param_parser;/home/nuc/ros2_humble/install/rmw;/home/nuc/ros2_humble/install/rosidl_runtime_c;/home/nuc/ros2_humble/install/rosidl_generator_c;/home/nuc/ros2_humble/install/rosidl_typesupport_interface;/home/nuc/ros2_humble/install/rosidl_generator_dds_idl;/home/nuc/ros2_humble/install/rosidl_cmake;/home/nuc/ros2_humble/install/rosidl_parser;/home/nuc/ros2_humble/install/rosidl_adapter;/home/nuc/ros2_humble/install/rosbag2_tests;/home/nuc/ros2_humble/install/ros_environment;/home/nuc/ros2_humble/install/rmw_implementation_cmake;/home/nuc/ros2_humble/install/resource_retriever;/home/nuc/ros2_humble/install/class_loader;/home/nuc/ros2_humble/install/rcpputils;/home/nuc/ros2_humble/install/rcl_logging_noop;/home/nuc/ros2_humble/install/rcl_logging_interface;/home/nuc/ros2_humble/install/rcutils;/home/nuc/ros2_humble/install/qt_gui_py_common;/home/nuc/ros2_humble/install/qt_dotgraph;/home/nuc/ros2_humble/install/python_qt_binding;/home/nuc/ros2_humble/install/python_orocos_kdl_vendor;/home/nuc/ros2_humble/install/launch_testing_ament_cmake;/home/nuc/ros2_humble/install/python_cmake_module;/home/nuc/ros2_humble/install/pybind11_vendor;/home/nuc/ros2_humble/install/performance_test_fixture;/home/nuc/ros2_humble/install/osrf_testing_tools_cpp;/home/nuc/ros2_humble/install/orocos_kdl_vendor;/home/nuc/ros2_humble/install/mimick_vendor;/home/nuc/ros2_humble/install/libyaml_vendor;/home/nuc/ros2_humble/install/libcurl_vendor;/home/nuc/ros2_humble/install/keyboard_handler;/home/nuc/ros2_humble/install/ignition_math6_vendor;/home/nuc/ros2_humble/install/ignition_cmake2_vendor;/home/nuc/ros2_humble/install/iceoryx_introspection;/home/nuc/ros2_humble/install/cyclonedds;/home/nuc/ros2_humble/install/iceoryx_posh;/home/nuc/ros2_humble/install/iceoryx_hoofs;/home/nuc/ros2_humble/install/iceoryx_binding_c;/home/nuc/ros2_humble/install/ament_cmake_ros;/home/nuc/ros2_humble/install/ament_cmake_auto;/home/nuc/ros2_humble/install/ament_cmake_gmock;/home/nuc/ros2_humble/install/gmock_vendor;/home/nuc/ros2_humble/install/ament_cmake_gtest;/home/nuc/ros2_humble/install/gtest_vendor;/home/nuc/ros2_humble/install/ament_cmake_google_benchmark;/home/nuc/ros2_humble/install/google_benchmark_vendor;/home/nuc/ros2_humble/install/fastrtps;/home/nuc/ros2_humble/install/foonathan_memory_vendor;/home/nuc/ros2_humble/install/fastrtps_cmake_module;/home/nuc/ros2_humble/install/fastcdr;/home/nuc/ros2_humble/install/eigen3_cmake_module;/home/nuc/ros2_humble/install/console_bridge_vendor;/home/nuc/ros2_humble/install/ament_cmake_xmllint;/home/nuc/ros2_humble/install/ament_cmake_pyflakes;/home/nuc/ros2_humble/install/ament_cmake_pycodestyle;/home/nuc/ros2_humble/install/ament_cmake_pep257;/home/nuc/ros2_humble/install/ament_cmake_pclint;/home/nuc/ros2_humble/install/ament_lint_auto;/home/nuc/ros2_humble/install/ament_cmake;/home/nuc/ros2_humble/install/ament_cmake_version;/home/nuc/ros2_humble/install/ament_cmake_vendor_package;/home/nuc/ros2_humble/install/ament_cmake_pytest;/home/nuc/ros2_humble/install/ament_cmake_nose;/home/nuc/ros2_humble/install/ament_cmake_mypy;/home/nuc/ros2_humble/install/ament_cmake_lint_cmake;/home/nuc/ros2_humble/install/ament_cmake_flake8;/home/nuc/ros2_humble/install/ament_cmake_cpplint;/home/nuc/ros2_humble/install/ament_cmake_cppcheck;/home/nuc/ros2_humble/install/ament_cmake_copyright;/home/nuc/ros2_humble/install/ament_cmake_clang_tidy;/home/nuc/ros2_humble/install/ament_cmake_clang_format;/home/nuc/ros2_humble/install/ament_cmake_test;/home/nuc/ros2_humble/install/ament_cmake_target_dependencies;/home/nuc/ros2_humble/install/ament_cmake_python;/home/nuc/ros2_humble/install/ament_cmake_export_dependencies;/home/nuc/ros2_humble/install/ament_cmake_libraries;/home/nuc/ros2_humble/install/ament_cmake_include_directories;/home/nuc/ros2_humble/install/ament_cmake_gen_version_h;/home/nuc/ros2_humble/install/ament_cmake_export_targets;/home/nuc/ros2_humble/install/ament_cmake_export_link_flags;/home/nuc/ros2_humble/install/ament_cmake_export_interfaces;/home/nuc/ros2_humble/install/ament_cmake_export_libraries;/home/nuc/ros2_humble/install/ament_cmake_export_include_directories;/home/nuc/ros2_humble/install/ament_cmake_export_definitions;/home/nuc/ros2_humble/install/ament_cmake_core;/home/nuc/ros2_humble/install/ament_index_cpp'.split(';')
        else:
            # don't consider any other prefix path than this one
            CMAKE_PREFIX_PATH = []
        # prepend current workspace if not already part of CPP
        base_path = os.path.dirname(__file__)
        # CMAKE_PREFIX_PATH uses forward slash on all platforms, but __file__ is platform dependent
        # base_path on Windows contains backward slashes, need to be converted to forward slashes before comparison
        if os.path.sep != '/':
            base_path = base_path.replace(os.path.sep, '/')

        if base_path not in CMAKE_PREFIX_PATH:
            CMAKE_PREFIX_PATH.insert(0, base_path)
        CMAKE_PREFIX_PATH = os.pathsep.join(CMAKE_PREFIX_PATH)

        environ = dict(os.environ)
        lines = []
        if not args.extend:
            lines += rollback_env_variables(environ, ENV_VAR_SUBFOLDERS)
        lines += prepend_env_variables(environ, ENV_VAR_SUBFOLDERS, CMAKE_PREFIX_PATH)
        lines += find_env_hooks(environ, CMAKE_PREFIX_PATH)
        print('\n'.join(lines))

        # need to explicitly flush the output
        sys.stdout.flush()
    except IOError as e:
        # and catch potential "broken pipe" if stdout is not writable
        # which can happen when piping the output to a file but the disk is full
        if e.errno == errno.EPIPE:
            print(e, file=sys.stderr)
            sys.exit(2)
        raise

    sys.exit(0)
