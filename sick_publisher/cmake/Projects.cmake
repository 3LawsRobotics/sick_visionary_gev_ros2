# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

unset(projects)
set(third_party_dir ${sick_visionary_gev_ros2_SOURCE_DIR}/third_party)

# GenIStream wrapper
list(APPEND projects genistream)
set(genistream_version "3.7.0")
set(genistream_path ${third_party_dir}/genistream/${genistream_version})

# yaml-cpp
list(APPEND projects yaml-cpp)
set(yamlcpp_version "0.8.0")
set(yamlcpp_path ${third_party_dir}/yaml-cpp/${yamlcpp_version}/yaml-cpp-${yamlcpp_version}.zip)
set(yamlcpp_md5 "6eff843c5a75afc88f979eb42c2ecd70")