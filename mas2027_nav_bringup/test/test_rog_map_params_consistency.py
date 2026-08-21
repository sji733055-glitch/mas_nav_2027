# Copyright 2026 mas2027
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Check that the two ROG-Map parameter blocks stay identical.

同一套 ROG-Map 参数在本包里存在两份，这是链路结构决定的，不是冗余：

* ``config/rog_map_params.yaml`` —— 给独立 ``rog_map_node`` 用
  （``use_rog_map_standalone:=True``，默认不启动，只在单独调试建图时开）。
  结构是 ``rog_map: ros__parameters: rog_map: {...}``。
* ``config/nav2_params.yaml`` 的
  ``planner_server: ros__parameters: MincoPlanner: rog_map: {...}`` —— 跑导航时
  ROG-Map 由 ``planner_server`` 进程内的 ``MincoPlanner`` 插件持有，读的是这一份。

两者无法合并成一份文件：前者是节点级参数文件，后者必须嵌在 ``planner_server``
节点下、``MincoPlanner`` 插件名之后，ROS 2 的参数文件格式没有跨节点引用/包含的
写法，``RewrittenYaml`` 也只能改标量、不能拼接嵌套段。

于是真正的风险不是"有两份"，而是"改了一份忘了另一份"——独立节点调出来的地图
和导航里实际用的地图不一致，现场会当成传感器或标定问题去查。这个测试把该风险
变成编译期失败：``colcon test`` 时逐叶子比对，不一致就报出具体键名。

改参数的流程：改 ``nav2_params.yaml``（导航实际生效的那份），同步回
``rog_map_params.yaml``，然后 ``colcon test --packages-select mas2027_nav_bringup``。
"""

import os

import pytest
import yaml

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config"
)
_STANDALONE_FILE = os.path.join(_CONFIG_DIR, "rog_map_params.yaml")
_NAV2_FILE = os.path.join(_CONFIG_DIR, "nav2_params.yaml")


def _load(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _flatten(node, prefix=""):
    """Flatten nested dicts into {'a.b.c': value} so mismatches name the exact key."""
    flat = {}
    for key, value in node.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flat.update(_flatten(value, name))
        else:
            flat[name] = value
    return flat


def _standalone_block():
    # 独立节点：节点名 rog_map，参数前缀同样是 rog_map
    # （宿主通过 Config::loadFromRosNode(node, "rog_map") 读取）。
    return _load(_STANDALONE_FILE)["rog_map"]["ros__parameters"]["rog_map"]


def _nav2_block():
    # 插件内实例：前缀是 "<插件名>.rog_map"，即 MincoPlanner.rog_map。
    return _load(_NAV2_FILE)["planner_server"]["ros__parameters"]["MincoPlanner"][
        "rog_map"
    ]


def test_rog_map_blocks_are_identical():
    """两份 ROG-Map 参数必须逐叶子相同，否则独立节点与导航行为会分叉."""
    standalone = _flatten(_standalone_block())
    nav2 = _flatten(_nav2_block())

    only_standalone = sorted(set(standalone) - set(nav2))
    only_nav2 = sorted(set(nav2) - set(standalone))
    mismatched = sorted(
        f"{key}: rog_map_params.yaml={standalone[key]!r} nav2_params.yaml={nav2[key]!r}"
        for key in set(standalone) & set(nav2)
        if standalone[key] != nav2[key]
    )

    problems = []
    if only_standalone:
        problems.append("只在 rog_map_params.yaml 里有: " + ", ".join(only_standalone))
    if only_nav2:
        problems.append("只在 nav2_params.yaml 里有: " + ", ".join(only_nav2))
    if mismatched:
        problems.append("取值不一致:\n  " + "\n  ".join(mismatched))

    assert not problems, (
        "ROG-Map 两份参数已经漂移。导航实际用的是 nav2_params.yaml 的 "
        "planner_server.ros__parameters.MincoPlanner.rog_map；"
        "请以它为准同步 rog_map_params.yaml。\n" + "\n".join(problems)
    )


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        # 这几项是按本仓库链路改过上游取值的，改回去会静默改变行为，单独钉住。
        ("frame_id", "odom"),
        ("visualization.frame_id", "odom"),
        ("ros_callback.odom_topic", "/Odometry"),
        ("ros_callback.cloud_topic", "/cloud_registered"),
    ],
)
def test_repo_specific_overrides_kept(key, expected):
    """本仓库相对上游 navi_minco_bit 改过的关键项不能被整段回滚覆盖掉."""
    nav2 = _flatten(_nav2_block())
    assert nav2.get(key) == expected, (
        f"{key} 期望 {expected!r}，实际 {nav2.get(key)!r}。"
        "上游 navi_minco_bit 用的是 camera_init / /aft_mapped_to_init / "
        "/cloud_registered_full，本仓库的 odom 与点云由 small_point_lio 在 odom 系发布。"
    )
