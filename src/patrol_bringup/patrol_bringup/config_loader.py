#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

import yaml


class PatrolConfigError(RuntimeError):
    """巡检系统配置错误。"""


REQUIRED_SECTIONS = (
    'system',
    'startup',
    'safety',
    'localization',
    'gnss_degraded',
    'entry_planner',
    'entry_executor',
    'route_follower',
    'auto_command_mux',
    'command_manager',
    'mission_manager',
    'map_frontend',
    'logging',
)


def _require_mapping(
    root: Mapping[str, Any],
    key: str,
) -> Mapping[str, Any]:
    value = root.get(key)

    if not isinstance(value, dict):
        raise PatrolConfigError(
            f'配置分区 {key!r} 必须是映射'
        )

    return value


def _number(
    section: Mapping[str, Any],
    key: str,
    *,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
    strictly_positive: bool = False,
) -> float:
    if key not in section:
        raise PatrolConfigError(
            f'配置缺少参数：{key}'
        )

    value = section[key]

    if isinstance(value, bool):
        raise PatrolConfigError(
            f'参数 {key} 不能是布尔值'
        )

    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise PatrolConfigError(
            f'参数 {key} 必须是数字：{value!r}'
        ) from exc

    if not math.isfinite(number):
        raise PatrolConfigError(
            f'参数 {key} 必须是有限数值'
        )

    if strictly_positive and number <= 0.0:
        raise PatrolConfigError(
            f'参数 {key} 必须大于0'
        )

    if minimum is not None and number < minimum:
        raise PatrolConfigError(
            f'参数 {key} 不能小于 {minimum}'
        )

    if maximum is not None and number > maximum:
        raise PatrolConfigError(
            f'参数 {key} 不能大于 {maximum}'
        )

    return number


def load_patrol_config(
    config_file: str,
) -> Dict[str, Any]:
    path = Path(config_file).expanduser().resolve()

    if not path.is_file():
        raise PatrolConfigError(
            f'统一配置文件不存在：{path}'
        )

    try:
        data = yaml.safe_load(
            path.read_text(encoding='utf-8')
        )
    except (OSError, yaml.YAMLError) as exc:
        raise PatrolConfigError(
            f'无法读取统一配置：{exc}'
        ) from exc

    if not isinstance(data, dict):
        raise PatrolConfigError(
            '统一配置根节点必须是映射'
        )

    if data.get('schema_version') != 1:
        raise PatrolConfigError(
            '仅支持 schema_version: 1'
        )

    for section_name in REQUIRED_SECTIONS:
        _require_mapping(data, section_name)

    startup = _require_mapping(data, 'startup')
    safety = _require_mapping(data, 'safety')
    degraded = _require_mapping(data, 'gnss_degraded')
    entry = _require_mapping(data, 'entry_executor')
    follower = _require_mapping(data, 'route_follower')
    command = _require_mapping(data, 'command_manager')

    _number(
        startup,
        'base_ready_timeout_sec',
        strictly_positive=True,
    )
    _number(
        startup,
        'runtime_ready_timeout_sec',
        strictly_positive=True,
    )
    _number(
        startup,
        'localization_ready_timeout_sec',
        strictly_positive=True,
    )
    _number(
        startup,
        'service_call_timeout_sec',
        strictly_positive=True,
    )
    _number(
        safety,
        'maximum_origin_distance_m',
        strictly_positive=True,
    )

    entry_forward = _number(
        entry,
        'forward_speed_rpm',
        strictly_positive=True,
    )
    entry_reverse = _number(
        entry,
        'reverse_speed_rpm',
        strictly_positive=True,
    )

    minimum_speed = _number(
        follower,
        'minimum_speed_rpm',
        strictly_positive=True,
    )
    maximum_speed = _number(
        follower,
        'maximum_speed_rpm',
        strictly_positive=True,
    )

    if minimum_speed > maximum_speed:
        raise PatrolConfigError(
            'route_follower.minimum_speed_rpm '
            '不能大于 maximum_speed_rpm'
        )

    weak_debounce = _number(
        degraded,
        'weak_debounce_sec',
        minimum=0.0,
    )
    maximum_hold = _number(
        degraded,
        'maximum_hold_sec',
        strictly_positive=True,
        maximum=10.0,
    )
    hold_speed = _number(
        degraded,
        'hold_speed_rpm',
        strictly_positive=True,
    )

    if weak_debounce >= maximum_hold:
        raise PatrolConfigError(
            'weak_debounce_sec 必须小于 '
            'maximum_hold_sec'
        )

    if hold_speed > maximum_speed:
        raise PatrolConfigError(
            '弱GNSS保持速度不能高于正常最大巡迹速度'
        )

    command_maximum_speed = _number(
        command,
        'maximum_speed_rpm',
        strictly_positive=True,
    )

    required_command_speed = max(
        entry_forward,
        entry_reverse,
        maximum_speed,
        hold_speed,
    )

    if command_maximum_speed < required_command_speed:
        raise PatrolConfigError(
            'command_manager.maximum_speed_rpm '
            '低于其他模块所需速度'
        )

    return dict(data)


def section_parameters(
    config: Mapping[str, Any],
    section_name: str,
    *,
    rename: Optional[Mapping[str, str]] = None,
    exclude: Iterable[str] = (),
) -> Dict[str, Any]:
    section = _require_mapping(config, section_name)
    rename_map = dict(rename or {})
    excluded = set(exclude)

    result: Dict[str, Any] = {}

    for source_name, value in section.items():
        if source_name in excluded:
            continue

        if isinstance(value, (dict, list, tuple)):
            raise PatrolConfigError(
                f'{section_name}.{source_name} '
                '不能直接作为ROS参数'
            )

        target_name = rename_map.get(
            source_name,
            source_name,
        )
        result[target_name] = value

    return result
