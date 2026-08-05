#!/usr/bin/env python3

import re
import subprocess
import sys
from pathlib import Path


WORKSPACE = Path("/home/nvidia/patrol_ws")
TEMPLATE = (
    WORKSPACE
    / "tools/tests/handoff_experiments/"
      "test_tight_curve_bad_handoff.py"
)

OUTPUT_DIR = Path("/tmp/patrol_handoff_matrix")
SUMMARY_FILE = Path(
    "/tmp/patrol_handoff_matrix_summary.txt"
)

CASES = [
    (0.10, 5.0),
    (0.10, 8.0),
    (0.15, 8.0),
    (0.15, 10.0),
    (0.20, 8.0),
    (0.20, 10.0),
    (0.25, 12.0),
    (0.35, 18.3),
]

RESULT_KEYS = [
    "MISSION_SUCCEEDED",
    "MISSION_FAILED",
    "HANDOFF_ERROR_INJECTED",
    "MAX_ABS_ROUTE_STEERING",
    "ROUTE_SATURATION_STEPS",
    "ROUTE_SATURATION_TIME",
    "MAX_ROUTE_PATH_ERROR",
    "ROUTE_END_ERROR",
    "ROUTE_HEADING_ERROR_DEG",
    "TEST_RESULT",
    "STABILITY_RESULT",
]


def case_name(lateral_error, heading_error):
    lateral_code = int(round(lateral_error * 100))
    heading_code = int(round(heading_error))

    return (
        f"handoff_lateral_{lateral_code:02d}cm_"
        f"heading_{heading_code:02d}deg"
    )


def replace_once(text, pattern, replacement, label):
    updated, count = re.subn(
        pattern,
        replacement,
        text,
        count=1,
    )

    if count != 1:
        raise RuntimeError(
            f"修改 {label} 失败，匹配数量={count}"
        )

    return updated


def make_case_script(
    template_text,
    lateral_error,
    heading_error,
):
    name = case_name(
        lateral_error,
        heading_error,
    )

    text = template_text

    text = text.replace(
        "/tmp/test_tight_curve_bad_handoff.yaml",
        f"/tmp/{name}.yaml",
    )
    text = text.replace(
        "/tmp/test_tight_curve_bad_handoff_recorded.yaml",
        f"/tmp/{name}_recorded.yaml",
    )
    text = text.replace(
        "/tmp/test_tight_curve_bad_handoff_launch.log",
        f"/tmp/{name}_launch.log",
    )

    text = replace_once(
        text,
        r"north \+= [-+]?[0-9]*\.?[0-9]+",
        (
            "east = 0.0\n"
            "            "
            f"north = {lateral_error:.2f}"
        ),
        "绝对横向误差",
    )

    text = replace_once(
        text,
        (
            r"ros_yaw \+= "
            r"math\.radians\("
            r"[-+]?[0-9]*\.?[0-9]+"
            r"\)"
        ),
        (
            "ros_yaw = "
            f"math.radians({heading_error:.1f})"
        ),
        "绝对交接航向误差",
    )

    text = replace_once(
        text,
        r'"lateral=[-+]?[0-9]*\.?[0-9]+m"',
        f'"lateral={lateral_error:.2f}m"',
        "横向误差输出",
    )

    text = replace_once(
        text,
        r'"heading=[-+]?[0-9]*\.?[0-9]+deg"',
        f'"heading={heading_error:.1f}deg"',
        "航向误差输出",
    )

    return name, text


def extract_results(output):
    results = {}

    for key in RESULT_KEYS:
        match = re.search(
            rf"^{re.escape(key)}:\s*(.+)$",
            output,
            flags=re.MULTILINE,
        )

        results[key] = (
            match.group(1).strip()
            if match
            else "MISSING"
        )

    return results


def main():
    if not TEMPLATE.is_file():
        raise RuntimeError(
            f"模板脚本不存在：{TEMPLATE}"
        )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    template_text = TEMPLATE.read_text(
        encoding="utf-8"
    )

    all_results = []

    for index, (
        lateral_error,
        heading_error,
    ) in enumerate(CASES, start=1):
        name, script_text = make_case_script(
            template_text,
            lateral_error,
            heading_error,
        )

        script_path = OUTPUT_DIR / f"{name}.py"
        log_path = OUTPUT_DIR / f"{name}.log"

        script_path.write_text(
            script_text,
            encoding="utf-8",
        )

        print()
        print("=" * 72)
        print(
            f"CASE {index}/{len(CASES)}:",
            f"{lateral_error:.2f} m / "
            f"{heading_error:.1f} deg",
        )
        print("=" * 72)

        compile_result = subprocess.run(
            [
                sys.executable,
                "-m",
                "py_compile",
                str(script_path),
            ],
            cwd=WORKSPACE,
            text=True,
            capture_output=True,
        )

        if compile_result.returncode != 0:
            output = (
                compile_result.stdout
                + compile_result.stderr
            )
            log_path.write_text(
                output,
                encoding="utf-8",
            )

            results = {
                key: "COMPILE_FAILED"
                for key in RESULT_KEYS
            }
        else:
            process = subprocess.run(
                [
                    sys.executable,
                    str(script_path),
                ],
                cwd=WORKSPACE,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )

            output = process.stdout

            log_path.write_text(
                output,
                encoding="utf-8",
            )

            print(output)

            results = extract_results(output)
            results["RETURN_CODE"] = (
                str(process.returncode)
            )

        all_results.append((
            lateral_error,
            heading_error,
            name,
            results,
        ))

    lines = [
        "PATROL HANDOFF MATRIX SUMMARY",
        "=" * 100,
        (
            "lateral_m heading_deg max_steering "
            "saturation_s max_path_error "
            "mission stability"
        ),
    ]

    for (
        lateral_error,
        heading_error,
        name,
        results,
    ) in all_results:
        lines.append(
            f"{lateral_error:9.2f} "
            f"{heading_error:11.1f} "
            f"{results['MAX_ABS_ROUTE_STEERING']:>12} "
            f"{results['ROUTE_SATURATION_TIME']:>12} "
            f"{results['MAX_ROUTE_PATH_ERROR']:>14} "
            f"{results['TEST_RESULT']:>7} "
            f"{results['STABILITY_RESULT']:>9}"
        )

        lines.append(
            f"  log: {OUTPUT_DIR / (name + '.log')}"
        )

    SUMMARY_FILE.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    print()
    print("\n".join(lines))
    print()
    print("汇总文件：", SUMMARY_FILE)


if __name__ == "__main__":
    main()
