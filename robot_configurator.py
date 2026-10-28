#!/usr/bin/env python3
"""
Robot Configurator — validates and computes derived values for servo_config.json.

Вычисляет производные величины (FK, лимиты, инверсии) и записывает их
в секцию _computed. Запускать после каждого изменения конфигурации.

Computes derived values (FK, limits, inversions) and writes them into
the _computed sections. Run after every config change.

Usage:
    python3 robot_configurator.py                     # validate & compute
    python3 robot_configurator.py --save              # validate, compute, save
    python3 robot_configurator.py --save --file my.json
"""

import json
import math
import sys
import copy
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).parent / "servo_config.json"

# Keys that are documentation (start with _) — skip in validation
DOC_PREFIX = "_"


def is_data_key(key: str) -> bool:
    """True if key is actual data (not documentation)."""
    return not key.startswith(DOC_PREFIX)


# ──────────────────────────────────────────────────────────────────
#  FK: Forward Kinematics
# ──────────────────────────────────────────────────────────────────

def fk_standing_height(hip_length: float, thigh_length: float,
                       calf_length: float, calf_angle_rad: float) -> dict:
    """
    Вычисляет высоту стояния через прямую кинематику.
    Computes standing height via forward kinematics.
    """
    cos_term = math.cos(math.pi - calf_angle_rad)
    discriminant = thigh_length**2 + calf_length**2 - 2*thigh_length*calf_length*cos_term
    if discriminant < 0:
        raise ValueError(
            f"FK error: negative discriminant ({discriminant:.6f}). "
            f"Impossible geometry: thigh={thigh_length}, calf={calf_length}, "
            f"calf_angle={calf_angle_rad}"
        )
    target_dist = math.sqrt(discriminant)
    standing_height = target_dist + hip_length
    return {
        "target_dist_m": round(target_dist, 6),
        "standing_height_m": round(standing_height, 6),
    }


def compute_standing_section(body: dict) -> dict:
    """
    Заполняет body.standing._computed.
    Populates body.standing._computed from link_lengths and nominal angles.
    """
    standing = body["standing"]
    links = body["link_lengths_m"]

    hip, thigh, calf = links["hip"], links["thigh"], links["calf"]
    calf_angle = standing["nominal_calf_angle_rad"]

    fk = fk_standing_height(hip, thigh, calf, calf_angle)
    h = fk["standing_height_m"]
    min_ratio = standing.get("height_min_ratio", 0.7)
    max_ratio = standing.get("height_max_ratio", 1.3)

    return {
        "_desc_ru": "АВТОМАТИЧЕСКИ РАСЧИТЫВАЕМЫЕ ЗНАЧЕНИЯ. НЕ РЕДАКТИРУЙТЕ ВРУЧНУЮ.",
        "_desc_en": "AUTO-COMPUTED VALUES. DO NOT EDIT MANUALLY.",
        "standing_height_m": fk["standing_height_m"],
        "min_height_m": round(h * min_ratio, 6),
        "max_height_m": round(h * max_ratio, 6),
        "target_dist_m": fk["target_dist_m"],
        "max_reach_m": round(thigh + calf, 6),
    }


# ──────────────────────────────────────────────────────────────────
#  Validation
# ──────────────────────────────────────────────────────────────────

def validate_body(cfg: dict) -> list[str]:
    """Проверяет секцию body / Validates body section."""
    warnings = []
    body = cfg["body"]
    links = body["link_lengths_m"]
    standing = body["standing"]
    legs = cfg["legs"]

    hip, thigh, calf = links["hip"], links["thigh"], links["calf"]
    max_reach = thigh + calf

    # FK reachability
    fk = fk_standing_height(hip, thigh, calf, standing["nominal_calf_angle_rad"])
    if fk["target_dist_m"] > max_reach:
        warnings.append(
            f"CRITICAL: target_dist ({fk['target_dist_m']:.4f}m) > max_reach "
            f"({max_reach:.4f}m). Robot CANNOT stand at this calf angle!"
        )

    # Leg positions vs body dimensions
    half_len = body["length_m"] / 2.0
    half_wid = body["width_m"] / 2.0
    for leg_name, leg in legs.items():
        if leg_name.startswith(DOC_PREFIX):
            continue
        dx = abs(abs(leg["hip_x_m"]) - half_len)
        dy = abs(abs(leg["hip_y_m"]) - half_wid)
        if dx > 0.001:
            warnings.append(f"Leg {leg_name}: hip_x ({leg['hip_x_m']}) != ±body_length/2 ({half_len})")
        if dy > 0.001:
            warnings.append(f"Leg {leg_name}: hip_y ({leg['hip_y_m']}) != ±body_width/2 ({half_wid})")

    # Inverted flags match leg mirrored
    joint_types = cfg["joints"]
    joint_list = cfg["joints"]["list"]
    leg_map = {name: leg["mirrored"] for name, leg in legs.items() if is_data_key(name)}
    for j in joint_list:
        if j["leg"] not in leg_map:
            warnings.append(f"Joint {j['name']}: unknown leg '{j['leg']}'")
            continue
        expected = leg_map[j["leg"]]
        if j["inverted"] != expected:
            warnings.append(
                f"Joint {j['name']}: inverted={j['inverted']} but leg {j['leg']} mirrored={expected}"
            )

    return warnings


def validate_joints(cfg: dict) -> list[str]:
    """Проверяет секцию joints / Validates joints section."""
    warnings = []
    joint_list = cfg["joints"]["list"]
    joint_types = cfg["joints"]

    # Servo IDs unique and sequential 0..11
    ids = [j["servo_id"] for j in joint_list]
    if ids != list(range(12)):
        warnings.append(f"Servo IDs not sequential 0-11: {ids}")

    # Names follow convention
    expected_names = []
    for leg in ["lf", "rf", "lr", "rr"]:
        for jtype in ["hip", "thigh", "calf"]:
            expected_names.append(f"{leg}_{jtype}_joint")
    actual_names = [j["name"] for j in joint_list]
    if actual_names != expected_names:
        warnings.append(f"Joint names don't match convention")

    # Each joint inherits correct limits from type template
    for j in joint_list:
        tmpl = joint_types.get(j["type"])
        if not tmpl:
            warnings.append(f"Joint {j['name']}: unknown type '{j['type']}'")
            continue
        for field in ["min_rad", "max_rad", "neutral_rad"]:
            if field in tmpl and field not in j:
                # OK — will inherit
                pass

    # Calf neutral matches standing nominal
    calf_neutrals = set()
    for j in joint_list:
        if j["type"] == "calf":
            neutral = joint_types["calf"].get("neutral_rad", j.get("neutral_rad"))
            calf_neutrals.add(neutral)
    nominal_calf = cfg["body"]["standing"]["nominal_calf_angle_rad"]
    calf_neutral = joint_types["calf"]["neutral_rad"]
    if abs(calf_neutral - nominal_calf) > 0.001:
        warnings.append(
            f"Calf neutral_rad ({calf_neutral}) != standing.nominal_calf_angle_rad ({nominal_calf})"
        )

    return warnings


def validate_gait(cfg: dict) -> list[str]:
    """Проверяет секцию gait / Validates gait section."""
    warnings = []
    gait = cfg["gait"]

    for gait_name, gait_cfg in gait.get("gaits", {}).items():
        if not is_data_key(gait_name):
            continue
        phases = gait_cfg.get("phase_offsets")
        if not phases:
            continue
        if len(phases) != 4:
            warnings.append(f"Gait '{gait_name}': phase_offsets must have 4 elements, got {len(phases)}")
            continue

        if gait_name == "trot":
            if phases[0] != phases[3]:
                warnings.append(f"Trot: LF(idx0)={phases[0]} != RR(idx3)={phases[3]}")
            if phases[1] != phases[2]:
                warnings.append(f"Trot: RF(idx1)={phases[1]} != LR(idx2)={phases[2]}")
            if phases[0] == phases[1]:
                warnings.append(f"Trot: diagonal pairs must differ")

    return warnings


def validate_all(cfg: dict) -> list[str]:
    """Запускает все проверки / Runs all validations."""
    warnings = []
    warnings.extend(validate_body(cfg))
    warnings.extend(validate_joints(cfg))
    warnings.extend(validate_gait(cfg))
    return warnings


# ──────────────────────────────────────────────────────────────────
#  Compute & Save
# ──────────────────────────────────────────────────────────────────

def compute_all(cfg: dict) -> dict:
    """
    Вычисляет все _computed секции. Возвращает изменённую копию.
    Computes all _computed sections. Returns a modified copy.
    """
    result = copy.deepcopy(cfg)

    # 1. Standing height from FK
    result["body"]["standing"]["_computed"] = compute_standing_section(result["body"])

    # 2. Code generation helpers (for C++ replacement)
    joint_list = result["joints"]["list"]
    inverted = [j["inverted"] for j in joint_list]
    nominal_calf = result["body"]["standing"]["nominal_calf_angle_rad"]
    joint_names = [j["name"] for j in joint_list]

    initial_angles = []
    for j in joint_list:
        if j["type"] == "calf":
            initial_angles.append(str(nominal_calf))
        elif j["type"] == "thigh":
            initial_angles.append(str(result["body"]["standing"]["nominal_thigh_angle_rad"]))
        else:
            initial_angles.append("0.0")

    result["_codegen"] = {
        "_desc_ru": "Автогенерированные значения для вставки в C++ код",
        "_desc_en": "Auto-generated values for direct use in C++ code",
        "inverted_array_cpp": "{" + ", ".join(str(v).lower() for v in inverted) + "}",
        "joint_names_cpp": '{"' + '", "'.join(joint_names) + '"}',
        "initial_angles_cpp": "{" + ", ".join(initial_angles) + "}",
        "joint_count": len(joint_list),
    }

    return result


def save(cfg: dict, path: Path) -> None:
    """Сохраняет конфигурацию / Saves configuration."""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
        f.write("\n")


# ──────────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────────

def main():
    args = sys.argv[1:]
    do_save = "--save" in args

    file_arg = None
    for i, a in enumerate(args):
        if a == "--file" and i + 1 < len(args):
            file_arg = args[i + 1]
    config_path = Path(file_arg) if file_arg else DEFAULT_CONFIG

    print(f"Config: {config_path}")

    with open(config_path, encoding="utf-8") as f:
        cfg = json.load(f)

    # Validate
    warnings = validate_all(cfg)
    if warnings:
        print(f"\n⚠  {len(warnings)} warning(s):")
        for w in warnings:
            print(f"   • {w}")
    else:
        print("\n✅ Validation passed")

    # Compute
    cfg = compute_all(cfg)

    standing = cfg["body"]["standing"]
    comp = standing["_computed"]
    links = cfg["body"]["link_lengths_m"]

    print(f"\n── FK Computed ──────────────────────────────────")
    print(f"  Link lengths:     hip={links['hip']}m  thigh={links['thigh']}m  calf={links['calf']}m")
    print(f"  Nominal calf:     {standing['nominal_calf_angle_rad']} rad "
          f"({math.degrees(standing['nominal_calf_angle_rad']):.1f}°)")
    print(f"  target_dist:      {comp['target_dist_m']}m")
    print(f"  standing_height:  {comp['standing_height_m']}m  ({comp['standing_height_m']*100:.2f} cm)")
    print(f"  height range:     {comp['min_height_m']}m – {comp['max_height_m']}m")
    print(f"  max_reach:        {comp['max_reach_m']}m")
    if comp['target_dist_m'] > comp['max_reach_m']:
        print(f"  ⚠  IK OVERSHOOT: target_dist > max_reach by "
              f"{(comp['target_dist_m'] - comp['max_reach_m'])*1000:.1f}mm")

    codegen = cfg.get("_codegen", {})
    print(f"\n── Code Generation ──────────────────────────────")
    print(f"  inverted[]:       {codegen['inverted_array_cpp']}")
    print(f"  initial_angles:   {codegen['initial_angles_cpp']}")
    print(f"  joint_names:      {codegen['joint_names_cpp']}")

    if do_save:
        save(cfg, config_path)
        print(f"\n💾 Saved to {config_path}")
    else:
        print(f"\n  (dry run — use --save to write)")

    print()
    return 0 if not warnings else 1


if __name__ == "__main__":
    sys.exit(main())
