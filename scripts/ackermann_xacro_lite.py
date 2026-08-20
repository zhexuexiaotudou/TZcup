#!/usr/bin/env python3
"""Minimal Xacro expansion harness for the sanitation vehicle description.

The real ``xacro`` executable ships with ROS 2 and is not available on every
CI/Windows host, so this module expands the *single* maintained Xacro file
``sanitation_vehicle.urdf.xacro`` using only the Xacro constructs it actually
uses (arg, property, macro, if, include).  The two upstream Linorobot2
includes (``imu`` and ``diff_drive_controller``) are replaced by documented
stubs that mirror the produced URDF/Gazebo fragments, which is sufficient for
topology/plugin-count/limit assertions.  Full expansion with the real
Linorobot2 files is a build-time concern on the ROS side.
"""

from __future__ import annotations

import ast
import math
import re
import xml.etree.ElementTree as ET


XACRO_NS = "http://ros.org/wiki/xacro"
XACRO = f"{{{XACRO_NS}}}"


def _is_xacro(element: ET.Element) -> bool:
    return element.tag.startswith(XACRO)


def _strip_ns(tag: str) -> str:
    return tag.split("}", 1)[1] if tag.startswith("{") else tag


class ExpressionEvaluator:
    """Evaluate the subset of Python expressions used by the Xacro file."""

    def __init__(self, namespace: dict):
        self.namespace = dict(namespace)
        self.namespace.setdefault("pi", math.pi)
        self.namespace.setdefault("true", True)
        self.namespace.setdefault("false", False)

    def evaluate(self, expression: str):
        source = expression.strip()
        if not source:
            return ""
        try:
            tree = ast.parse(source, mode="eval")
        except SyntaxError as exc:
            raise ValueError(f"invalid xacro expression: {source!r}: {exc}") from exc
        return eval(  # noqa: S307 - controlled local harness, no untrusted input
            compile(tree, "<xacro-lite>", "eval"),
            {"__builtins__": {}, "math": math},
            self.namespace,
        )


def _quote_value(value: str) -> str:
    lowered = value.strip().lower()
    if lowered in ("true", "false"):
        return lowered
    return f"'{value}'"


class XacroLite:
    """Expand the sanitation vehicle Xacro for a given drive model."""

    def __init__(
        self,
        source: str,
        overrides: dict[str, str] | None = None,
        stubs: dict[str, tuple[str, str]] | None = None,
    ):
        self.source = source
        self.overrides = dict(overrides or {})
        self.args: dict[str, str] = {}
        self.properties: dict[str, object] = {}
        self.macros: dict[str, tuple[str, str]] = dict(stubs or {})
        self.evaluator = ExpressionEvaluator({})
        self.root = ET.fromstring(source)

    # -- substitution helpers ------------------------------------------------

    def _substitute_args(self, text: str) -> str:
        pattern = re.compile(r"\$\(arg\s+([a-zA-Z0-9_]+)\)")

        def replace(match: re.Match) -> str:
            name = match.group(1)
            return _quote_value(self.args[name])

        return pattern.sub(replace, text)

    def _substitute_args_raw(self, text: str) -> str:
        pattern = re.compile(r"\$\(arg\s+([a-zA-Z0-9_]+)\)")

        def replace(match: re.Match) -> str:
            return str(self.args[match.group(1)])

        return pattern.sub(replace, text)

    def _evaluate_text(self, text: str) -> str:
        text = self._substitute_args_raw(text)
        if "${" not in text:
            return text
        out = []
        position = 0
        while True:
            start = text.find("${", position)
            if start < 0:
                out.append(text[position:])
                break
            out.append(text[position:start])
            end = text.find("}", start + 2)
            if end < 0:
                raise ValueError(f"unterminated ${{...}} in: {text!r}")
            expression = text[start + 2:end]
            out.append(str(self.evaluator.evaluate(expression)))
            position = end + 1
        return "".join(out)

    def _evaluate_condition(self, text: str) -> str:
        return self._evaluate_text(text)

    def _expand_macro(
        self, name: str, arguments: dict[str, str]
    ) -> list[ET.Element]:
        if name not in self.macros:
            raise ValueError(f"undefined xacro macro: {name}")
        params, body = self.macros[name]
        namespace = dict(self.properties)
        for param in params.split():
            if param not in arguments:
                raise ValueError(f"missing xacro parameter {param!r} for {name}")
            value = arguments[param]
            namespace[param] = self._parse_scalar(value)
        body_text = body
        for param in params.split():
            value = arguments[param]
            body_text = body_text.replace(f"$({param})", value).replace(
                f"${{{param}}}", value
            )
        nested = ET.fromstring(f"<root xmlns:xacro=\"{XACRO_NS}\">{body_text}</root>")
        self.evaluator.namespace.update(namespace)
        expanded = []
        for child in list(nested):
            expanded.extend(self._process(child))
        return expanded

    @staticmethod
    def _parse_scalar(value: str):
        if value.strip().lower() == "true":
            return True
        if value.strip().lower() == "false":
            return False
        try:
            return int(value)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value

    # -- recursive processor -------------------------------------------------

    def _process(self, element: ET.Element) -> list[ET.Element]:
        tag = element.tag
        if not _is_xacro(element):
            for attribute, value in list(element.attrib.items()):
                element.set(attribute, self._evaluate_text(value))
            replacement = []
            for child in list(element):
                replacement.extend(self._process(child))
            for child in list(element):
                element.remove(child)
            for new_child in replacement:
                element.append(new_child)
            if element.text:
                element.text = self._evaluate_text(element.text)
            return [element]

        name = _strip_ns(tag)
        if name == "include":
            # Upstream includes are provided as stubs in the test harness.
            return []
        if name == "arg":
            argument = element.attrib["name"]
            self.args[argument] = str(
                self.overrides.get(argument, element.attrib.get("default", ""))
            )
            return []
        if name == "property":
            property_name = element.attrib["name"]
            raw_value = self._evaluate_text(
                self._substitute_args_raw(element.attrib["value"])
            )
            self.properties[property_name] = self._parse_scalar(raw_value)
            self.evaluator.namespace.update(self.properties)
            return []
        if name == "macro":
            self.macros[element.attrib["name"]] = (
                element.attrib.get("params", ""),
                "".join(
                    ET.tostring(child, encoding="unicode")
                    for child in element
                ),
            )
            return []
        if name == "if":
            condition = self._evaluate_condition(element.attrib["value"])
            if not bool(self.evaluator.evaluate(condition)):
                return []
            return self._process_children(element)
        if name in self.macros:
            return self._expand_macro(name, dict(element.attrib))
        raise ValueError(f"unsupported xacro element: {name}")

    def _process_children(self, element: ET.Element) -> list[ET.Element]:
        expanded: list[ET.Element] = []
        for child in list(element):
            expanded.extend(self._process(child))
        return expanded

    # -- public API ----------------------------------------------------------

    def expand(self) -> ET.Element:
        # First pass: register args, properties and macros in document order.
        for element in self.root.iter():
            if _is_xacro(element):
                tag = _strip_ns(element.tag)
                if tag == "arg":
                    argument = element.attrib["name"]
                    self.args[argument] = str(
                        self.overrides.get(
                            argument, element.attrib.get("default", "")
                        )
                    )
                elif tag == "property":
                    property_name = element.attrib["name"]
                    raw_value = self._substitute_args_raw(element.attrib["value"])
                    self.properties[property_name] = self._parse_scalar(
                        self._evaluate_text(raw_value)
                    )
                    self.evaluator.namespace.update(self.properties)
                elif tag == "macro":
                    self.macros[element.attrib["name"]] = (
                        element.attrib.get("params", ""),
                        "".join(
                            ET.tostring(child, encoding="unicode")
                            for child in element
                        ),
                    )
        self.evaluator.namespace.update(self.properties)
        self.evaluator.namespace.update(self.args)
        output = ET.Element("robot")
        for child in list(self.root):
            output.extend(self._process(child))
        return output


def vehicle_expander(
    drive_model: str,
    overrides: dict[str, str] | None = None,
) -> XacroLite:
    """Build the configured lightweight expander for a vehicle profile."""
    if drive_model not in ("ackermann", "skid_steer_legacy"):
        raise ValueError(
            f"drive_model must be 'ackermann' or 'skid_steer_legacy', got "
            f"{drive_model!r}"
        )
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1]
        / "starter_ws"
        / "src"
        / "sanitation_vehicle_description"
        / "urdf"
        / "sanitation_vehicle.urdf.xacro"
    ).read_text(encoding="utf-8")
    stubs = {
        "imu": (
            "",
            """
            <link name="imu_link"/>
            <gazebo reference="imu_link">
              <sensor name="imu_sensor" type="imu">
                <topic>imu/data</topic>
                <update_rate>100</update_rate>
                <always_on>true</always_on>
                <visualize>false</visualize>
              </sensor>
            </gazebo>
            """,
        ),
        "diff_drive_controller": (
            "wheel_separation wheel_radius",
            """
            <gazebo>
              <plugin filename="gz-sim-diff-drive-system"
                      name="gz::sim::systems::DiffDrive">
                <left_joint>front_left_wheel_joint</left_joint>
                <right_joint>front_right_wheel_joint</right_joint>
                <wheel_separation>${wheel_separation}</wheel_separation>
                <wheel_radius>${wheel_radius}</wheel_radius>
                <odom_topic>/odom/unfiltered</odom_topic>
                <odom_frame>odom</odom_frame>
                <robot_base_frame>base_footprint</robot_base_frame>
                <odom_publish_frequency>50</odom_publish_frequency>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>false</publish_odom_tf>
              </plugin>
            </gazebo>
            """,
        ),
    }
    all_overrides = dict(
        {
            "drive_model": drive_model,
            "enable_wheel_slip": "false",
            "enable_training_gt": "false",
            "enable_verification_camera": "false",
            "enable_self_mask_gt": "false",
            "enable_manipulator": "false",
            "brush_forward_x": "0.68" if drive_model == "ackermann" else "0.58",
        },
        **(overrides or {}),
    )
    return XacroLite(source, overrides=all_overrides, stubs=stubs)


def expand_vehicle(
    drive_model: str,
    overrides: dict[str, str] | None = None,
) -> ET.Element:
    """Expand ``sanitation_vehicle.urdf.xacro`` for a drive model."""
    return vehicle_expander(drive_model, overrides).expand()


def expand_vehicle_xml(drive_model: str) -> str:
    root = expand_vehicle(drive_model)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")
