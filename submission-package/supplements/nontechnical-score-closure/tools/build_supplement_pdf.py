#!/usr/bin/env python3
"""Build the standalone nontechnical score-closure supplement PDF."""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


FONT = "STSong-Light"
TITLE_FONT = "STSong-Light"


def esc(value: object) -> str:
    return html.escape(str(value), quote=False)


def make_styles() -> dict[str, ParagraphStyle]:
    styles = getSampleStyleSheet()
    return {
        "title": ParagraphStyle(
            "CnTitle",
            parent=styles["Title"],
            fontName=TITLE_FONT,
            fontSize=24,
            leading=32,
            alignment=TA_CENTER,
            spaceAfter=12,
            textColor=colors.HexColor("#12263A"),
        ),
        "subtitle": ParagraphStyle(
            "CnSubtitle",
            parent=styles["Normal"],
            fontName=FONT,
            fontSize=11,
            leading=17,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#425466"),
        ),
        "h1": ParagraphStyle(
            "CnH1",
            parent=styles["Heading1"],
            fontName=FONT,
            fontSize=16,
            leading=22,
            textColor=colors.HexColor("#0B4F6C"),
            spaceBefore=10,
            spaceAfter=8,
        ),
        "h2": ParagraphStyle(
            "CnH2",
            parent=styles["Heading2"],
            fontName=FONT,
            fontSize=12,
            leading=17,
            textColor=colors.HexColor("#126782"),
            spaceBefore=7,
            spaceAfter=5,
        ),
        "body": ParagraphStyle(
            "CnBody",
            parent=styles["BodyText"],
            fontName=FONT,
            fontSize=9,
            leading=14,
            alignment=TA_LEFT,
            spaceAfter=5,
        ),
        "small": ParagraphStyle(
            "CnSmall",
            parent=styles["BodyText"],
            fontName=FONT,
            fontSize=7.5,
            leading=10.5,
            spaceAfter=3,
        ),
        "note": ParagraphStyle(
            "CnNote",
            parent=styles["BodyText"],
            fontName=FONT,
            fontSize=8.5,
            leading=13,
            leftIndent=4 * mm,
            rightIndent=4 * mm,
            borderColor=colors.HexColor("#E5A84B"),
            borderWidth=0.6,
            borderPadding=5,
            backColor=colors.HexColor("#FFF8E8"),
            spaceAfter=7,
        ),
    }


def P(text: object, style: ParagraphStyle) -> Paragraph:
    return Paragraph(esc(text), style)


def table(
    rows: list[list[object]],
    widths: list[float],
    styles: dict[str, ParagraphStyle],
    header: bool = True,
) -> Table:
    converted = [
        [P(cell, styles["small"]) for cell in row]
        for row in rows
    ]
    result = Table(converted, colWidths=widths, repeatRows=1 if header else 0)
    commands = [
        ("FONTNAME", (0, 0), (-1, -1), FONT),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#AAB7C4")),
        ("LEFTPADDING", (0, 0), (-1, -1), 3),
        ("RIGHTPADDING", (0, 0), (-1, -1), 3),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]
    if header:
        commands.extend(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#D9EAF2")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.HexColor("#102A43")),
            ]
        )
    result.setStyle(TableStyle(commands))
    return result


def footer(canvas, doc) -> None:
    canvas.saveState()
    canvas.setFont(FONT, 7.5)
    canvas.setFillColor(colors.HexColor("#6B7C8F"))
    canvas.drawString(
        18 * mm,
        12 * mm,
        "TZcup DG-202604 | 非技术得分补齐包 | 未实测项不升级为通过",
    )
    canvas.drawRightString(A4[0] - 18 * mm, 12 * mm, f"第 {doc.page} 页")
    canvas.restoreState()


def build() -> Path:
    root = Path(__file__).resolve().parents[1]
    output = root / "pdf" / "TZcup-nontechnical-score-closure.pdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    pdfmetrics.registerFont(UnicodeCIDFont(FONT))
    styles = make_styles()

    official = json.loads((root / "data/official-score-items.json").read_text(encoding="utf-8"))
    closure = json.loads((root / "data/closure-status.json").read_text(encoding="utf-8"))
    maintenance = json.loads((root / "data/maintenance-procedure.json").read_text(encoding="utf-8"))
    reliability = json.loads((root / "data/reliability-event-dictionary.json").read_text(encoding="utf-8"))
    social = json.loads((root / "data/social-benefit-scenarios.json").read_text(encoding="utf-8"))
    ip = json.loads((root / "data/ip-boundary.json").read_text(encoding="utf-8"))
    with (root / "data/demo-storyboard.csv").open(encoding="utf-8-sig", newline="") as stream:
        storyboard = list(csv.DictReader(stream))
    with (root / "data/demo-acceptance-checklist.csv").open(encoding="utf-8-sig", newline="") as stream:
        demo_checklist = list(csv.DictReader(stream))
    with (root / "data/industrialization-gates.csv").open(encoding="utf-8-sig", newline="") as stream:
        gates = list(csv.DictReader(stream))

    story: list[object] = []
    story.append(Spacer(1, 42 * mm))
    story.append(P("TZcup 首轮非技术得分补齐包", styles["title"]))
    story.append(P("DG-202604 应用价值、方案完成度与产业化补充材料", styles["subtitle"]))
    story.append(Spacer(1, 15 * mm))
    story.append(
        table(
            [
                ["项目", "值"],
                ["官方PDF SHA-256", official["source_pdf_sha256"].upper()],
                ["正式报告基线", f"{closure['baselines']['submission_report']['branch']} / {closure['baselines']['submission_report']['commit']}"],
                ["价值包基线", f"{closure['baselines']['value_package']['branch']} / {closure['baselines']['value_package']['commit']}"],
                ["Gazebo / S100P", "未启动 / 未访问"],
                ["本包正式主张分值", "0 分"],
            ],
            [42 * mm, 120 * mm],
            styles,
        )
    )
    story.append(Spacer(1, 12 * mm))
    story.append(
        P(
            "本包把不依赖新增仿真或板端运行的材料整理成可审计交付。"
            "计划、设计、模型、局部视频和模板均保留原始状态，不转换成实测通过。",
            styles["note"],
        )
    )
    story.append(PageBreak())

    story.append(P("1. 执行摘要", styles["h1"]))
    story.append(
        P(
            "非技术分最容易提升的部分是文档可读性、产业化评审路径、维护程序冻结和演示制作套件。"
            "培训和长期可靠性仍依赖真实用户与运行时间；社会效益依赖现场对照；专利/论文依赖真实申请或发表。",
            styles["body"],
        )
    )
    story.append(
        table(
            [
                ["优先级", "分项", "分值", "本轮材料", "闭证阻塞"],
                ["P0", "文档质量", 5, "独立PDF、索引、哈希、复算", "无新增实验，需最终冻结"],
                ["P0", "产业化方案", 5, "G0-G5门槛、成本公式、风险", "客户场地和真实报价"],
                ["P1", "三步维护", 5, "数字程序、验证协议、测试", "实件快拆件与操作试验"],
                ["P1", "演示视频", 4, "分镜、解说、媒体校验器", "完整5-10分钟任务视频"],
                ["P2", "社会效益", 5, "参数模型和来源分层", "同场景前后对照"],
                ["P2", "硬件/软件完整", 6, "模块-证据-缺口矩阵", "端到端任务和感知/建图"],
                ["P3", "3天掌握", 2, "认证接口与判定算法", "真实新手队列"],
                ["P3", "日均故障率", 3, "事件字典与统计资格", "299个零故障车辆日"],
                ["P4", "专利/论文", 5, "披露模板与论文提纲", "无申请或发表证据"],
            ],
            [16 * mm, 37 * mm, 14 * mm, 52 * mm, 53 * mm],
            styles,
        )
    )

    story.append(PageBreak())
    story.append(P("2. 官方分项与诚实边界", styles["h1"]))
    rows = [["ID", "分项", "分值", "状态"]]
    for item in official["items"]:
        rows.append(
            [
                item["id"],
                item["item"],
                item["max_points"],
                item["status"],
            ]
        )
    story.append(table(rows, [42 * mm, 54 * mm, 14 * mm, 60 * mm], styles))
    story.append(PageBreak())
    story.append(P("2.1 可以主张与不能主张", styles["h1"]))
    rows = [["ID", "可以主张", "不能主张"]]
    for item in official["items"]:
        rows.append([item["id"], item["claimable"], item["not_claimable"]])
    story.append(table(rows, [42 * mm, 64 * mm, 64 * mm], styles))

    story.append(PageBreak())
    story.append(P("3. 3天操作员认证接口", styles["h1"]))
    story.append(
        P(
            "现有课程来自 value-package。当前证据等级为 CURRICULUM，不是 USER_QUEUE。"
            "认证台账必须保存所有参与者、退出者和失败项。",
            styles["body"],
        )
    )
    story.append(
        table(
            [
                ["证据等级", "允许结论"],
                ["CURRICULUM", "存在课程和验收表，不能证明新手掌握"],
                ["INTERNAL_REHEARSAL", "项目成员演练过，不能代表目标用户"],
                ["USER_QUEUE", "目标用户真实完成，可报告掌握率"],
                ["FIELD_ACCEPTED", "用户队列和现场运行均通过"],
            ],
            [48 * mm, 122 * mm],
            styles,
        )
    )
    story.append(Spacer(1, 5 * mm))
    story.append(
        table(
            [
                ["通过门", "冻结规则"],
                ["安全项", "全部为A且不得出现关键安全错误"],
                ["操作项", "A/B比例不低于90%"],
                ["时间", "第三天结束前通过才计入“3天掌握”"],
                ["报告", "必须包含未完成和退出人数"],
            ],
            [45 * mm, 125 * mm],
            styles,
        )
    )

    story.append(PageBreak())
    story.append(P("4. 三步维护：冻结程序与物理验收", styles["h1"]))
    story.append(
        P(
            "安全停车、任务取消、维护模式和能量隔离是前置条件，不计入三步机械更换。"
            "机械步骤从打开服务开口开始，到复位检查结束。",
            styles["body"],
        )
    )
    procedure_rows = [["部件", "步骤1", "步骤2", "步骤3"]]
    for procedure in maintenance["procedures"]:
        procedure_rows.append(
            [procedure["title"]]
            + [step["action"] for step in procedure["steps"]]
        )
    story.append(table(procedure_rows, [30 * mm, 48 * mm, 50 * mm, 42 * mm], styles))
    story.append(Spacer(1, 5 * mm))
    story.append(
        table(
            [["物理验收项", "状态", "通过判据"]]
            + [
                [item["id"], item["status"], item["criterion"]]
                for item in maintenance["physical_acceptance"]
            ],
            [35 * mm, 44 * mm, 91 * mm],
            styles,
        )
    )

    story.append(PageBreak())
    story.append(P("5. 日均故障率统计资格", styles["h1"]))
    story.append(
        P(
            "车辆日分母在开窗之前冻结。主故障事件使用车辆日计数，"
            "故障导致车辆无法出勤时也必须保留在审计中。",
            styles["body"],
        )
    )
    qualification = reliability["qualification"]
    story.append(
        table(
            [
                ["置信度", "目标上界", "零主故障", "1个主故障", "2个主故障"],
                [
                    qualification["confidence"],
                    qualification["upper_bound_target"],
                    qualification["zero_fault_min_vehicle_days"],
                    qualification["one_fault_min_vehicle_days"],
                    qualification["two_fault_min_vehicle_days"],
                ],
            ],
            [25 * mm, 25 * mm, 30 * mm, 35 * mm, 35 * mm],
            styles,
        )
    )
    story.append(Spacer(1, 5 * mm))
    story.append(P("主故障集合", styles["h2"]))
    story.append(P("、".join(reliability["primary_fault_event_ids"]), styles["body"]))
    story.append(P("允许排除但必须留痕", styles["h2"]))
    story.append(P("、".join(reliability["excluded_event_ids"]), styles["body"]))
    story.append(
        P(
            "299车日是统计门槛，不是已经观测到的运行天数；项目当前状态仍为 NOT_MEASURED。",
            styles["note"],
        )
    )

    story.append(PageBreak())
    story.append(P("6. 社会效益参数模型", styles["h1"]))
    story.append(
        P(
            "社会效益必须同时报告人工工时、监护、返工、能耗和清洁质量。"
            "当前情景均为模型假设，不具有项目结果效力。",
            styles["body"],
        )
    )
    benefit_rows = [["情景", "人工小时/日", "自主后小时/日", "减少小时", "减少率", "电量差kWh"]]
    for scenario in social["scenarios"]:
        baseline = sum(
            scenario[key]
            for key in (
                "manual_cleaning_hours_per_day",
                "manual_supervision_hours_per_day",
                "manual_rework_hours_per_day",
            )
        )
        autonomous = sum(
            scenario[key]
            for key in (
                "autonomous_supervision_hours_per_day",
                "autonomous_rework_hours_per_day",
            )
        )
        benefit_rows.append(
            [
                scenario["label"],
                baseline,
                autonomous,
                baseline - autonomous,
                f"{(baseline - autonomous) / baseline:.1%}",
                scenario["vehicle_kwh_per_day"]
                - scenario["manual_equipment_kwh_per_day"],
            ]
        )
    story.append(table(benefit_rows, [25 * mm, 26 * mm, 28 * mm, 22 * mm, 22 * mm, 27 * mm], styles))
    story.append(
        P(
            "quality_claimable=false，project_result_claimable=false。"
            "没有同场景前后对照和清洁质量数据时，不填写社会效益结论。",
            styles["note"],
        )
    )

    story.append(PageBreak())
    story.append(P("7. 硬件/软件完整度", styles["h1"]))
    story.append(
        table(
            [
                ["模块", "状态", "当前能力", "阻断"],
                ["车辆与场景", "PARTIAL", "URDF、传感器、刷体、污物状态", "维护件和实件接口未冻结"],
                ["感知", "FAIL", "RGB-D/2D和GPU离线校准", "官方95%未测，策略召回不足"],
                ["建图", "PARTIAL", "离线射线重建和历史板端回放", "实时SLAM未闭环"],
                ["定位", "PARTIAL", "严格配对、Nav2路线", "max失效，统计口径未冻结"],
                ["规划与安全", "PARTIAL", "Nav2、速度门、急停", "动态避障与完整任务未闭环"],
                ["清扫执行", "PASS_MEASURED", "600mm连续清除带", "只覆盖受控固定试验"],
                ["任务解析", "PASS_INTERNAL", "冻结UTF-8到DSL", "不是公开盲测或语音端到端"],
                ["板端", "PARTIAL", "控制/定位/建图回放", "感知BLOCKED，无联合负载"],
                ["文档与回滚", "DELIVERED", "报告、索引、失败、哈希", "不改变技术未达标"],
            ],
            [34 * mm, 32 * mm, 48 * mm, 56 * mm],
            styles,
        )
    )
    story.append(
        P(
            "可以主张模块化方案完整、证据可追踪；不能主张端到端功能完整。",
            styles["note"],
        )
    )

    story.append(PageBreak())
    story.append(P("8. 演示视频制作套件", styles["h1"]))
    story.append(
        P(
            "正式目标时长570秒。所有任务关键片段必须来自同一冻结版本，"
            "剪辑不得隐藏失败或把局部片段伪装成完整任务。",
            styles["body"],
        )
    )
    storyboard_rows = [["序", "时间", "场景", "必需内容", "验收"]]
    for row in storyboard:
        storyboard_rows.append(
            [
                row["order"],
                f"{row['start_s']}-{row['end_s']}s",
                row["scene"],
                row["required_content"],
                row["acceptance"],
            ]
        )
    story.append(table(storyboard_rows, [8 * mm, 20 * mm, 28 * mm, 60 * mm, 54 * mm], styles))

    story.append(PageBreak())
    story.append(P("9. 演示媒体自动验收", styles["h1"]))
    story.append(
        table(
            [["ID", "类别", "要求", "证据", "阻塞"]]
            + [
                [
                    row["id"],
                    row["category"],
                    row["requirement"],
                    row["evidence"],
                    row["blocking"],
                ]
                for row in demo_checklist
            ],
            [20 * mm, 25 * mm, 57 * mm, 52 * mm, 16 * mm],
            styles,
        )
    )
    story.append(
        P(
            "已有6.333秒GUI片段会因时长不足而FAIL。必须保留该回执，不能调低5分钟下限。",
            styles["note"],
        )
    )

    story.append(PageBreak())
    story.append(P("10. 产业化门槛", styles["h1"]))
    story.append(
        table(
            [["Gate", "阶段", "Go条件", "当前", "所需证据"]]
            + [
                [row["id"], row["stage"], row["go_condition"], row["status"], row["required_evidence"]]
                for row in gates
            ],
            [14 * mm, 30 * mm, 55 * mm, 27 * mm, 44 * mm],
            styles,
        )
    )
    story.append(
        P(
            "G0仅代表离线证据冻结。台架、影子模式、受限自主、扩区和量产均未运行。",
            styles["note"],
        )
    )

    story.append(PageBreak())
    story.append(P("11. 专利与论文边界", styles["h1"]))
    story.append(
        table(
            [
                ["项目", "当前记录", "可主张"],
                ["专利申请", ip["patent_application"], "否"],
                ["专利授权", ip["patent_grant"], "否"],
                ["论文投稿", ip["paper_submission"], "否"],
                ["论文发表", ip["paper_publication"], "否"],
            ],
            [38 * mm, 92 * mm, 40 * mm],
            styles,
        )
    )
    story.append(Spacer(1, 5 * mm))
    story.append(
        P(
            "候选方向："
            + "；".join(ip["candidate_topics"])
            + "。候选方向和披露模板不构成新颖性结论，也不构成申请或发表证据。",
            styles["body"],
        )
    )

    story.append(PageBreak())
    story.append(P("12. 验收与复现", styles["h1"]))
    story.append(
        table(
            [
                ["检查", "命令", "预期"],
                ["单元测试", "py -3 -m unittest discover -s tests -v", "全部通过"],
                ["封包校验", "py -3 tools\\verify_package.py --write", "PASS，官方主张0分"],
                ["可靠性计划", "py -3 tools\\reliability_qualification.py --plan --write", "299/473/628"],
                ["维护程序", "py -3 tools\\validate_maintenance_procedure.py", "PASS，物理未测"],
                ["社会效益", "py -3 tools\\social_benefit_model.py --scenario data\\social-benefit-scenarios.json", "模型结果不可作为项目结果"],
                ["PDF重建", "py -3 tools\\build_supplement_pdf.py", "生成独立补充PDF"],
            ],
            [27 * mm, 85 * mm, 58 * mm],
            styles,
        )
    )
    story.append(Spacer(1, 8 * mm))
    story.append(
        P(
            "结论：本包提升了文档、维护设计、演示准备和产业化材料的可审阅性；"
            "它没有新增用户、长稳、实件、现场或知识产权事实，因此不把任何未测项写成通过。",
            styles["note"],
        )
    )

    doc = SimpleDocTemplate(
        str(output),
        pagesize=A4,
        leftMargin=18 * mm,
        rightMargin=18 * mm,
        topMargin=16 * mm,
        bottomMargin=18 * mm,
        title="TZcup 首轮非技术得分补齐包",
        author="TZcup project",
    )
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.parse_args()
    output = build()
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
