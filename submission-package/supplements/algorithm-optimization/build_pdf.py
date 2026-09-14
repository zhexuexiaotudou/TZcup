#!/usr/bin/env python3
"""Render the algorithm-optimization review brief as a standalone PDF."""

from __future__ import annotations

import html
from pathlib import Path

from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import Paragraph


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "README.md"
OUTPUT = ROOT / "algorithm-optimization-brief.pdf"
PAGE_SIZE = (595.28, 841.89)


def main() -> int:
    pdfmetrics.registerFont(TTFont("CN", r"C:\Windows\Fonts\simsun.ttc", subfontIndex=0))
    body = ParagraphStyle(
        "body",
        fontName="CN",
        fontSize=11.2,
        leading=20,
        wordWrap="CJK",
        textColor="#263849",
    )
    heading = ParagraphStyle(
        "heading",
        parent=body,
        fontSize=15,
        leading=24,
        textColor="#123048",
        spaceBefore=4,
        spaceAfter=2,
    )

    lines = SOURCE.read_text(encoding="utf-8").splitlines()
    title = "算法优化审查简报"
    paragraphs: list[tuple[str, str]] = []
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("# "):
            continue
        if stripped.startswith("## "):
            paragraphs.append(("heading", stripped[3:]))
        elif stripped.startswith("- "):
            paragraphs.append(("body", "• " + stripped[2:]))
        else:
            paragraphs.append(("body", stripped))

    pdf = canvas.Canvas(str(OUTPUT), pagesize=PAGE_SIZE)
    pdf.setTitle("DG-202604 算法优化审查简报")
    page = 1
    y = 790
    pdf.setFillColorRGB(0.06, 0.18, 0.26)
    pdf.rect(0, 745, PAGE_SIZE[0], 96, fill=1, stroke=0)
    pdf.setFillColorRGB(1, 1, 1)
    pdf.setFont("CN", 19)
    pdf.drawString(43, 790, title)
    pdf.setFont("CN", 10)
    pdf.drawString(43, 762, "DG-202604 | PARTIAL REVIEW | 未升级任何 official live gate")
    y = 713

    for kind, text in paragraphs:
        style = heading if kind == "heading" else body
        para = Paragraph(html.escape(text), style)
        width, height = para.wrap(505, 650)
        if y - height < 70:
            pdf.setFont("CN", 9)
            pdf.drawString(43, 43, "算法优化审查简报")
            pdf.drawRightString(550, 43, f"{page}")
            pdf.showPage()
            page += 1
            y = 790
        para.drawOn(pdf, 45, y - height)
        y -= height + (16 if kind == "heading" else 10)

    pdf.setFont("CN", 9)
    pdf.drawString(43, 43, "算法优化审查简报")
    pdf.drawRightString(550, 43, f"{page}")
    pdf.save()
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
