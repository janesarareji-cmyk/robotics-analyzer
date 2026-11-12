"""R8.3: ReportData -> PDF with Qt ``QTextDocument`` + ``QPdfWriter`` (no new dependency).

Needs a ``QGuiApplication`` (fonts). Graph images are added as document resources.
"""
from __future__ import annotations

import re

from .html_report import render_html
from .report import ReportData


def write_pdf(d: ReportData, path) -> int:
    """Write the PDF report to ``path``. Returns the page count (>= 1)."""
    from PySide6.QtCore import QMarginsF, QUrl
    from PySide6.QtGui import (QGuiApplication, QImage, QPageLayout, QPageSize, QPdfWriter,
                               QTextDocument)

    if QGuiApplication.instance() is None:
        raise RuntimeError("PDF report needs a running Qt application")
    names = {}

    def src(i, _png):
        names[i] = f"graph{i}.png"
        return names[i]

    html = render_html(d, image_src=src)
    doc = QTextDocument()
    for i, (_t, png) in enumerate(d.graphs):
        img = QImage.fromData(png, "PNG")
        doc.addResource(QTextDocument.ImageResource, QUrl(f"graph{i}.png"), img)
    doc.setHtml(html)
    writer = QPdfWriter(str(path))
    writer.setPageSize(QPageSize(QPageSize.A4))
    writer.setPageMargins(QMarginsF(15, 15, 15, 15), QPageLayout.Millimeter)
    writer.setTitle(d.title)
    doc.print_(writer)
    del writer                                  # flush and close the file
    data = open(path, "rb").read()
    return max(1, len(re.findall(rb"/Type\s*/Page\b", data)))
