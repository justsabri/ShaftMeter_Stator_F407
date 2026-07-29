import html
import json
import os
import re
import subprocess
import zipfile
from collections import defaultdict, deque
from pathlib import Path
from xml.sax.saxutils import escape


EMU_PER_INCH = 914400


def xml_text(text: str) -> str:
    return escape(text, {'"': '&quot;'})


def run_xml(text: str, bold=False, size=21):
    b = "<w:b/>" if bold else ""
    return (
        "<w:r><w:rPr>"
        '<w:rFonts w:ascii="Microsoft YaHei" w:hAnsi="Microsoft YaHei" w:eastAsia="Microsoft YaHei"/>'
        f"{b}<w:sz w:val=\"{size}\"/><w:szCs w:val=\"{size}\"/>"
        "</w:rPr>"
        f"<w:t xml:space=\"preserve\">{xml_text(text)}</w:t>"
        "</w:r>"
    )


def para_xml(text: str, style=None, bold=False, size=21, indent_left=None):
    ppr = ""
    if style:
        ppr += f'<w:pStyle w:val="{style}"/>'
    if indent_left is not None:
        ppr += f'<w:ind w:left="{indent_left}"/>'
    ppr = f"<w:pPr>{ppr}</w:pPr>" if ppr else ""
    return f"<w:p>{ppr}{run_xml(text, bold=bold, size=size)}</w:p>"


def table_xml(rows):
    if not rows:
        return ""
    max_cols = max(len(r) for r in rows)
    tbl = [
        "<w:tbl>",
        "<w:tblPr><w:tblStyle w:val=\"TableGrid\"/><w:tblW w:w=\"0\" w:type=\"auto\"/>"
        "<w:tblBorders><w:top w:val=\"single\" w:sz=\"4\" w:space=\"0\" w:color=\"auto\"/>"
        "<w:left w:val=\"single\" w:sz=\"4\" w:space=\"0\" w:color=\"auto\"/>"
        "<w:bottom w:val=\"single\" w:sz=\"4\" w:space=\"0\" w:color=\"auto\"/>"
        "<w:right w:val=\"single\" w:sz=\"4\" w:space=\"0\" w:color=\"auto\"/>"
        "<w:insideH w:val=\"single\" w:sz=\"4\" w:space=\"0\" w:color=\"auto\"/>"
        "<w:insideV w:val=\"single\" w:sz=\"4\" w:space=\"0\" w:color=\"auto\"/></w:tblBorders></w:tblPr>",
    ]
    for ridx, row in enumerate(rows):
        tbl.append("<w:tr>")
        for c in row + [""] * (max_cols - len(row)):
            shade = '<w:shd w:fill="EDEDED"/>' if ridx == 0 else ""
            tbl.append(
                "<w:tc><w:tcPr><w:tcW w:w=\"2400\" w:type=\"dxa\"/>"
                f"{shade}</w:tcPr>"
                f"{para_xml(c, bold=(ridx == 0), size=19)}</w:tc>"
            )
        tbl.append("</w:tr>")
    tbl.append("</w:tbl>")
    return "".join(tbl)


NODE_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)(?:\[([^\]]+)\]|\{([^}]+)\})")
EDGE_RE = re.compile(
    r"([A-Za-z][A-Za-z0-9_]*)(?:\[[^\]]+\]|\{[^}]+\})?\s*(?:--\s*([^>-]+?)\s*-->|-->)\s*([A-Za-z][A-Za-z0-9_]*)(?:\[([^\]]+)\]|\{([^}]+)\})?"
)


def parse_mermaid(code: str):
    nodes = {}
    shapes = {}
    order = []
    edges = []
    for line in code.splitlines():
        line = line.strip()
        if not line or line.startswith("flowchart"):
            continue
        for nid, label_sq, label_brace in NODE_RE.findall(line):
            label = label_sq or label_brace
            if nid not in nodes:
                nodes[nid] = label
                shapes[nid] = "decision" if label_brace else "process"
                order.append(nid)
        m = EDGE_RE.search(line)
        if m:
            src, elabel, dst, dst_label_sq, dst_label_brace = m.groups()
            dst_label = dst_label_sq or dst_label_brace
            if src not in nodes:
                nodes[src] = src
                shapes[src] = "process"
                order.append(src)
            if dst not in nodes:
                nodes[dst] = dst_label or dst
                shapes[dst] = "decision" if dst_label_brace else "process"
                order.append(dst)
            edges.append((src, dst, (elabel or "").strip()))
    return nodes, shapes, order, edges


def wrap_svg_text(text, max_chars=12):
    parts = []
    cur = ""
    for ch in text:
        cur += ch
        if len(cur) >= max_chars:
            parts.append(cur)
            cur = ""
    if cur:
        parts.append(cur)
    return parts or [""]


def render_png_from_spec(spec, out_path: Path):
    spec_path = out_path.with_suffix(".json")
    spec_path.write_text(json.dumps(spec, ensure_ascii=False), encoding="utf-8")
    script = Path(__file__).with_name("render_flowchart_png.ps1")
    subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(script),
            "-SpecPath",
            str(spec_path),
            "-OutPath",
            str(out_path),
        ],
        check=True,
    )


def mermaid_to_png(code: str, out_path: Path):
    nodes, shapes, order, edges = parse_mermaid(code)
    if not nodes:
        render_png_from_spec(
            {
                "width": 800,
                "height": 120,
                "node_w": 180,
                "node_h": 58,
                "nodes": [],
                "edges": [],
                "positions": {},
            },
            out_path,
        )
        return 800, 120

    incoming = defaultdict(int)
    adj = defaultdict(list)
    for s, d, _ in edges:
        adj[s].append(d)
        incoming[d] += 1
    level = {n: 0 for n in order}
    q = deque([n for n in order if incoming[n] == 0])
    while q:
        n = q.popleft()
        for d in adj[n]:
            level[d] = max(level[d], level[n] + 1)
            incoming[d] -= 1
            if incoming[d] == 0:
                q.append(d)

    by_level = defaultdict(list)
    for n in order:
        by_level[level.get(n, 0)].append(n)

    node_w = 180
    node_h = 58
    x_gap = 44
    y_gap = 64
    margin = 40
    max_cols = max(len(v) for v in by_level.values())
    max_level = max(by_level.keys())
    width = max(760, margin * 2 + max_cols * node_w + (max_cols - 1) * x_gap)
    height = margin * 2 + (max_level + 1) * node_h + max_level * y_gap
    pos = {}
    for lev in range(max_level + 1):
        row = by_level.get(lev, [])
        row_w = len(row) * node_w + max(0, len(row) - 1) * x_gap
        x0 = (width - row_w) / 2
        y = margin + lev * (node_h + y_gap)
        for i, nid in enumerate(row):
            pos[nid] = (x0 + i * (node_w + x_gap), y)

    spec = {
        "width": int(width),
        "height": int(height),
        "node_w": node_w,
        "node_h": node_h,
        "nodes": [{"id": nid, "label": nodes[nid], "shape": shapes.get(nid, "process")} for nid in order],
        "edges": [{"src": s, "dst": d, "label": label} for s, d, label in edges],
        "positions": {nid: {"x": float(x), "y": float(y)} for nid, (x, y) in pos.items()},
    }
    render_png_from_spec(spec, out_path)
    return int(width), int(height)

    items = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<defs><marker id=\"arrow\" markerWidth=\"10\" markerHeight=\"10\" refX=\"9\" refY=\"3\" orient=\"auto\" markerUnits=\"strokeWidth\">"
        "<path d=\"M0,0 L0,6 L9,3 z\" fill=\"#444\"/></marker></defs>",
        '<rect x="0" y="0" width="100%" height="100%" fill="#ffffff"/>',
    ]
    for s, d, elabel in edges:
        if s not in pos or d not in pos:
            continue
        sx, sy = pos[s]
        dx, dy = pos[d]
        x1 = sx + node_w / 2
        y1 = sy + node_h
        x2 = dx + node_w / 2
        y2 = dy
        mid_y = (y1 + y2) / 2
        path = f"M{x1:.1f},{y1:.1f} C{x1:.1f},{mid_y:.1f} {x2:.1f},{mid_y:.1f} {x2:.1f},{y2:.1f}"
        items.append(f'<path d="{path}" fill="none" stroke="#555" stroke-width="1.6" marker-end="url(#arrow)"/>')
        if elabel:
            items.append(
                f'<text x="{(x1+x2)/2:.1f}" y="{mid_y-4:.1f}" text-anchor="middle" '
                'font-size="13" font-family="Microsoft YaHei, SimSun, sans-serif" fill="#333">'
                f'{html.escape(elabel)}</text>'
            )
    for nid in order:
        x, y = pos[nid]
        items.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{node_w}" height="{node_h}" rx="8" fill="#F7FAFC" stroke="#3B6EA8" stroke-width="1.5"/>')
        lines = wrap_svg_text(nodes[nid], 12)
        start_y = y + node_h / 2 - (len(lines) - 1) * 9
        for i, line in enumerate(lines[:3]):
            items.append(
                f'<text x="{x + node_w/2:.1f}" y="{start_y + i*18:.1f}" text-anchor="middle" '
                'dominant-baseline="middle" font-size="14" font-family="Microsoft YaHei, SimSun, sans-serif" fill="#111">'
                f'{html.escape(line)}</text>'
            )
    items.append("</svg>")
    out_path.write_text("\n".join(items), encoding="utf-8")
    return int(width), int(height)


def image_xml(rid, width_px, height_px):
    max_w = 6.4 * EMU_PER_INCH
    w = min(max_w, width_px / 96 * EMU_PER_INCH)
    h = w * height_px / max(width_px, 1)
    return f"""
<w:p><w:pPr><w:jc w:val="center"/></w:pPr><w:r><w:drawing>
<wp:inline distT="0" distB="0" distL="0" distR="0" xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing">
<wp:extent cx="{int(w)}" cy="{int(h)}"/><wp:docPr id="{rid[3:]}" name="Flowchart {rid[3:]}"/>
<a:graphic xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">
<pic:pic xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">
<pic:nvPicPr><pic:cNvPr id="{rid[3:]}" name="flowchart.svg"/><pic:cNvPicPr/></pic:nvPicPr>
<pic:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>
<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{int(w)}" cy="{int(h)}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>
</pic:pic></a:graphicData></a:graphic>
</wp:inline></w:drawing></w:r></w:p>
"""


def parse_table(lines, idx):
    rows = []
    while idx < len(lines) and lines[idx].strip().startswith("|"):
        line = lines[idx].strip().strip("|")
        cells = [c.strip() for c in line.split("|")]
        if not all(re.fullmatch(r":?-{3,}:?", c) for c in cells):
            rows.append(cells)
        idx += 1
    return rows, idx


def md_to_blocks(md_path: Path, media_dir: Path):
    lines = md_path.read_text(encoding="utf-8").splitlines()
    blocks = []
    i = 0
    img_no = 1
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        if line.startswith("```mermaid"):
            code = []
            i += 1
            while i < len(lines) and not lines[i].startswith("```"):
                code.append(lines[i])
                i += 1
            i += 1
            img_name = f"flowchart{img_no}.png"
            w, h = mermaid_to_png("\n".join(code), media_dir / img_name)
            blocks.append(("image", img_name, w, h))
            img_no += 1
            continue
        m = re.match(r"^(#{1,6})\s+(.*)", line)
        if m:
            level = len(m.group(1))
            blocks.append(("heading", level, m.group(2).strip()))
            i += 1
            continue
        if line.strip().startswith("|"):
            rows, i = parse_table(lines, i)
            blocks.append(("table", rows))
            continue
        m = re.match(r"^\s*(\d+)\.\s+(.*)", line)
        if m:
            blocks.append(("list", f"{m.group(1)}. {m.group(2).strip()}"))
            i += 1
            continue
        para = [line.strip()]
        i += 1
        while i < len(lines):
            nxt = lines[i]
            if not nxt.strip() or nxt.startswith("#") or nxt.startswith("```") or nxt.strip().startswith("|") or re.match(r"^\s*\d+\.\s+", nxt):
                break
            para.append(nxt.strip())
            i += 1
        blocks.append(("para", " ".join(para)))
    return blocks


def document_xml(blocks):
    body = []
    image_id = 1
    for b in blocks:
        if b[0] == "heading":
            level, text = b[1], b[2]
            style = f"Heading{min(level, 3)}"
            size = {1: 32, 2: 28, 3: 24}.get(level, 22)
            body.append(para_xml(text, style=style, bold=True, size=size))
        elif b[0] == "para":
            body.append(para_xml(b[1], size=21))
        elif b[0] == "list":
            body.append(para_xml(b[1], size=21, indent_left=420))
        elif b[0] == "table":
            body.append(table_xml(b[1]))
        elif b[0] == "image":
            body.append(image_xml(f"rId{image_id}", b[2], b[3]))
            image_id += 1
    body.append(
        '<w:sectPr><w:pgSz w:w="11906" w:h="16838"/>'
        '<w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" w:header="720" w:footer="720" w:gutter="0"/>'
        "</w:sectPr>"
    )
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:wpc="http://schemas.microsoft.com/office/word/2010/wordprocessingCanvas"
xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
xmlns:o="urn:schemas-microsoft-com:office:office"
xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math"
xmlns:v="urn:schemas-microsoft-com:vml"
xmlns:wp14="http://schemas.microsoft.com/office/word/2010/wordprocessingDrawing"
xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"
xmlns:w10="urn:schemas-microsoft-com:office:word"
xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"
xmlns:w14="http://schemas.microsoft.com/office/word/2010/wordml"
xmlns:wpg="http://schemas.microsoft.com/office/word/2010/wordprocessingGroup"
xmlns:wpi="http://schemas.microsoft.com/office/word/2010/wordprocessingInk"
xmlns:wne="http://schemas.microsoft.com/office/word/2006/wordml"
xmlns:wps="http://schemas.microsoft.com/office/word/2010/wordprocessingShape"
xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture"
mc:Ignorable="w14 wp14"><w:body>{''.join(body)}</w:body></w:document>"""


def styles_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:rPr><w:rFonts w:ascii="Microsoft YaHei" w:hAnsi="Microsoft YaHei" w:eastAsia="Microsoft YaHei"/><w:sz w:val="21"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:basedOn w:val="Normal"/><w:pPr><w:spacing w:before="360" w:after="120"/></w:pPr><w:rPr><w:b/><w:sz w:val="32"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:pPr><w:spacing w:before="300" w:after="100"/></w:pPr><w:rPr><w:b/><w:sz w:val="28"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading3"><w:name w:val="heading 3"/><w:basedOn w:val="Normal"/><w:pPr><w:spacing w:before="240" w:after="80"/></w:pPr><w:rPr><w:b/><w:sz w:val="24"/></w:rPr></w:style>
<w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/><w:tblPr><w:tblBorders><w:top w:val="single" w:sz="4" w:color="auto"/><w:left w:val="single" w:sz="4" w:color="auto"/><w:bottom w:val="single" w:sz="4" w:color="auto"/><w:right w:val="single" w:sz="4" w:color="auto"/><w:insideH w:val="single" w:sz="4" w:color="auto"/><w:insideV w:val="single" w:sz="4" w:color="auto"/></w:tblBorders></w:tblPr></w:style>
</w:styles>"""


def create_docx(md_path: Path, out_path: Path):
    work_media = out_path.parent / "_docx_flowcharts"
    work_media.mkdir(exist_ok=True)
    blocks = md_to_blocks(md_path, work_media)
    images = [b[1] for b in blocks if b[0] == "image"]

    rels = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">']
    for idx, img in enumerate(images, start=1):
        rels.append(f'<Relationship Id="rId{idx}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="media/{img}"/>')
    rels.append("</Relationships>")

    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="xml" ContentType="application/xml"/>
<Default Extension="png" ContentType="image/png"/>
<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
</Types>""")
        z.writestr("_rels/.rels", """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
</Relationships>""")
        z.writestr("word/document.xml", document_xml(blocks))
        z.writestr("word/styles.xml", styles_xml())
        z.writestr("word/_rels/document.xml.rels", "".join(rels))
        for img in images:
            z.write(work_media / img, f"word/media/{img}")


if __name__ == "__main__":
    root = Path.cwd()
    md = root / "软著设计说明书-船用轴功率与推力仪定子软件.md"
    out = root / "软著设计说明书-船用轴功率与推力仪定子软件.docx"
    create_docx(md, out)
    print(out)
