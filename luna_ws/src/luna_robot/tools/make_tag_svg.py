#!/usr/bin/env python3
"""Print file for the R2 dock tag: tag36h11 ID 0, 80 mm black square, 100 mm with its white ring, on a 104 mm cut
line (the tag mast plate). Print at 100 % (no "fit to page") on a laser printer and check the 80 mm with a ruler.
    python3 tools/make_tag_svg.py ../../../docs/dock_tag36h11_id0_80mm.svg"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from luna_robot.core import tag36h11            # noqa: E402
from luna_robot.core.dock_tag import TagGeometry  # noqa: E402

PLATE = TagGeometry().plate_m * 1000


def svg(tag_id=0, size_mm=TagGeometry().size_m * 1000):
    cell = size_mm / tag36h11.BLACK_WIDTH
    full = cell * tag36h11.TOTAL_WIDTH
    o = (PLATE - full) / 2
    page = PLATE + 20
    rects = []
    for r, row in enumerate(tag36h11.bitmap(tag_id)):
        for c, white in enumerate(row):
            if not white:
                rects.append(f'<rect x="{10 + o + c * cell:.3f}" y="{10 + o + r * cell:.3f}" width="{cell:.3f}" height="{cell:.3f}"/>')
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{page}mm" height="{page + 8}mm" viewBox="0 0 {page} {page + 8}">
<rect x="0" y="0" width="{page}" height="{page + 8}" fill="#fff"/>
<rect x="10" y="10" width="{PLATE}" height="{PLATE}" fill="none" stroke="#999" stroke-width="0.2" stroke-dasharray="1 1"/>
<g fill="#000" shape-rendering="crispEdges">{''.join(rects)}</g>
<text x="10" y="{page + 3}" font-family="sans-serif" font-size="2.6">tag36h11 ID {tag_id} · siyah kare {size_mm:g} mm · beyaz halka dahil {full:g} mm · kesik çizgi: {PLATE:g} mm plaka (etiket direği)</text>
<text x="10" y="{page + 6.5}" font-family="sans-serif" font-size="2.6">%100 ölçekle yazdırın; siyah karenin kenarını cetvelle {size_mm:g} mm olarak doğrulayın.</text>
</svg>
'''


if __name__ == '__main__':
    with open(sys.argv[1], 'w') as f:
        f.write(svg())
