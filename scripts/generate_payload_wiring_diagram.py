from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'docs' / 'spot_payload_wiring_diagram_minimal_bw.png'


def font(size, bold=False):
    names = [
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
        if bold
        else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        '/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf'
        if bold
        else '/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf',
    ]
    for name in names:
        path = Path(name)
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


F_TITLE = font(44, True)
F_H = font(28, True)
F = font(24)
F_SMALL = font(20)
F_TINY = font(18)


def box(draw, xy, title, lines=()):
    x1, y1, x2, y2 = xy
    draw.rounded_rectangle(xy, radius=18, outline='black', width=3, fill='white')
    draw.text((x1 + 22, y1 + 18), title, fill='black', font=F_H)
    y = y1 + 62
    for line in lines:
        draw.text((x1 + 22, y), line, fill='black', font=F_SMALL)
        y += 30


def line(draw, p1, p2, label='', dashed=False, label_offset=(0, 0)):
    if dashed:
        draw_dashed_line(draw, p1, p2, width=3)
    else:
        draw.line([p1, p2], fill='black', width=4)
    if label:
        lx = (p1[0] + p2[0]) // 2 + label_offset[0]
        ly = (p1[1] + p2[1]) // 2 + label_offset[1]
        tw = draw.textlength(label, font=F_TINY)
        draw.rectangle((lx - 8, ly - 4, lx + tw + 8, ly + 24), fill='white')
        draw.text((lx, ly), label, fill='black', font=F_TINY)


def path(draw, points, label='', dashed=False, label_at=0, label_offset=(0, 0)):
    for i in range(len(points) - 1):
        line(draw, points[i], points[i + 1], dashed=dashed)
    if label:
        p1 = points[label_at]
        p2 = points[label_at + 1]
        lx = (p1[0] + p2[0]) // 2 + label_offset[0]
        ly = (p1[1] + p2[1]) // 2 + label_offset[1]
        tw = draw.textlength(label, font=F_TINY)
        draw.rectangle((lx - 8, ly - 4, lx + tw + 8, ly + 24), fill='white')
        draw.text((lx, ly), label, fill='black', font=F_TINY)


def draw_dashed_line(draw, p1, p2, width=3, dash=18, gap=12):
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    length = (dx * dx + dy * dy) ** 0.5
    if length == 0:
        return
    ux = dx / length
    uy = dy / length
    distance = 0
    while distance < length:
        start = distance
        end = min(distance + dash, length)
        draw.line(
            [
                (x1 + ux * start, y1 + uy * start),
                (x1 + ux * end, y1 + uy * end),
            ],
            fill='black',
            width=width,
        )
        distance += dash + gap


def table(draw, xy, rows):
    x, y = xy
    widths = [150, 280]
    row_h = 42
    draw.rectangle((x, y, x + sum(widths), y + row_h * len(rows)), outline='black', width=2)
    draw.line((x + widths[0], y, x + widths[0], y + row_h * len(rows)), fill='black', width=2)
    for i, row in enumerate(rows):
        yy = y + i * row_h
        if i:
            draw.line((x, yy, x + sum(widths), yy), fill='black', width=1)
        draw.text((x + 12, yy + 9), row[0], fill='black', font=F_TINY)
        draw.text((x + widths[0] + 12, yy + 9), row[1], fill='black', font=F_TINY)


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new('RGB', (2550, 1600), 'white')
    draw = ImageDraw.Draw(image)

    draw.text(
        (80, 54),
        'Spot GXP Payload Wiring - Minimal Power/Data Layout',
        fill='black',
        font=F_TITLE,
    )

    box(
        draw,
        (90, 180, 470, 430),
        'Spot GXP',
        ['HD15 power breakout', 'RJ45 payload network'],
    )
    table(
        draw,
        (90, 465),
        [
            ('GND', 'pins 1-4, 6'),
            ('12V', 'pins 7, 8, 11, 12'),
            ('24V', 'pins 13-14 unused'),
            ('5V', 'pin 15 unused'),
            ('PPS', 'pin 5 unused'),
        ],
    )

    box(
        draw,
        (600, 180, 1020, 430),
        'HD15 Solder Cup',
        ['Join all 12V pins to +12V bus', 'Join all GND pins to ground bus'],
    )
    box(
        draw,
        (1140, 170, 1560, 430),
        'Inline Fuse Block',
        ['F1: Jetson barrel', 'F2: Router 5V converter', 'F3: OAK 5V converter'],
    )

    box(draw, (1830, 120, 2290, 310), 'Jetson Orin Nano', ['Barrel jack power', 'ROS 2 compute'])
    box(draw, (1830, 510, 2290, 700), 'GL.iNet Router', ['USB-C power', 'Private mission LAN'])
    box(draw, (1830, 900, 2290, 1090), 'Luxonis OAK-D Pro W', ['USB-C power', 'USB data to Jetson'])

    box(draw, (1140, 510, 1560, 700), '12V -> 5V USB-C', ['Router power converter'])
    box(draw, (1140, 900, 1560, 1090), '12V -> 5V USB-C', ['OAK power converter'])

    box(draw, (90, 1230, 470, 1420), 'GXP RJ45', ['Optional Ethernet path'])
    box(draw, (690, 1230, 1070, 1420), 'Jetson Ethernet', ['Use if wired LAN is available'])
    box(draw, (1360, 1230, 1740, 1420), 'Tablet / Laptop', ['Wi-Fi to router'])

    line(draw, (470, 260), (600, 260), 'HD15 cable', label_offset=(-24, -36))
    line(draw, (1020, 245), (1140, 245), '+12V', label_offset=(-12, -34))
    line(draw, (1020, 345), (1140, 345), 'GND', label_offset=(-10, 10))

    line(draw, (1560, 230), (1830, 230), 'F1 + GND', label_offset=(-32, -34))
    path(draw, [(1350, 430), (1350, 510)], 'F2 12V', label_offset=(-56, 12))
    line(draw, (1560, 605), (1830, 605), '5V USB-C', label_offset=(-44, -34))
    path(draw, [(1430, 430), (1430, 900)], 'F3 12V', label_offset=(-56, 118))
    line(draw, (1560, 995), (1830, 995), '5V USB-C', label_offset=(-44, -34))

    path(
        draw,
        [(2290, 995), (2360, 995), (2360, 230), (2290, 230)],
        'USB data only',
        dashed=True,
        label_at=1,
        label_offset=(12, -36),
    )

    line(draw, (470, 1325), (690, 1325), 'Ethernet', label_offset=(-32, -36))
    path(
        draw,
        [(2290, 605), (2440, 605), (2440, 1325), (1740, 1325)],
        'Wi-Fi LAN',
        dashed=True,
        label_at=1,
        label_offset=(-82, 88),
    )

    draw.text((80, 1500), 'Verify Jetson barrel voltage/polarity and final fuse sizes before powering hardware.', fill='black', font=F)
    draw.text((80, 1540), 'OAK-D and router each use their own regulated USB-C power path; Jetson USB is data for OAK.', fill='black', font=F)

    image.save(OUTPUT)
    print(OUTPUT)


if __name__ == '__main__':
    main()
