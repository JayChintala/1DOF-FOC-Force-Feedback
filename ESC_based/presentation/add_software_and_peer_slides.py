# -*- coding: utf-8 -*-
"""
Updates "Force-Aware Telepresence for Robot Learning.pptx" with the
2026-11-15 session:

  edits   Results: Force Readings   - chart + captions, scale recheck done
          Results: Where It Stands  - calibration and current-error rows
          Remaining Work            - force accuracy done; peer-to-peer added
  adds    The Software              after Next Steps
          Peer-to-Peer Coupling     after Results: Where It Stands
          Peer-to-Peer: Measured    after Peer-to-Peer Coupling
          Two Approaches            after Peer-to-Peer: Measured

Geometry is copied from the Rigid-vs-Soft and Where-It-Stands slides so the
new slides sit identically. Idempotent by construction: it always rebuilds
from the untouched backup, so re-running cannot leave orphaned parts.

Needs python-pptx + Pillow.   python3 add_software_and_peer_slides.py
"""
import copy
import os
import shutil

from PIL import Image
from pptx import Presentation
from pptx.chart.data import XyChartData
from pptx.dml.color import RGBColor
from pptx.util import Inches, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
DECK = os.path.join(HERE, "Force-Aware Telepresence for Robot Learning.pptx")
BACKUP = os.path.join(
    HERE, "Force-Aware Telepresence for Robot Learning (before 2026-11-15 update).pptx")

TITLE_BOX = (Inches(0.96), Inches(0.38), Inches(11.50), Inches(1.45))
CAP_X, CAP_W = Inches(0.87), Inches(11.59)
CAP_Y, CAP_H = Inches(6.25), Inches(1.05)
CAPTION_PT, CAPTION_RGB = 12, RGBColor(0x52, 0x51, 0x4E)

# --- 2026-11-15 scale check (b7_force_cal, Tafter 400) ----------------------
IQ_NEW = [0.0496, 0.0992, 0.1983, 0.2993, 0.4479, 0.5973, 0.7887]
N_NEW = [0.0196, 0.0785, 0.1765, 0.3040, 0.4609, 0.5982, 0.8042]
IQ_OLD, N_OLD = [0.574, 0.654], [0.7063, 0.9712]      # 2026-10-28, before the fix
SLOPE, INTERCEPT = 1.0582, -0.0261


def first_line(slide):
    for sh in slide.shapes:
        if sh.has_text_frame and sh.text_frame.text.strip():
            return sh.text_frame.text.strip().split("\n")[0]
    return ""


def find(prs, title):
    for s in prs.slides:
        if first_line(s) == title:
            return s
    raise KeyError(title)


def set_caption(shape, text):
    """Replace a caption's text, keeping its first run's formatting."""
    tf = shape.text_frame
    p0 = tf.paragraphs[0]
    keep = p0.runs[0]
    keep.text = text
    for extra in p0.runs[1:]:
        extra._r.getparent().remove(extra._r)
    for extra in tf.paragraphs[1:]:
        extra._p.getparent().remove(extra._p)


# ---------------------------------------------------------------- edits ----
def edit_force_readings(prs):
    s = find(prs, "Results: Force Readings")
    charts = [sh for sh in s.shapes if sh.has_chart]
    cd = XyChartData()
    a = cd.add_series("Scale, 2026-11-15 — all 7 levels")
    for x, y in zip(IQ_NEW, N_NEW):
        a.add_data_point(x, y)
    b = cd.add_series("2026-10-28, before Tafter 400")
    for x, y in zip(IQ_OLD, N_OLD):
        b.add_data_point(x, y)
    c = cd.add_series("1.058 N/A fit (R² 0.999)")
    for x in (0.0, 0.8):
        c.add_data_point(x, SLOPE * x + INTERCEPT)
    charts[0].chart.replace_data(cd)

    boxes = [sh for sh in s.shapes if sh.has_text_frame and sh.name.startswith("TextBox")]
    set_caption(boxes[0],
                "Scale recheck done (2026-11-15), 7 levels from 0.05 to 0.8 A: 1.058 N/A, "
                "R² 0.999, largest residual 1.4 g. The two 2026-10-28 points above 0.5 A sat "
                "off the line because the current read low; the Tafter 400 firmware fix removed "
                "that, and the top level is now the best-fitting point in the set.")
    set_caption(boxes[1],
                "14 steady holds pressing the scale through teleop (2026-10-28): the harder the "
                "push, the harder the follower presses, about 10.6 g per 100 mA. In a still hold "
                "this match is by design (felt gain 1, offset = 20 mA dead band); the independent "
                "check is the scale recheck on the left.")


def edit_where_it_stands(prs):
    s = find(prs, "Results: Where It Stands")
    tb = [sh for sh in s.shapes if sh.has_table][0].table
    for ri in range(len(tb.rows)):
        metric = tb.cell(ri, 0).text.strip()
        new = None
        if metric == "Current reading error at 0.8 A":
            new = "−1.4 %"
        elif metric == "Force calibration at the tip":
            new = "1.058 N/A, R² 0.999, to 0.8 A"
        if new:
            cell = tb.cell(ri, 2)
            p0 = cell.text_frame.paragraphs[0]
            p0.runs[0].text = new
            for extra in p0.runs[1:]:
                extra._r.getparent().remove(extra._r)
    box = [sh for sh in s.shapes if sh.has_text_frame and sh.name.startswith("TextBox")][0]
    set_caption(box,
                "Before = the original law, or the step just before each fix. The rise times come "
                "from different tests. Calibration and current error rechecked on the scale "
                "2026-11-15. Peak force is capped at ≈ 0.84 N at the tip by the 0.8 A motor rating.")


def edit_remaining_work(prs):
    s = find(prs, "Remaining Work")
    body = [sh for sh in s.shapes
            if sh.has_text_frame and sh.name.startswith("Content Placeholder")][0]
    new = [
        "Blind rigid-vs-soft trials by feel — press below ≈ 0.5 N, since the 0.84 N ceiling "
        "flattens both materials",
        "Peer-to-peer: damp the ≈ 57 Hz resonance, and fit a plant model for motor B so friction "
        "and gravity can be cancelled on both sides",
        "More force: gearbox or capstan drive (now capped at ≈ 0.84 N at the tip)",
        "Scale to 3 degrees of freedom and record force-labelled demonstrations for robot learning",
    ]
    paras = body.text_frame.paragraphs
    for para, text in zip(paras, new):
        para.runs[0].text = text
        for extra in para.runs[1:]:
            extra._r.getparent().remove(extra._r)


# ------------------------------------------------------------ new slides ----
def new_picture_slide(prs, title, image, img_w, img_y, caption, notes):
    layout = next(l for l in prs.slide_masters[0].slide_layouts if l.name == "Title Only")
    slide = prs.slides.add_slide(layout)
    t = slide.shapes.title
    t.left, t.top, t.width, t.height = TITLE_BOX
    t.text_frame.text = title

    path = os.path.join(HERE, image)
    iw, ih = Image.open(path).size
    h = int(img_w * ih / iw)
    pic = slide.shapes.add_picture(path, 0, img_y, width=img_w, height=h)
    pic.left = int((prs.slide_width - img_w) / 2)
    pic.name = title + " figure"

    box = slide.shapes.add_textbox(CAP_X, CAP_Y, CAP_W, CAP_H)
    box.text_frame.word_wrap = True
    run = box.text_frame.paragraphs[0].add_run()
    run.text = caption
    run.font.size = Pt(CAPTION_PT)
    run.font.color.rgb = CAPTION_RGB
    slide.notes_slide.notes_text_frame.text = notes
    return slide


def new_table_slide(prs, title, rows, caption, notes, model_table):
    layout = next(l for l in prs.slide_masters[0].slide_layouts if l.name == "Title Only")
    slide = prs.slides.add_slide(layout)
    t = slide.shapes.title
    t.left, t.top, t.width, t.height = TITLE_BOX
    t.text_frame.text = title

    widths = [Inches(4.10), Inches(3.70), Inches(3.70)]
    gf = slide.shapes.add_table(len(rows), 3, Inches(0.96), Inches(1.80),
                                sum(widths, Inches(0)), Inches(0.55) * len(rows))
    tbl = gf.table
    # adopt the deck's table style rather than python-pptx's default
    src = copy.deepcopy(model_table._tbl.find(
        "{http://schemas.openxmlformats.org/drawingml/2006/main}tblPr"))
    old = tbl._tbl.find("{http://schemas.openxmlformats.org/drawingml/2006/main}tblPr")
    tbl._tbl.replace(old, src)
    for c, w in zip(tbl.columns, widths):
        c.width = w
    for r in tbl.rows:
        r.height = Inches(0.55)
    for ri, row in enumerate(rows):
        for ci, text in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text_frame.word_wrap = True
            p = cell.text_frame.paragraphs[0]
            run = p.add_run()
            run.text = text
            run.font.size = Pt(14)
            run.font.bold = (ri == 0)

    box = slide.shapes.add_textbox(CAP_X, Inches(6.34), CAP_W, Inches(0.95))
    box.text_frame.word_wrap = True
    run = box.text_frame.paragraphs[0].add_run()
    run.text = caption
    run.font.size = Pt(CAPTION_PT)
    run.font.color.rgb = CAPTION_RGB
    slide.notes_slide.notes_text_frame.text = notes
    return slide


COMPARE = [
    ["", "Leader–follower", "Peer-to-peer"],
    ["Roles", "fixed: one leads, one follows", "none — same law on both"],
    ["Law runs on", "the Pi, 500 Hz", "both ESCs, 1 kHz"],
    ["The hand feels", "estimated contact force", "the spring between the shafts"],
    ["Free motion", "light — the follower is powered", "3.17× heavier — both rotors"],
    ["Friction + gravity", "cancelled by a fitted model", "not cancelled (0.11–0.19 N)"],
    ["Rigid vs soft", "20× apart, measured", "not tested yet"],
    ["Hands off", "stable", "≈ 57 Hz resonance"],
]


def main():
    if not os.path.exists(BACKUP):
        shutil.copy2(DECK, BACKUP)
        print("backed up ->", os.path.basename(BACKUP))
    shutil.copy2(BACKUP, DECK)          # always start pristine

    prs = Presentation(DECK)
    edit_force_readings(prs);  print("edited  Results: Force Readings")
    edit_where_it_stands(prs); print("edited  Results: Where It Stands")
    edit_remaining_work(prs);  print("edited  Remaining Work")

    model_table = [sh for sh in find(prs, "Results: Where It Stands").shapes
                   if sh.has_table][0].table

    plan = [
        ("Next Steps", lambda: new_picture_slide(
            prs, "The Software", "diagram_system_notitle.png", Inches(9.0), Inches(1.72),
            "Both boards run the ESC_based firmware — 16 kHz FOC current loop, 1 kHz control "
            "tick, telemetry every 1 ms. The Pi runs the teleoperation loop at 500 Hz over one CAN "
            "bus and consumes every telemetry frame (0 dropped in 60 s). Every run is recorded in "
            "TEST_LOG.md — 6,180 lines, 85 dated entries, each with its numbers against a stated "
            "pass threshold.",
            "ESC_based replaced the earlier design in which the Pi computed everything. The "
            "coupling law can now run on the ESCs themselves at 1 kHz with the Pi out of the loop, "
            "which is what peer-to-peer uses.\n\n"
            "Host side: one kernel-filtered CAN socket shared by both motors, RX thread at "
            "real-time priority, a per-frame telemetry queue, and logs staged in RAM to avoid "
            "SD-card stalls. Velocity and the observers are per-instance classes, with dt taken "
            "from the ESC's own microsecond clock rather than a nominal constant.\n\n"
            "TEST_LOG.md is the record: every run, its numbers against a stated threshold, PASS or "
            "FAIL, the log paths, and the reasoning whenever something failed.")),
        ("Results: Where It Stands", lambda: new_picture_slide(
            prs, "Peer-to-Peer Coupling", "diagram_peer.png", Inches(11.0), Inches(1.75),
            "Newest change: drop the leader/follower split. Each ESC runs the same spring-damper "
            "law with own and peer swapped, reading the other's telemetry straight off the CAN bus "
            "at 1 kHz. Because the error is antisymmetric the two currents come out equal and "
            "opposite, so neither motor is in charge — hold one in each hand and each feels the other.",
            "This is PEER mode, already in the ESC_based firmware, so no new code was needed.\n\n"
            "own0 and peer0 are captured when the mode engages, which only happens once both "
            "shafts are still, so both boards take their zero from the same physical state.\n\n"
            "Kd_local, a damping term on the motor's own velocity, is available and set to zero "
            "here. It is the only term with no bus delay.")),
        ("Peer-to-Peer Coupling", lambda: new_picture_slide(
            prs, "Peer-to-Peer: Measured", "peer_to_peer.png", Inches(11.59), Inches(2.15),
            "60 s two-handed, Kp 0.007 A/count: equal and opposite to within 6 % (corr +0.998, and "
            "exactly 1.000 while saturated), the two shafts tracking to a median 7 counts — 0.83 mm "
            "at the tip. Open: a ≈ 57 Hz resonance that is undamped unless a hand is on it.",
            "Measured 2026-11-15, firmware PEER mode, peer age under 1 ms throughout.\n\n"
            "The ~57 Hz mode is the coupled natural frequency: sqrt(Kp*(G_A+G_B)) predicts 48 Hz. "
            "Adding local damping did nothing, and dropping the peer-position prediction cut it 29 "
            "percent at rest but cost 40 percent in motion. Untried: a lower Kp, which would move "
            "the mode to about 36 Hz.")),
        ("Peer-to-Peer: Measured", lambda: new_table_slide(
            prs, "Two Approaches", COMPARE,
            "Peer-to-peer is not a replacement — it is the honest-physics version. Leader–follower "
            "feels lighter only because it is asymmetric: the follower's own supply accelerates its "
            "rotor, so the operator's hand never pays for it. Through a spring, you always do.",
            "Which one to use depends on the goal. For recording force-labelled demonstrations, "
            "leader-follower is still the right architecture: it is transparent, calibrated, and "
            "the follower can touch an environment the operator is not touching.\n\n"
            "Peer-to-peer is the better fit for two-handed interaction, and for any case where "
            "both ends are held by a person. Its remaining work is damping the resonance and "
            "cancelling friction and gravity locally on each motor, which needs a plant model for "
            "motor B.", model_table)),
    ]

    lst = prs.slides._sldIdLst
    for anchor, make in plan:
        slide = make()
        idx = [first_line(s) for s in prs.slides].index(anchor)
        el = list(lst)[-1]
        lst.remove(el)
        lst.insert(idx + 1, el)
        print("inserted %-24r after %-26r -> position %d"
              % (first_line(slide), anchor, idx + 2))

    prs.save(DECK)
    print("saved %s (%d slides)" % (os.path.basename(DECK), len(list(lst))))


if __name__ == "__main__":
    main()
