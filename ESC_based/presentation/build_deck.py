"""
Builds force_feedback_PI_update.pptx (16:9) from the block diagrams, cropped log
plots in deck_assets/, and numbers from ESC_based/software/TEST_LOG.md.

Needs python-pptx:  python3 build_deck.py
"""
import csv
import os
import random
import re

from lxml import etree
from pptx import Presentation
from pptx.chart.data import CategoryChartData, XyChartData
from pptx.dml.color import RGBColor
from pptx.enum.chart import (XL_CHART_TYPE, XL_LEGEND_POSITION, XL_MARKER_STYLE, XL_LABEL_POSITION,
                             XL_TICK_LABEL_POSITION)
from pptx.enum.dml import MSO_LINE_DASH_STYLE
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR, MSO_AUTO_SIZE
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt, Emu

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "deck_assets")
LOGS = os.path.join(HERE, "..", "software", "logs")
OUT = os.path.join(HERE, "force_feedback_PI_update.pptx")

FONT = "Calibri"
EQ_FONT = "Cambria"
INK, MUTED = "1F2933", "5F6B7A"
LEADER, FOLLOWER, GREEN, RED, PURPLE = "2A6FDB", "D9682C", "178A68", "C0392B", "6F4FA8"
NEUTRAL, T_LEADER, T_FOLLOWER, T_GREEN, T_RED = "F3F5F7", "E7EFFD", "FDEEE4", "E2F4EC", "FBEAEA"
DARK, ON_DARK, ON_DARK_MUTED = "1F2933", "FFFFFF", "C9D2DB"

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
L_TITLE, L_TITLE_ONLY = prs.slide_layouts[0], prs.slide_layouts[5]
slide_no = [0]


def rgb(h):
    return RGBColor.from_string(h)


def style_run(run, size=16, bold=False, color=INK, italic=False, font=FONT, sub=False):
    f = run.font
    f.size, f.bold, f.italic, f.name = Pt(size), bold, italic, font
    f.color.rgb = rgb(color)
    if sub:
        run._r.get_or_add_rPr().set("baseline", "-25000")


def fill_tf(tf, paras, anchor=MSO_ANCHOR.TOP, margin=None):
    """paras: list of dicts {runs: [(text, style)], align, bullet, num, space_after, space_before}."""
    tf.word_wrap = True
    tf.auto_size = MSO_AUTO_SIZE.NONE
    tf.vertical_anchor = anchor
    if margin is not None:
        tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = Inches(margin)
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        para.alignment = p.get("align", PP_ALIGN.LEFT)
        if p.get("space_after") is not None:
            para.space_after = Pt(p["space_after"])
        if p.get("space_before") is not None:
            para.space_before = Pt(p["space_before"])
        if p.get("bullet") or p.get("num"):
            pPr = para._p.get_or_add_pPr()
            pPr.set("marL", str(Emu(Inches(0.3))))
            pPr.set("indent", str(-Emu(Inches(0.3))))
            if p.get("num"):
                bu = etree.SubElement(pPr, qn("a:buAutoNum"))
                bu.set("type", "arabicPeriod")
            else:
                bu = etree.SubElement(pPr, qn("a:buChar"))
                bu.set("char", "•")
        for text, st in p["runs"]:
            r = para.add_run()
            r.text = text
            style_run(r, **st)
    return tf


UNIT = re.compile(r"(\d) (%|A|mA|ms|s|Hz|kHz|N|N/A|counts)(?!\w)")


def P(text, align=PP_ALIGN.LEFT, space_after=None, bullet=False, num=False, space_before=None, **st):
    text = UNIT.sub("\\1\u00a0\\2", text)
    return {"runs": [(text, st)], "align": align, "space_after": space_after,
            "bullet": bullet, "num": num, "space_before": space_before}


def text(slide, x, y, w, h, paras, anchor=MSO_ANCHOR.TOP, margin=0.0, name=None):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        tb.name = name
    fill_tf(tb.text_frame, paras, anchor, margin)
    return tb


def card(slide, x, y, w, h, fill=NEUTRAL, paras=(), anchor=MSO_ANCHOR.TOP, margin=0.18, name=None,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE):
    s = slide.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        s.name = name
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE:
        s.adjustments[0] = 0.08
    s.fill.solid()
    s.fill.fore_color.rgb = rgb(fill)
    s.line.fill.background()
    s.shadow.inherit = False
    if paras:
        fill_tf(s.text_frame, list(paras), anchor, margin)
    return s


def tag(slide, label, color):
    w = 0.115 * len(label) + 0.4
    s = card(slide, 0.6, 0.38, w, 0.32, fill=color, margin=0.0, anchor=MSO_ANCHOR.MIDDLE, name="Section tag",
             paras=[P(label, align=PP_ALIGN.CENTER, size=11, bold=True, color="FFFFFF")])
    s.adjustments[0] = 0.5


def number(slide):
    slide_no[0] += 1
    text(slide, 12.1, 7.05, 0.63, 0.3, [P(str(slide_no[0]), align=PP_ALIGN.RIGHT, size=10, color=MUTED)],
         name="Slide number")


def content_slide(title, label, color, notes):
    s = prs.slides.add_slide(L_TITLE_ONLY)
    t = s.shapes.title
    t.left, t.top, t.width, t.height = Inches(0.6), Inches(0.78), Inches(12.13), Inches(0.72)
    fill_tf(t.text_frame, [P(title, size=32, bold=True, color=INK)], anchor=MSO_ANCHOR.TOP, margin=0.0)
    tag(s, label, color)
    s.notes_slide.notes_text_frame.text = notes
    number(s)
    return s


def picture(slide, path, x, y, w=None, h=None, alt=""):
    pic = slide.shapes.add_picture(path, Inches(x), Inches(y),
                                   Inches(w) if w else None, Inches(h) if h else None)
    pic._element.nvPicPr.cNvPr.set("descr", alt)
    return pic


def alt_chart(gf, alt):
    gf._element.nvGraphicFramePr.cNvPr.set("descr", alt)


def quiet_axes(chart, x_title=None, y_title=None, size=12):
    chart.font.name, chart.font.size = FONT, Pt(size)
    chart.font.color.rgb = rgb(MUTED)
    for ax, ttl in ((chart.category_axis, x_title), (chart.value_axis, y_title)):
        ax.format.line.color.rgb = rgb("B8C2CC")
        ax.has_major_gridlines = False
        if ttl:
            ax.has_title = True
            ax.axis_title.text_frame.text = ttl
            r = ax.axis_title.text_frame.paragraphs[0].runs[0]
            style_run(r, size=12, color=MUTED)
    for ax in (chart.category_axis, chart.value_axis):
        ax.tick_label_position = XL_TICK_LABEL_POSITION.LOW
    chart.value_axis.has_major_gridlines = True
    chart.value_axis.major_gridlines.format.line.color.rgb = rgb("E3E7EB")


def chart_title(chart, s):
    chart.has_title = True
    chart.chart_title.text_frame.text = s
    style_run(chart.chart_title.text_frame.paragraphs[0].runs[0], size=14, bold=True, color=INK)


def dark(slide):
    slide.background.fill.solid()
    slide.background.fill.fore_color.rgb = rgb(DARK)


# =========================================================== 1. Title
s = prs.slides.add_slide(L_TITLE)
dark(s)
t = s.shapes.title
t.left, t.top, t.width, t.height = Inches(0.9), Inches(2.0), Inches(11.5), Inches(1.6)
fill_tf(t.text_frame, [P("Sensorless Force-Feedback Teleoperation", size=44, bold=True, color=ON_DARK)],
        anchor=MSO_ANCHOR.BOTTOM, margin=0.0)
sub = s.placeholders[1]
sub.left, sub.top, sub.width, sub.height = Inches(0.9), Inches(3.75), Inches(11.5), Inches(1.0)
fill_tf(sub.text_frame, [P("A 1-DOF leader–follower with two FOC motors: how it works and where it stands",
                           size=22, color=ON_DARK_MUTED)], margin=0.0)
for i, (lbl, col) in enumerate((("LEADER", LEADER), ("FOLLOWER", FOLLOWER))):
    card(s, 0.9 + i * 2.55, 5.2, 1.9, 0.5, fill=col, anchor=MSO_ANCHOR.MIDDLE, margin=0.0,
         name=f"{lbl.title()} chip", paras=[P(lbl, align=PP_ALIGN.CENTER, size=14, bold=True, color="FFFFFF")])
text(s, 2.85, 5.18, 0.6, 0.5, [P("⇄", align=PP_ALIGN.CENTER, size=26, color=ON_DARK_MUTED)],
     anchor=MSO_ANCHOR.MIDDLE)
text(s, 0.9, 6.3, 11.5, 0.5, [P("Jay Chintala  ·  November 2026", size=16, color=ON_DARK_MUTED)])
s.notes_slide.notes_text_frame.text = (
    "One motor senses the contact force, the other reproduces it for the operator, "
    "with no force sensor anywhere. This talk: how it works, what changed, and what is left.")
slide_no[0] += 1

# =========================================================== 2. Goal
s = content_slide("Feel what the follower touches, without a force sensor", "GOAL", INK,
                  "The current in each motor is the only force signal. Success means an operator can tell "
                  "a rigid object from a soft one by feel alone, in blind trials. The 0.8 A motor rating "
                  "caps the force at about 0.84 N at the arm tip.")
text(s, 0.6, 1.8, 5.7, 5.0, [
    P("The motor current is the force sensor.", size=22, bold=True, color=INK, space_after=14),
    P("Leader: turned by the operator's hand", bullet=True, size=17, space_after=8),
    P("Follower: its arm touches the object", bullet=True, size=17, space_after=8),
    P("Each motor's current and motion are used to estimate the external torque on it",
      bullet=True, size=17, space_after=8),
    P("Success: telling rigid from soft by feel, in blind trials", bullet=True, size=17, bold=True,
      color=GREEN),
])
stats = [("0", "force or torque sensors", INK, NEUTRAL),
         ("16 kHz", "current loop on each ESC", LEADER, T_LEADER),
         ("500 Hz", "teleoperation loop on the Pi\n(1 kHz telemetry over CAN)", GREEN, T_GREEN),
         ("0.8 A", "motor rating, about 0.84 N\nat the follower arm tip", FOLLOWER, T_FOLLOWER)]
for i, (big, lbl, col, tint) in enumerate(stats):
    x, y = 6.75 + (i % 2) * 3.1, 1.8 + (i // 2) * 2.55
    card(s, x, y, 2.85, 2.25, fill=tint, anchor=MSO_ANCHOR.MIDDLE, margin=0.2, name=f"Stat {i + 1}",
         paras=[P(big, size=40, bold=True, color=col, space_after=4), P(lbl, size=14, color=INK)])

# =========================================================== 3. System
s = content_slide("Two ESCs, one Pi, one CAN bus", "SYSTEM", INK,
                  "Each ESC closes its own 16 kHz current loop and streams current, position and a "
                  "timestamp at 1 kHz. The Pi runs the teleoperation loop at 500 Hz and decides every "
                  "current command. Firmware changes made the streamed current trustworthy and added "
                  "a 20 ms command timeout for safety.")
picture(s, os.path.join(ASSETS, "system.png"), 1.05, 1.6, w=11.2,
        alt="Block diagram: Raspberry Pi 4 on a 1 Mbit/s CAN bus with two B-G431B-ESC1 boards, "
            "each driving a motor; commands at 500 Hz down, telemetry at 1 kHz up.")

# =========================================================== 4. Original law
s = content_slide("Starting point: the leader felt the follower's raw current", "STARTING POINT", MUTED,
                  "The forward path already used a hand-torque observer. The problem was the reflection "
                  "path: the leader was driven by the follower's own filtered current, which mostly "
                  "contains its inertia, its friction and the echo of my push.")
picture(s, os.path.join(ASSETS, "original.png"), 1.0, 1.6, w=11.33,
        alt="Original control law block diagram: hand-torque observer feeds the follower; the leader "
            "receives a low-pass filtered copy of the follower's measured current.")

# =========================================================== 5. Echo
s = content_slide("That felt force was an echo of my own push", "PROBLEM", RED,
                  "Each dot is one control tick from two Phase A runs: presses against a table, and "
                  "presses into foam. The leader command equals minus my hand torque with slope -1.01 "
                  "and r = -0.997, whatever the object. So the operator felt their own push, and table "
                  "and foam were indistinguishable.")
random.seed(1)
xy = XyChartData()
for fname, label in (("teleop_KVs3e-05_FB-1_5.csv", "Presses on a table"),
                     ("teleop_KVs3e-05_FB-1_6.csv", "Presses into foam")):
    rows = []
    with open(os.path.join(LOGS, fname)) as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["tau_h"]), float(r["iq_b_cmd"])))
            except (KeyError, ValueError):
                pass
    ser = xy.add_series(label)
    for a, b in random.sample(rows, 250):
        ser.add_data_point(a, b)
ref = xy.add_series("Exact echo (slope −1)")
ref.add_data_point(-0.8, 0.8)
ref.add_data_point(0.8, -0.8)
gf = s.shapes.add_chart(XL_CHART_TYPE.XY_SCATTER, Inches(0.6), Inches(1.6), Inches(7.4), Inches(5.3), xy)
alt_chart(gf, "Scatter of leader current command against estimated hand torque; table and foam points "
              "both lie on a line of slope -1.")
ch = gf.chart
quiet_axes(ch, "hand torque on the leader, estimated (A)", "current commanded to the leader (A)")
for ax in (ch.category_axis, ch.value_axis):
    ax.minimum_scale, ax.maximum_scale, ax.major_unit = -0.8, 0.8, 0.4
    ax.tick_labels.number_format, ax.tick_labels.number_format_is_linked = "0.0", False
ch.has_legend = True
ch.legend.position, ch.legend.include_in_layout = XL_LEGEND_POSITION.TOP, False
ch.legend.font.size = Pt(12)
for ser, col in zip(ch.plots[0].series, (FOLLOWER, GREEN, INK)):
    if col == INK:
        ser.marker.style = XL_MARKER_STYLE.NONE
        ser.format.line.color.rgb = rgb(INK)
        ser.format.line.width = Pt(1.5)
        ser.format.line.dash_style = MSO_LINE_DASH_STYLE.DASH
    else:
        ser.format.line.fill.background()
        ser.marker.style, ser.marker.size = XL_MARKER_STYLE.CIRCLE, 5
        ser.marker.format.fill.solid()
        ser.marker.format.fill.fore_color.rgb = rgb(col)
        ser.marker.format.line.fill.background()
text(s, 0.6, 6.95, 7.4, 0.3, [P("Phase A, 2026-10-20: 250 random ticks from each run (18k each); "
                                "fit over all ticks: slope −1.01, r = −0.997.", size=11, color=MUTED)])
for i, (big, lbl, col, tint) in enumerate((
        ("−1.01", "slope of leader command vs my hand torque,\nidentical for table and foam", RED, T_RED),
        ("116 mA", "RMS force felt in free space,\nwith nothing touching the follower", RED, T_RED),
        ("66 ms", "contact rise time (10–90 %)", MUTED, NEUTRAL))):
    card(s, 8.45, 1.7 + i * 1.8, 4.28, 1.55, fill=tint, anchor=MSO_ANCHOR.MIDDLE, margin=0.2,
         name=f"Callout {i + 1}",
         paras=[P(big, size=32, bold=True, color=col, space_after=2), P(lbl, size=14, color=INK)])

# =========================================================== 6. Key idea
s = content_slide("Use each motor as its own force sensor", "KEY IDEA", GREEN,
                  "Newton for each motor: acceleration is G times the sum of motor current and external "
                  "torque, both in amps. So the current that did not become acceleration is the external "
                  "torque. It is computed in a derivative-free observer form at 20 Hz. It only works if G, "
                  "the friction and gravity model, and the current reading are all right.")
eq = card(s, 0.6, 1.75, 12.13, 1.75, fill=T_GREEN, name="Equation band")
tb = text(s, 0.9, 1.75, 11.6, 1.75, [])
tf = tb.text_frame
fill_tf(tf, anchor=MSO_ANCHOR.MIDDLE, paras=[{"runs": [("plant:  ", dict(size=18, color=MUTED)),
                       ("dv/dt = G · (i + τ", dict(size=30, italic=True, font=EQ_FONT)),
                       ("ext", dict(size=30, italic=True, font=EQ_FONT, sub=True)),
                       (")", dict(size=30, italic=True, font=EQ_FONT)),
                       ("      ⇒      observer:  ", dict(size=18, color=MUTED)),
                       ("τ", dict(size=30, italic=True, font=EQ_FONT, color=GREEN, bold=True)),
                       ("ext", dict(size=30, italic=True, font=EQ_FONT, color=GREEN, bold=True, sub=True)),
                       (" ≈ a/G − i", dict(size=30, italic=True, font=EQ_FONT, color=GREEN, bold=True))],
              "align": PP_ALIGN.CENTER, "space_after": 10},
             P("The measured current that did not turn into acceleration is the torque from outside "
               "(the hand, or the object). Derivative-free disturbance observer, filtered at 20 Hz.",
               align=PP_ALIGN.CENTER, size=16, color=INK)])
for i, (hdr, body, col, tint) in enumerate((
        ("1 · Measured plant gain G",
         "Measured, not taken from a datasheet: 3.85e6 counts/s² per A for the 3D-printed follower arm. "
         "The leader observer uses 7.9e6 (a re-measurement says 22 % higher).", LEADER, T_LEADER),
        ("2 · Friction and gravity model",
         "Fitted to the 3D-printed follower arm and subtracted from the estimate. Friction is about "
         "100× what was first assumed.", FOLLOWER, T_FOLLOWER),
        ("3 · Accurate current reading",
         "The estimate is only as good as the measured current, so the firmware was fixed first "
         "(next slide).", GREEN, T_GREEN))):
    card(s, 0.6 + i * 4.14, 3.85, 3.85, 2.3, fill=tint, margin=0.22, name=f"Requirement {i + 1}",
         paras=[P(hdr, size=18, bold=True, color=col, space_after=8), P(body, size=15, color=INK)])
text(s, 0.6, 6.35, 12.13, 0.3, [P("Everything the observer needs, in order: G, the model, the current.",
                                  size=12, color=MUTED, italic=True)])

# =========================================================== 7. Firmware
s = content_slide("Making the current measurement trustworthy", "FIX · FIRMWARE", LEADER,
                  "Four firmware changes on both ESCs. The big one: above about 0.55 A at some rotor "
                  "angles the current read low, up to 14 % at 0.8 A, while the real current overshot to "
                  "about 0.93 A. Raising the ADC sampling delay Tafter made it worse; lowering it to 400 "
                  "fixed it, within 2 % up to 0.8 A at a bad and a good angle. 400 also won a blind A/B "
                  "teleop comparison.")
items = (("9.7× less noise", "Telemetry sends the 1 ms mean of 16 current samples, not one sample.", LEADER),
         ("Correct offsets", "Calibration averages longer, with the bridge switching. Phase C had been "
                             "114–140 mA off.", LEADER),
         ("No breakdown above 0.55 A", "ADC sampling delay (Tafter) set to 400: error −14 % → −2 % at 0.8 A.",
          GREEN),
         ("Safer", "20 ms command timeout; over-current trip restored to its real level.", INK))
for i, (hdr, body, col) in enumerate(items):
    text(s, 0.6, 1.75 + i * 1.3, 5.4, 1.2, [P(hdr, size=19, bold=True, color=col, space_after=3),
                                             P(body, size=15, color=INK)])
cd = CategoryChartData()
cd.categories = ["0.30", "0.45", "0.60", "0.80"]
tafter = (("Tafter 400 (now)", (-1.4, -1.6, -1.6, -2.0), GREEN, 3.5),
          ("807 (stock)", (-1.4, -1.9, -3.0, -14.0), MUTED, 2.25),
          ("1360", (-1.2, -1.7, -7.1, -31.9), "E8A07A", 2.25),
          ("2040", (-1.4, -3.1, -19.5, -42.6), RED, 2.25))
for name, vals, *_ in tafter:
    cd.add_series(name, vals)
gf = s.shapes.add_chart(XL_CHART_TYPE.LINE_MARKERS, Inches(6.4), Inches(1.6), Inches(6.33), Inches(4.85), cd)
alt_chart(gf, "Line chart: current reading error vs commanded current for four Tafter values; 400 stays "
              "within 2 percent, larger values fall to -14 to -43 percent at 0.8 A.")
ch = gf.chart
quiet_axes(ch, "commanded current (A)", "reading error (%)")
chart_title(ch, "Current reading error, bad rotor angle")
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale, ch.value_axis.major_unit = -45, 5, 10
ch.value_axis.tick_labels.number_format, ch.value_axis.tick_labels.number_format_is_linked = "0", False
ch.has_legend = True
ch.legend.position, ch.legend.include_in_layout = XL_LEGEND_POSITION.BOTTOM, False
ch.legend.font.size = Pt(12)
for ser, (_, _, col, w) in zip(ch.plots[0].series, tafter):
    ser.smooth = False
    ser.format.line.color.rgb, ser.format.line.width = rgb(col), Pt(w)
    ser.marker.style, ser.marker.size = XL_MARKER_STYLE.CIRCLE, 7
    ser.marker.format.fill.solid()
    ser.marker.format.fill.fore_color.rgb = rgb(col)
    ser.marker.format.line.fill.background()
text(s, 6.4, 6.5, 6.33, 0.55, [P("Follower held stalled by hand, 47–80° electrical; pass = within 2 %. "
                                 "At a good angle 400 reads −1.6 % at 0.8 A.", size=11, color=MUTED)])

# =========================================================== 8. Control architecture
s = content_slide("How it works: two observers, each feeding the other motor", "HOW IT WORKS", GREEN,
                  "Top: the leader observer estimates my hand torque and the follower is commanded that "
                  "current, plus weak velocity tracking. Bottom: the follower observer estimates the "
                  "environment torque, minus its own friction and gravity, and the leader is commanded "
                  "that. Both are feedforward, so they add no loop gain. The starred terms switch on "
                  "only in contact: they make holds steady and contact stiff.")
picture(s, os.path.join(ASSETS, "control.png"), 1.0, 1.6, w=11.33,
        alt="Current control architecture: hand-torque observer feeds follower command; environment-torque "
            "observer feeds leader command; a contact latch enables contact springs on both sides.")

# =========================================================== 9. Free space
s = content_slide("Free space is quiet; contact still arrives fast", "RESULT · FREE SPACE", GREEN,
                  "With nothing touching the follower, the leader used to push back with 116 mA RMS of "
                  "false force. With the environment estimate it is about 19 mA. Contact still comes "
                  "through quickly: a tap reaches 90 % in a median of 4 ms.")
rows = (("116 mA", "before: follower's raw current", "19 mA", "now: environment estimate",
         "RMS false force on the leader, free motion"),
        ("66 ms", "before (hand clamp, 0.55 A preload)", "4 ms", "now: median over taps",
         "contact rise time, 10–90 %"))
for i, (b, bl, a, al, cap) in enumerate(rows):
    y = 1.75 + i * 2.65
    card(s, 0.6, y, 3.5, 2.0, fill=T_RED, anchor=MSO_ANCHOR.MIDDLE, margin=0.2, name=f"Before {i + 1}",
         paras=[P(b, size=40, bold=True, color=RED, space_after=2), P(bl, size=13, color=INK)])
    arr = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(4.3), Inches(y + 0.7), Inches(0.6), Inches(0.6))
    arr.fill.solid()
    arr.fill.fore_color.rgb = rgb("B8C2CC")
    arr.line.fill.background()
    card(s, 5.1, y, 3.5, 2.0, fill=T_GREEN, anchor=MSO_ANCHOR.MIDDLE, margin=0.2, name=f"After {i + 1}",
         paras=[P(a, size=40, bold=True, color=GREEN, space_after=2), P(al, size=13, color=INK)])
    text(s, 0.6, y + 2.05, 8.0, 0.35, [P(cap, size=13, color=MUTED, italic=True)])
text(s, 9.1, 1.75, 3.63, 0.4, [P("Why free space is quiet", size=18, bold=True, color=INK)])
for i, (hdr, body) in enumerate((("Environment estimate", "the follower's own inertia and friction are "
                                                          "removed before anything is felt"),
                                 ("20 mA dead band", "hides the follower's stiction at rest"),
                                 ("Leader cogging cancelled", "a 30 s calibration at session start cuts "
                                                              "the leader's ripple 3–4×"))):
    card(s, 9.1, 2.3 + i * 1.55, 3.63, 1.35, fill=NEUTRAL, anchor=MSO_ANCHOR.MIDDLE, margin=0.18,
         name=f"Reason {i + 1}",
         paras=[P(hdr, size=15, bold=True, color=GREEN, space_after=2), P(body, size=13, color=INK)])

# =========================================================== 10. Holds
s = content_slide("Held presses no longer fade", "FIX · CONTACT", FOLLOWER,
                  "In a static press the force circulates: hand, follower, object, leader, back to the hand "
                  "estimate, with loop gain about 1. The dead band and friction leak a fixed amount each "
                  "pass, so a held press decayed to zero in 1 to 1.5 s. The fix latches the position at "
                  "contact and adds a small one-way spring on the follower, ramped in over 0.3 s.")
text(s, 0.6, 1.7, 7.7, 0.35, [P("Before: no contact spring  (run REF, 2026-10-26)", size=15, bold=True,
                                color=RED)])
picture(s, os.path.join(ASSETS, "ref_force.png"), 0.6, 2.05, w=7.7,
        alt="Estimated contact force over a run without the contact spring; long presses sag toward zero.")
text(s, 0.6, 3.85, 7.7, 0.35, [P("After: contact spring  (run HOLD3, same day)", size=15, bold=True,
                                 color=GREEN)])
picture(s, os.path.join(ASSETS, "hold3_force.png"), 0.6, 4.2, w=7.7,
        alt="Estimated contact force over a run with the contact spring; presses hold flat plateaus.")
text(s, 0.6, 5.85, 7.7, 0.6, [P("Estimated contact force at the follower tip (N) over about 55 s; grey "
                                "bands mark contact. Before: long presses sag. After: flat plateaus.",
                                size=11, color=MUTED)])
for i, (hdr, body, col, tint) in enumerate((
        ("Why it faded", "Force circulates through both observers with loop gain ≈ 1; the dead band "
                         "and friction leak a little each pass.", RED, T_RED),
        ("Fix", "At contact, latch the position; add a one-way spring on the follower "
                "(≤ 50 mA, ramped in over 0.3 s).", FOLLOWER, T_FOLLOWER),
        ("Result", "Hold slope −0.3…−0.5 A/s → about ±0.03 A/s.\n“The holds feel solid.”", GREEN, T_GREEN))):
    card(s, 8.7, 1.7 + i * 1.75, 4.03, 1.55, fill=tint, anchor=MSO_ANCHOR.MIDDLE, margin=0.2,
         name=f"Hold {hdr}",
         paras=[P(hdr, size=16, bold=True, color=col, space_after=3), P(body, size=13, color=INK)])

# =========================================================== 11. Leader spring
s = content_slide("Rigid contact now stops the leader", "FIX · CONTACT", FOLLOWER,
                  "Rigid-versus-soft failed on feel: the follower measured rigid as about 10 times stiffer, "
                  "but the leader travelled 34 degrees while the force built, whatever the object. A leader "
                  "spring on the same contact latch, plus a little damping, cut that to about 2 to 3 "
                  "degrees. A weak slipping position coupling also stopped the follower sagging under gravity.")
cd = CategoryChartData()
cd.categories = ["No leader spring", "+ leader spring", "+ contact damping", "+ position coupling (now)"]
cd.add_series("Leader travel (°)", (34.0, 4.5, 1.8, 2.8))
gf = s.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(0.6), Inches(1.6), Inches(7.3), Inches(4.9), cd)
alt_chart(gf, "Bar chart of leader travel while the felt force builds: 34 degrees without the leader spring, "
              "then 4.5, 1.8 and 2.8 degrees.")
ch = gf.chart
quiet_axes(ch)
chart_title(ch, "Leader travel (degrees) while felt force builds 0.15 → 0.7 N")
ch.category_axis.reverse_order = True
ch.category_axis.tick_labels.font.size = Pt(13)
ch.category_axis.tick_labels.font.color.rgb = rgb(INK)
ch.value_axis.minimum_scale, ch.value_axis.maximum_scale = 0, 40
ch.value_axis.visible = False
ch.value_axis.has_major_gridlines = False
ch.has_legend = False
plot = ch.plots[0]
plot.gap_width = 60
plot.has_data_labels = True
dl = plot.data_labels
dl.number_format, dl.number_format_is_linked = '0.0"°"', False
dl.position = XL_LABEL_POSITION.OUTSIDE_END
dl.font.size, dl.font.bold = Pt(14), True
dl.font.color.rgb = rgb(INK)
for idx, pt in enumerate(plot.series[0].points):
    pt.format.fill.solid()
    pt.format.fill.fore_color.rgb = rgb(RED if idx == 0 else GREEN)
text(s, 0.6, 6.55, 7.3, 0.5, [P("Rigid object, median per press; runs _14, _18, _19, _20 on 2026-10-29.",
                                size=11, color=MUTED)])
card(s, 8.3, 1.7, 4.43, 1.75, fill=T_GREEN, anchor=MSO_ANCHOR.MIDDLE, margin=0.22, name="Travel callout",
     paras=[P("34° → 2–3°", size=36, bold=True, color=GREEN, space_after=2),
            P("leader travel into a rigid object", size=14, color=INK)])
card(s, 8.3, 3.65, 4.43, 1.6, fill=NEUTRAL, anchor=MSO_ANCHOR.MIDDLE, margin=0.22, name="Drift callout",
     paras=[P("167 → 18 counts/s", size=24, bold=True, color=INK, space_after=2),
            P("follower drift with the leader held still (slipping position coupling)", size=13, color=INK)])
card(s, 8.3, 5.45, 4.43, 1.45, fill=T_FOLLOWER, anchor=MSO_ANCHOR.MIDDLE, margin=0.22, name="Quote",
     paras=[P("“The contact with the rigid object was the best it has ever been.”", size=15, italic=True,
              color=INK)])

# =========================================================== 12. Results table
s = content_slide("Where it stands", "RESULTS", GREEN,
                  "Before means the original law, or the step just before each fix. The leader travel figure "
                  "was measured with the new felt force but before the leader spring.")
data = (("Metric", "Before", "Now"),
        ("False force in free space (RMS)", "116 mA", "≈ 19 mA"),
        ("Contact rise time (10–90 %)", "66 ms", "4 ms"),
        ("Held press", "fades to 0 in 1–1.5 s", "holds (≈ ±0.03 A/s)"),
        ("Leader travel into a rigid object", "34°", "≈ 2–3°"),
        ("Current reading at 0.8 A", "−14 % (real ≈ 0.93 A)", "−2 %"),
        ("Rigid vs soft, measured by the follower", "indistinguishable", "9.6–20× apart"),
        ("Worst Pi loop stall", "1–2 s", "tens of ms"),
        ("Force calibration at the tip", "none", "1.044 N/A"))
tbl_shape = s.shapes.add_table(len(data), 3, Inches(0.6), Inches(1.65), Inches(12.13), Inches(0.53 * len(data)))
tbl = tbl_shape.table
for j, wdt in enumerate((5.2, 3.5, 3.43)):
    tbl.columns[j].width = Inches(wdt)
for i, row in enumerate(data):
    tbl.rows[i].height = Inches(0.53)
    for j, val in enumerate(row):
        c = tbl.cell(i, j)
        c.fill.solid()
        c.fill.fore_color.rgb = rgb(INK if i == 0 else (NEUTRAL if i % 2 == 0 else "FFFFFF"))
        c.vertical_anchor = MSO_ANCHOR.MIDDLE
        c.margin_left = c.margin_right = Inches(0.15)
        if i == 0:
            st = dict(size=15, bold=True, color="FFFFFF")
        elif j == 2:
            st = dict(size=15, bold=True, color=GREEN)
        elif j == 1:
            st = dict(size=15, color=RED)
        else:
            st = dict(size=15, color=INK)
        fill_tf(c.text_frame, [P(val, **st)], anchor=MSO_ANCHOR.MIDDLE)
text(s, 0.6, 1.65 + 0.53 * len(data) + 0.15, 12.13, 0.4,
     [P("Before = the original law, or the step just before each fix. Peak force is still capped at ≈ 0.84 N "
        "by the 0.8 A motor rating.", size=12, color=MUTED, italic=True)])

# =========================================================== 13. Method
s = content_slide("A phased plan with written pass criteria", "METHOD", INK,
                  "Phase A characterised the rig before any control changes. Phase B built and checked the "
                  "estimators offline and fixed the firmware. Phase C closed the loop: shadow mode first, "
                  "then live. Phase D is the formal evaluation. Every run is logged in TEST_LOG.md with its "
                  "command, numbers against the threshold, and PASS or FAIL.")
phases = (("A", "Characterise", "plant gain, noise floor, ripple, friction, command latency", "done", GREEN),
          ("B", "Estimators + firmware", "observers offline, current averaging, offsets, force calibration",
           "done", GREEN),
          ("C", "Close the loop", "shadow mode, then live felt force, holds, contact stiffness", "done", GREEN),
          ("D", "Evaluate", "force accuracy, stiffness discrimination, blind rigid vs soft", "next", FOLLOWER))
for i, (letter, name, body, status, col) in enumerate(phases):
    x = 0.6 + i * 3.11
    card(s, x, 1.8, 2.8, 3.0, fill=T_GREEN if status == "done" else T_FOLLOWER, margin=0.2,
         name=f"Phase {letter}",
         paras=[P(letter, size=34, bold=True, color=col, space_after=0),
                P(name, size=18, bold=True, color=INK, space_after=6),
                P(body, size=14, color=INK)])
    pill = card(s, x + 0.2, 4.3, 0.95, 0.32, fill=col, anchor=MSO_ANCHOR.MIDDLE, margin=0.0,
                name=f"Phase {letter} status",
                paras=[P(status.upper(), align=PP_ALIGN.CENTER, size=11, bold=True, color="FFFFFF")])
    pill.adjustments[0] = 0.5
    if i < 3:
        arr = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x + 2.84), Inches(3.1), Inches(0.24),
                                 Inches(0.4))
        arr.fill.solid()
        arr.fill.fore_color.rgb = rgb("B8C2CC")
        arr.line.fill.background()
for i, (hdr, body) in enumerate((
        ("Every run logged", "Command, numbers against the threshold, PASS/FAIL and log paths, in "
                             "TEST_LOG.md."),
        ("Feel decides, blind where possible", "Example: Tafter 400 vs 807 in hidden order; my preference "
                                               "matched the data (2.6× less buzz)."))):
    card(s, 0.6 + i * 6.21, 5.15, 5.92, 1.7, fill=NEUTRAL, anchor=MSO_ANCHOR.MIDDLE, margin=0.22,
         name=f"Method note {i + 1}",
         paras=[P(hdr, size=17, bold=True, color=INK, space_after=4), P(body, size=14, color=INK)])

# =========================================================== 14. Next (dark)
s = prs.slides.add_slide(L_TITLE_ONLY)
dark(s)
t = s.shapes.title
t.left, t.top, t.width, t.height = Inches(0.6), Inches(0.78), Inches(12.13), Inches(0.72)
fill_tf(t.text_frame, [P("Limitations and next steps", size=32, bold=True, color=ON_DARK)],
        anchor=MSO_ANCHOR.TOP, margin=0.0)
tag(s, "NEXT", FOLLOWER)
s.notes_slide.notes_text_frame.text = (
    "The sensing side works: the follower separates rigid from soft by 10 to 20 times. The rendering side "
    "just improved with the leader spring, so the first job is to redo the rigid-versus-soft feel test. "
    "Then force accuracy on the scale, stiffness discrimination, and blind trials.")
number(s)
text(s, 0.6, 1.8, 5.8, 0.5, [P("Limitations", size=22, bold=True, color=ON_DARK)])
text(s, 0.6, 2.45, 5.8, 4.4, [
    P("Peak force ≈ 0.84 N at the tip (0.8 A motor rating)", bullet=True, size=17, color=ON_DARK_MUTED,
      space_after=10),
    P("Friction ≈ 100× the first assumption", bullet=True, size=17, color=ON_DARK_MUTED, space_after=10),
    P("Current noise at 0.8 A: 26–29 mA vs a 25 mA target", bullet=True, size=17, color=ON_DARK_MUTED,
      space_after=10),
    P("Both ESCs occasionally drop off the bus together; cause unknown", bullet=True, size=17,
      color=ON_DARK_MUTED)])
text(s, 7.0, 1.8, 5.73, 0.5, [P("Next steps", size=22, bold=True, color=ON_DARK)])
text(s, 7.0, 2.45, 5.73, 4.4, [
    P("Redo rigid vs soft by feel, now with the leader spring", num=True, size=17, color=ON_DARK,
      bold=True, space_after=10),
    P("Force accuracy on the scale, up to 0.8 A", num=True, size=17, color=ON_DARK_MUTED, space_after=10),
    P("Stiffness discrimination (foam vs rigid)", num=True, size=17, color=ON_DARK_MUTED, space_after=10),
    P("Blind rigid-vs-soft trials: the end goal", num=True, size=17, color=ON_DARK_MUTED)])
card(s, 0.6, 5.15, 12.13, 1.5, fill="2E3B48", anchor=MSO_ANCHOR.MIDDLE, margin=0.3, name="Takeaway",
     paras=[P("Takeaway", size=14, bold=True, color=FOLLOWER, space_after=4),
            P("Sensing works: the follower already separates rigid from soft by 10–20×. Rendering just caught "
              "up with the leader spring, so the feel test is the next experiment.", size=17, color=ON_DARK)])

# =========================================================== B1. Peer coupling
s = content_slide("Backup: on-ESC PD coupling (preliminary)", "BACKUP", MUTED,
                  "The ESC_based firmware can also run a classic spring-damper between the two shaft "
                  "positions on the ESCs themselves, at 1 kHz with peer data under 1 ms old. It reached "
                  "38 times the Pi's stiffness but buzzed at full scale. Not tuned or evaluated; kept as the "
                  "alternative architecture.")
picture(s, os.path.join(ASSETS, "peer.png"), 0.6, 1.6, h=5.3,
        alt="ESC-to-ESC PEER coupling run at Kp 0.01: shaft positions, coupling error and ESC A current with "
            "full-scale oscillation bursts.")
text(s, 9.95, 1.75, 2.78, 5.0, [
    P("Each ESC: spring-damper on the position difference, 1 kHz, peer data < 1 ms old.", size=15,
      space_after=12),
    P("Kp up to 0.01 A/count: ≈ 38× the Pi position mirror (0.000265).", size=15, space_after=12),
    P("Full-scale ±0.8 A buzzing at that stiffness (first 15 s of this run).", size=15, color=RED,
      space_after=12),
    P("Runs from 2026-10-06; not tuned or evaluated.", size=13, color=MUTED, italic=True)])

# =========================================================== B2. Gains
s = content_slide("Backup: operating-point settings", "BACKUP", MUTED,
                  "All set through TELEOP_* environment variables on teleop_test.py; with none set, the script "
                  "behaves exactly like the original. Tafter is sent by can_interface before every START.")
gains = (("Term", "Setting", "Where"),
         ("Felt-force gain K_F · dead band", "1.0 · 20 mA", "leader"),
         ("Leader damping B", "0.35e-5 A/(count/s)", "leader"),
         ("Leader contact spring · max · damping", "2e-3 A/count · 0.4 A · 1e-5", "leader, in contact"),
         ("Follower contact spring · max · ramp", "0.5e-3 A/count · 0.05 A · 0.3 s", "follower, in contact"),
         ("Slipping position coupling · window", "1e-3 A/count · 30 counts", "follower"),
         ("Observers", "20 Hz; fitted follower model", "both"),
         ("Current clamp", "±0.8 A", "both"),
         ("ADC sampling delay (Tafter)", "400 (stock 807)", "both ESCs"))
tbl = s.shapes.add_table(len(gains), 3, Inches(0.6), Inches(1.65), Inches(12.13), Inches(0.5 * len(gains))).table
for j, wdt in enumerate((5.0, 4.4, 2.73)):
    tbl.columns[j].width = Inches(wdt)
for i, row in enumerate(gains):
    tbl.rows[i].height = Inches(0.5)
    for j, val in enumerate(row):
        c = tbl.cell(i, j)
        c.fill.solid()
        c.fill.fore_color.rgb = rgb(INK if i == 0 else (NEUTRAL if i % 2 == 0 else "FFFFFF"))
        c.vertical_anchor = MSO_ANCHOR.MIDDLE
        c.margin_left = c.margin_right = Inches(0.15)
        st = dict(size=14, bold=True, color="FFFFFF") if i == 0 else dict(size=14, color=INK)
        fill_tf(c.text_frame, [P(val, **st)], anchor=MSO_ANCHOR.MIDDLE)

# =========================================================== B3. Phase A
s = content_slide("Backup: Phase A characterisation", "BACKUP", MUTED,
                  "Measured on the rig before any control changes, 2026-10-20. These numbers set the "
                  "observer constants and showed which assumptions were wrong, friction most of all.")
pa = (("Quantity", "Measured", "Consequence"),
      ("Plant gain G", "9.0e6 / 9.7e6 counts/s² per A (±2–3 %), old arm", "re-fit per arm (3D arm: 3.85e6)"),
      ("Friction", "breakaway 0.05–0.09 A, running ≈ 0.04 A", "≈ 100× the assumption; must be modelled"),
      ("Current noise", "30–43 mA std per raw sample", "→ 1 ms averaging in firmware (9.7× lower)"),
      ("Torque ripple", "12/rev ≈ 22 mA, plus 24, 28, 36/rev", "leader calibration at session start"),
      ("Command latency", "50 % of a step 1.47 ms after the send", "fine for 500 Hz"),
      ("Pi loop timing", "p95 2.03 ms, but stalls up to 1 s", "real-time priority + RAM logging"))
tbl = s.shapes.add_table(len(pa), 3, Inches(0.6), Inches(1.65), Inches(12.13), Inches(0.55 * len(pa))).table
for j, wdt in enumerate((2.9, 4.9, 4.33)):
    tbl.columns[j].width = Inches(wdt)
for i, row in enumerate(pa):
    tbl.rows[i].height = Inches(0.55)
    for j, val in enumerate(row):
        c = tbl.cell(i, j)
        c.fill.solid()
        c.fill.fore_color.rgb = rgb(INK if i == 0 else (NEUTRAL if i % 2 == 0 else "FFFFFF"))
        c.vertical_anchor = MSO_ANCHOR.MIDDLE
        c.margin_left = c.margin_right = Inches(0.15)
        st = dict(size=14, bold=True, color="FFFFFF") if i == 0 else dict(size=14, color=INK)
        fill_tf(c.text_frame, [P(val, **st)], anchor=MSO_ANCHOR.MIDDLE)

prs.save(OUT)
print("wrote", OUT, "slides:", len(prs.slides))
