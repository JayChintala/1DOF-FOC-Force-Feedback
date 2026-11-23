# -*- coding: utf-8 -*-
"""
Updates the deck for the disturbance-observer P2P (2026-11-22): rewrites the old
PEER slides (Peer-to-Peer Coupling, Peer-to-Peer: Measured, Two Approaches, the
P2P line of Remaining Work) and adds the P2P-vs-teleop comparison to
"Force-Aware Telepresence for Robot Learning.pptx", after "Two Approaches":

  P2P vs Teleop: Free Space      felt inertia and friction at the handle
  P2P vs Teleop: Taps and Holds  tap force rise, held-press drift
  P2P vs Teleop: Rigid vs Soft   handle travel to 0.7 N, stiffness at A
  P2P vs Teleop: Scorecard       table

Every chart is a native (editable) PowerPoint chart, styled like the deck's
existing "Results: Force Readings" charts; teleop is always blue and P2P
always orange. Numbers: Peer_to_Peer/testing.md ("P2P vs the old teleop
controller"), computed by ESC_based/software/p2p_compare.py and p2p_feel.py
from the logs in Peer_to_Peer/data/.

Idempotent: always rebuilds from the untouched backup.
Needs python-pptx.   python3 add_p2p_vs_teleop_slides.py
"""
import copy
import os
import shutil

from pptx import Presentation
from pptx.chart.data import CategoryChartData
from pptx.dml.color import RGBColor
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION, XL_LEGEND_POSITION
from pptx.util import Inches, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
DECK = os.path.join(HERE, "Force-Aware Telepresence for Robot Learning.pptx")
BACKUP = os.path.join(
    HERE, "Force-Aware Telepresence for Robot Learning (before 2026-11-22 update).pptx")

# Geometry and style copied from the deck (Two Approaches / Force Readings).
TITLE_BOX = (Inches(0.96), Inches(0.38), Inches(11.50), Inches(1.45))
CHART_L = (Inches(0.66), Inches(1.75), Inches(5.96), Inches(4.21))
CHART_R = (Inches(6.73), Inches(1.75), Inches(5.96), Inches(4.21))
CAP_X, CAP_Y, CAP_W, CAP_H = Inches(0.87), Inches(6.12), Inches(11.59), Inches(1.15)
CAPTION_PT, LABEL_PT, CHART_TITLE_PT = 12, 12, 14
GREY = RGBColor(0x52, 0x51, 0x4E)
GRID = RGBColor(0xE4, 0xE3, 0xDF)
TELEOP = RGBColor(0x2A, 0x78, 0xD6)        # deck blue
TELEOP_LIGHT = RGBColor(0x9C, 0xC2, 0xEE)  # teleop, superseded setting
P2P = RGBColor(0xEB, 0x68, 0x34)           # deck orange
RIGID = RGBColor(0x52, 0x51, 0x4E)
SOFT = RGBColor(0x1B, 0xAF, 0x7A)          # deck green

B_ROTOR = 112.7   # nA per count/s^2: motor B's rotor alone (1/G_B)


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


def base_slide(prs, title, caption, notes):
    layout = next(l for l in prs.slide_masters[0].slide_layouts if l.name == "Title Only")
    slide = prs.slides.add_slide(layout)
    t = slide.shapes.title
    t.left, t.top, t.width, t.height = TITLE_BOX
    t.text_frame.text = title
    box = slide.shapes.add_textbox(CAP_X, CAP_Y, CAP_W, CAP_H)
    box.name = "Caption"
    box.text_frame.word_wrap = True
    run = box.text_frame.paragraphs[0].add_run()
    run.text = caption
    run.font.size = Pt(CAPTION_PT)
    run.font.color.rgb = GREY
    slide.notes_slide.notes_text_frame.text = notes
    return slide


def style_axes(chart, number_format):
    for ax in (chart.category_axis, chart.value_axis):
        ax.tick_labels.font.size = Pt(LABEL_PT)
        ax.tick_labels.font.color.rgb = GREY
        ax.format.line.color.rgb = GRID
    chart.value_axis.has_major_gridlines = True
    chart.value_axis.major_gridlines.format.line.color.rgb = GRID
    chart.value_axis.tick_labels.number_format = number_format
    chart.value_axis.tick_labels.number_format_is_linked = False
    chart.category_axis.has_major_gridlines = False


def column_chart(slide, box, title, categories, series, number_format,
                 point_colors=None, series_colors=None, name=None):
    """series: [(name, values)]. One series -> colour by point (controller);
    several -> colour by series, with a legend."""
    cd = CategoryChartData()
    cd.categories = categories
    for sname, vals in series:
        cd.add_series(sname, vals, number_format=number_format)
    gf = slide.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, *box, cd)
    gf.name = name or title
    ch = gf.chart
    ch.has_title = True
    ch.chart_title.text_frame.text = title
    p = ch.chart_title.text_frame.paragraphs[0]
    p.runs[0].font.size = Pt(CHART_TITLE_PT)
    p.runs[0].font.bold = True
    p.runs[0].font.color.rgb = GREY
    style_axes(ch, number_format)
    plot = ch.plots[0]
    plot.gap_width = 70
    plot.overlap = -10 if len(series) > 1 else 0
    plot.has_data_labels = True
    dl = plot.data_labels
    dl.number_format = number_format
    dl.number_format_is_linked = False
    dl.position = XL_LABEL_POSITION.OUTSIDE_END
    dl.font.size = Pt(LABEL_PT)
    dl.font.color.rgb = GREY
    if len(series) == 1:
        ch.has_legend = False
        for pt, col in zip(plot.series[0].points, point_colors):
            pt.format.fill.solid()
            pt.format.fill.fore_color.rgb = col
    else:
        ch.has_legend = True
        ch.legend.position = XL_LEGEND_POSITION.TOP
        ch.legend.include_in_layout = False
        ch.legend.font.size = Pt(LABEL_PT)
        ch.legend.font.color.rgb = GREY
        for s, col in zip(plot.series, series_colors):
            s.format.fill.solid()
            s.format.fill.fore_color.rgb = col
    return ch


# ----------------------------------------------------------------- slides ----
def slide_free(prs):
    s = base_slide(
        prs, "P2P vs Teleop: Free Space",
        "Same hand-force estimator run on both controllers' logs, fit as force = m·a + friction, at "
        "matched effort (C3 vs P2P: |a| rms 0.12 vs 0.13 M counts/s²). Teleop's handle feels lighter "
        "than its own rotor; P2P feels 1.9× it — exactly what Kf 50 % predicts (2.0×). Kf was halved "
        "to calm a ≈ 20 Hz wobble, so this is the price of that.",
        "Data: Peer_to_Peer/testing.md, 2026-11-22. Script: ESC_based/software/p2p_feel.py.\n\n"
        "Why not compare motor currents: teleop's handle current excludes the handle's own inertia "
        "and friction (the hand pays those passively), while P2P's handle current also cancels "
        "them. Replaying one observer (80 rad/s, G_B, each run's own ripple calibration) on the "
        "measured current and encoder of both is the like-for-like comparison.\n\n"
        "Teleop runs: C3 operating point (2026-10-25) and runs 22, 27, 28 (current operating point; "
        "gentler motion). Run 21's old '26.2 mA free-space RMS' is not used: the handle moved only "
        "~100 counts in that run.\n\n"
        "Hand-force RMS at matched effort: teleop C3 19.0 mA, P2P 52.4 mA.")
    cats = ["Teleop C3", "Teleop 22", "Teleop 27", "Teleop 28", "P2P"]
    cols = [TELEOP] * 4 + [P2P]
    inertia = [69, 47, 68, 56, 214]
    column_chart(s, CHART_L, "Inertia felt at the handle (× B's rotor)", cats,
                 [("inertia", [round(v / B_ROTOR, 2) for v in inertia])], "0.0\"×\"", cols)
    column_chart(s, CHART_R, "Friction felt at the handle (mA)", cats,
                 [("friction", [17.0, 5.9, 4.8, 3.8, 28.2])], "0", cols)


def slide_taps_holds(prs):
    s = base_slide(
        prs, "P2P vs Teleop: Taps and Holds",
        "Taps: teleop's force reaches the hand in 4 ms (median; 75th pct 34 ms), P2P in 41 ms "
        "(75th pct 62 ms) — bounded by its 80 rad/s observer, also lowered for the wobble. Holds: "
        "both stay flat, but teleop needed an added contact spring to stop presses fading; P2P's "
        "observers hold force by design, with the two sides matching within ≈ 3 %.",
        "Taps: 33 P2P taps on the rigid stop, scored with the same impact detector as teleop's "
        "(analyze_teleop_baseline.impacts): the follower stops sharply while the handle is still "
        "moving; rise = 10-90 % of the handle current after impact. Teleop's 4 ms is the C3 "
        "taps-only run; its fast rise comes through an unfiltered inertia term at impact.\n\n"
        "Holds: plateau slope of |force| from 0.25 s after contact to 0.1 s before release "
        "(hold_eval.py definition). Teleop without the contact spring: -0.32/-0.52 A/s (REF "
        "2026-10-26); with it: -0.03/+0.02 (HOLD3). P2P: median |slope| 0.024 A/s over 15 holds on "
        "rigid and sponge. Most P2P holds were at the 0.8 A ceiling; the one clean sub-ceiling hold "
        "(5.9 s at 0.63 N on the sponge) drifted +0.023 A/s.")
    column_chart(s, CHART_L, "Tap force rise, 10–90 % (ms, median)", ["Teleop", "P2P"],
                 [("rise", [4.0, 40.6])], "0", [TELEOP, P2P])
    column_chart(s, CHART_R, "Held-press force drift (A/s, magnitude)",
                 ["Teleop, no contact spring", "Teleop, contact spring", "P2P"],
                 [("drift", [0.42, 0.025, 0.024])], "0.000", [TELEOP_LIGHT, TELEOP, P2P])


def slide_contact(prs):
    s = base_slide(
        prs, "P2P vs Teleop: Rigid vs Soft",
        "At the operator's handle, P2P is the stiffest yet on rigid contact (21 counts to 0.7 N, vs "
        "50 with teleop's leader spring and 382 without), and rigid vs soft now shows in the hand: "
        "3.1× more travel on the soft object, vs 1.24× for teleop. Robot-side stiffness came out "
        "lower in P2P on rigid (6 counts of travel, near the encoder limit; cause unconfirmed).",
        "Handle travel: from contact (0.15 N) to 0.7 N, per press, median. Teleop without the "
        "leader spring: runs _14 (rigid) and _16 (sponge), 2026-10-29. With the leader spring "
        "(current teleop): run _18, rigid only - no soft run was recorded with it.\n\n"
        "Stiffness at A: stiffness_eval.py definition (quasi-static frames between 0.3 and 0.7 N). "
        "Teleop rigid 1.62 N/mm is a lower bound (3 counts of travel). Teleop soft = the "
        "2026-10-29 sponge (0.08 N/mm). The P2P soft run measured 0.17 N/mm, which matches the "
        "squishy foam from 2026-10-29 rather than that sponge - which object was used is "
        "unconfirmed.\n\n"
        "P2P presses mostly reached the 0.8 A ceiling (0.84 N), as teleop's did.")
    column_chart(s, CHART_L, "Handle travel, contact → 0.7 N (counts)",
                 ["Teleop, no leader spring", "Teleop, leader spring", "P2P"],
                 [("Rigid", [382, 50, 21]), ("Soft", [473, None, 65])], "0",
                 series_colors=[RIGID, SOFT])
    column_chart(s, CHART_R, "Stiffness measured at the robot side (N/mm)",
                 ["Teleop", "P2P"],
                 [("Rigid", [1.62, 0.53]), ("Soft", [0.08, 0.17])], "0.00",
                 series_colors=[RIGID, SOFT])


SCORE = [
    ["", "Teleop (leader–follower)", "P2P (disturbance observers)"],
    ["Inertia felt, free space", "0.4–0.6× B's rotor", "1.9× — heavier"],
    ["Friction felt, free space", "4–17 mA", "28 mA"],
    ["Tap force rise (median)", "4 ms", "41 ms"],
    ["Held presses", "flat, with an added contact spring", "flat by design (0.024 A/s)"],
    ["Force matching in holds", "≤ 5 % against a scale", "≈ 3 %, action vs reaction"],
    ["Handle travel to 0.7 N, rigid", "50 counts (leader spring)", "21 counts"],
    ["Rigid vs soft at the hand", "1.24× travel (no spring)", "3.1× travel"],
]


def slide_scorecard(prs, model_table):
    s = base_slide(
        prs, "P2P vs Teleop: Scorecard",
        "P2P wins on contact — a stiffer handle, rigid vs soft you can feel, holds that don't fade "
        "without extra machinery. Teleop wins in free space and on taps. Both P2P losses trace to "
        "gains backed off for the ≈ 20 Hz wobble, so finding that wobble is the next step.",
        "All numbers: Peer_to_Peer/testing.md, 2026-11-22, with the raw logs in "
        "Peer_to_Peer/data/2026-11-22_p2p_vs_teleop_logs.zip.\n\n"
        "P2P settings: wn 20 Hz, zeta 1, Kf 50 %, observer 80 rad/s, in-session ripple "
        "calibration, Iq max 0.8 A. Roles as in teleop: operator on B, A touches the environment.\n\n"
        "'Force matching' is not the same test on both sides: teleop's 5 % was against a scale; "
        "P2P's 3 % is |f_A + f_B| / |f_A| during holds.")
    s.shapes._spTree.remove(s.shapes[-1]._element)     # caption re-added below the table
    widths = [Inches(3.70), Inches(3.90), Inches(3.90)]
    row_h = Inches(0.52)
    gf = s.shapes.add_table(len(SCORE), 3, Inches(0.96), Inches(1.80),
                            sum(widths, Inches(0)), row_h * len(SCORE))
    gf.name = "Scorecard table"
    tbl = gf.table
    ns = "{http://schemas.openxmlformats.org/drawingml/2006/main}tblPr"
    tbl._tbl.replace(tbl._tbl.find(ns), copy.deepcopy(model_table._tbl.find(ns)))
    for c, w in zip(tbl.columns, widths):
        c.width = w
    for r in tbl.rows:
        r.height = row_h
    for ri, row in enumerate(SCORE):
        for ci, text in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text_frame.word_wrap = True
            run = cell.text_frame.paragraphs[0].add_run()
            run.text = text
            run.font.size = Pt(14)
            run.font.bold = (ri == 0 or ci == 0)
    box = s.shapes.add_textbox(CAP_X, Inches(6.34), CAP_W, Inches(0.95))
    box.name = "Caption"
    box.text_frame.word_wrap = True
    run = box.text_frame.paragraphs[0].add_run()
    run.text = ("P2P wins on contact — a stiffer handle, rigid vs soft you can feel, holds that "
                "don't fade without extra machinery. Teleop wins in free space and on taps. Both "
                "P2P losses trace to gains backed off for the ≈ 20 Hz wobble.")
    run.font.size = Pt(CAPTION_PT)
    run.font.color.rgb = GREY


# ------------------------------------- edits: old PEER slides -> DOB P2P ----
A_COL = RGBColor(0x2A, 0x6F, 0xDB)   # motor A / B colours of diagram_p2p_dob.png
B_COL = RGBColor(0xD9, 0x68, 0x2C)


def replace_caption(slide, text, notes):
    box = [sh for sh in slide.shapes if sh.has_text_frame and sh.name.startswith("TextBox")][0]
    p0 = box.text_frame.paragraphs[0]
    p0.runs[0].text = text
    for extra in p0.runs[1:]:
        extra._r.getparent().remove(extra._r)
    for extra in box.text_frame.paragraphs[1:]:
        extra._p.getparent().remove(extra._p)
    slide.notes_slide.notes_text_frame.text = notes


def edit_peer_coupling(prs):
    s = find(prs, "Peer-to-Peer Coupling")
    old = [sh for sh in s.shapes if sh.shape_type == 13][0]          # the PEER diagram
    geo = (old.left, old.top, old.width, old.height)
    old._element.getparent().remove(old._element)
    pic = s.shapes.add_picture(os.path.join(HERE, "diagram_p2p_dob.png"), *geo)
    pic.name = "Peer-to-Peer Coupling figure"
    replace_caption(
        s,
        "No leader, no follower: both ESCs run the same law at 1 kHz. Each runs a disturbance "
        "observer for its own motor and for the other's (from its telemetry), which cancels "
        "friction, cogging and gravity locally and leaves each side's external force. A force "
        "channel keeps those forces equal and opposite; a position channel keeps the shafts aligned.",
        "Keio-style acceleration-based 4-channel bilateral control (Ohnishi, Katsura, Shimono). "
        "Firmware CHANGE 9 (P2P mode) and CHANGE 10 (in-session ripple/cogging calibration), "
        "2026-11-18/19.\n\n"
        "This replaced PEER mode, a plain spring-damper between the shafts, which felt springy: "
        "a push had to stretch the spring before the other shaft moved, it saturated at 114 counts, "
        "and stiffer gains hit a 57 Hz buzz. Here a push drives the common (force) mode - both "
        "shafts accelerate together - and the observers' integral action drives the steady gap to "
        "zero.\n\n"
        "Kf sets the inertia the hand feels (apparent mass 2/Kf). Gains: wn 20 Hz, zeta 1, Kf 50 %, "
        "observer 80 rad/s. Details: Peer_to_Peer/README.md and references there.")


def _xy_chart(slide, box, title, t, series, y_title, number_format):
    from pptx.chart.data import XyChartData
    cd = XyChartData()
    for name, ys in series:
        sr = cd.add_series(name)
        for x, y in zip(t, ys):
            sr.add_data_point(x, y)
    gf = slide.shapes.add_chart(XL_CHART_TYPE.XY_SCATTER_LINES_NO_MARKERS, *box, cd)
    gf.name = title
    ch = gf.chart
    ch.has_title = True
    ch.chart_title.text_frame.text = title
    r = ch.chart_title.text_frame.paragraphs[0].runs[0]
    r.font.size, r.font.bold, r.font.color.rgb = Pt(CHART_TITLE_PT), True, GREY
    style_axes(ch, number_format)
    ch.category_axis.tick_labels.number_format = "0"
    ch.category_axis.tick_labels.number_format_is_linked = False
    from pptx.enum.chart import XL_TICK_LABEL_POSITION
    ch.category_axis.tick_label_position = XL_TICK_LABEL_POSITION.LOW   # below the plot, not on y = 0
    ch.category_axis.minimum_scale = 0
    ch.category_axis.maximum_scale = round(max(t))
    ch.category_axis.has_title = True
    ch.category_axis.axis_title.text_frame.text = "time (s)"
    ch.value_axis.has_title = True
    ch.value_axis.axis_title.text_frame.text = y_title
    for ax in (ch.category_axis, ch.value_axis):
        rr = ax.axis_title.text_frame.paragraphs[0].runs[0]
        rr.font.size, rr.font.bold, rr.font.color.rgb = Pt(LABEL_PT), False, GREY
    ch.has_legend = True
    ch.legend.position = XL_LEGEND_POSITION.TOP
    ch.legend.include_in_layout = False
    ch.legend.font.size = Pt(LABEL_PT)
    ch.legend.font.color.rgb = GREY
    from pptx.enum.dml import MSO_LINE_DASH_STYLE
    for sr, col, dash in zip(ch.plots[0].series, (A_COL, B_COL), (None, MSO_LINE_DASH_STYLE.DASH)):
        sr.format.line.color.rgb = col
        sr.format.line.width = Pt(2.25)
        if dash:
            sr.format.line.dash_style = dash
        sr.smooth = False
    return ch


def edit_peer_measured(prs):
    import json
    d = json.load(open(os.path.join(HERE, "p2p_measured_window.json")))
    s = find(prs, "Peer-to-Peer: Measured")
    old = [sh for sh in s.shapes if sh.shape_type == 13][0]
    old._element.getparent().remove(old._element)
    _xy_chart(s, CHART_L, "Free motion: both shafts move together", d["free_t_s"],
              [("Motor A", d["free_pos_A_deg"]), ("Motor B", d["free_pos_B_deg"])],
              "angle (degrees)", "0")
    _xy_chart(s, CHART_R, "Rigid presses: equal and opposite force", d["t_s"],
              [("Force on A (object)", d["f_A_N"]), ("Force on B (hand), flipped", d["minus_f_B_N"])],
              "force (N)", "0.0")
    st, fs, cg = d["stats"], d["free_stats"], d["contact_gap"]
    replace_caption(
        s,
        f"Left: 6 s of free motion — the shafts stay within {fs['gap_p95_counts']:.0f} counts "
        f"(95th pct, ≈ {fs['gap_p95_counts'] * 0.09:.1f}°). Right: rigid presses — the object's force "
        f"on A and the hand's on B match (corr {st['corr_fA_minus_fB']:.3f}, "
        f"{st['mismatch_in_contact_pct']:.0f} % apart in contact). Once a press reaches the 0.8 A "
        f"ceiling (0.84 N), A can't push harder and the handle slips past it.",
        "Data: Peer_to_Peer/data/2026-11-22_p2p_vs_teleop_logs.zip; window extraction in "
        "p2p_measured_window.json (" + d["free_source"] + "; " + d["source"] + "). Forces are the "
        "firmware's observer outputs, replayed from the logged currents and encoders "
        "(p2p_compare.py).\n\n"
        f"Gap in contact: below the current ceiling median {cg['below_ceiling_p50']} counts (95th "
        f"pct {cg['below_ceiling_p95']}); at or just after the ceiling median {cg['at_ceiling_p50']} "
        f"(95th pct {cg['at_ceiling_p95']}) - the handle slips past the arm because the arm is "
        "already pushing as hard as 0.8 A allows. Teleop has the same ceiling (its leader spring "
        "is capped too).\n\n"
        "Open: a ~20 Hz common-mode wobble hands-off (both shafts together), probably structural. "
        "It is why Kf and the observer cutoff are backed off.")


def edit_two_approaches(prs):
    s = find(prs, "Two Approaches")
    tbl = [sh for sh in s.shapes if sh.has_table][0].table
    new = {
        "The hand feels": ("estimated contact force", "the other side's force, via observers"),
        "Free motion": ("light — 0.4–0.6× B's rotor", "1.9× B's rotor (Kf 50 %)"),
        "Friction + gravity": ("cancelled by a fitted model", "cancelled per session (calibration sweep)"),
        "Rigid vs soft": ("20× at the follower, 1.2× at the hand", "3.2× at A, 3.1× at the hand"),
        "Hands off": ("stable", "stable; ≈ 20 Hz wobble"),
    }
    for ri in range(len(tbl.rows)):
        key = tbl.cell(ri, 0).text.strip()
        if key in new:
            for ci, text in zip((1, 2), new[key]):
                p0 = tbl.cell(ri, ci).text_frame.paragraphs[0]
                p0.runs[0].text = text
                for extra in p0.runs[1:]:
                    extra._r.getparent().remove(extra._r)
    replace_caption(
        s,
        "Two architectures, kept side by side. Leader–follower is lighter in free space because the "
        "follower is powered and never loads the hand; peer-to-peer gives honest physics — a stiffer, "
        "more readable contact with no designated master. Measured head to head on the next slides.",
        "Leader-follower: the Pi runs the loop at 500 Hz; the follower tracks the leader, and the "
        "follower's estimated contact force is played back on the leader, with a contact spring on "
        "each side. Peer-to-peer: both ESCs at 1 kHz, disturbance observers, force and position "
        "channels (CHANGE 9/10).\n\n"
        "Rigid vs soft: leader-follower measures 20x apart at the follower but only 1.24x in leader "
        "travel (2026-10-29, before the leader spring). P2P: 3.2x at A and 3.1x in handle travel "
        "(2026-11-22). The soft object in the P2P run may have been the squishy foam rather than "
        "the sponge.\n\n"
        "Friction + gravity in P2P: a hands-off calibration sweep after every START fits each "
        "motor's ripple, cogging, Coulomb friction and (on arm A) gravity; the firmware cancels "
        "them.")


def edit_remaining_work(prs):
    s = find(prs, "Remaining Work")
    body = [sh for sh in s.shapes
            if sh.has_text_frame and sh.name.startswith("Content Placeholder")][0]
    for para in body.text_frame.paragraphs:
        if para.text.startswith("Peer-to-peer"):
            para.runs[0].text = ("Peer-to-peer: find the ≈ 20 Hz wobble so the force gain and "
                                 "observer can go back up — lighter free space, crisper taps")
            for extra in para.runs[1:]:
                extra._r.getparent().remove(extra._r)


def main():
    if not os.path.exists(BACKUP):
        shutil.copy2(DECK, BACKUP)
        print("backed up ->", os.path.basename(BACKUP))
    shutil.copy2(BACKUP, DECK)          # always start pristine

    prs = Presentation(DECK)
    edit_peer_coupling(prs);  print("edited  Peer-to-Peer Coupling")
    edit_peer_measured(prs);  print("edited  Peer-to-Peer: Measured")
    edit_two_approaches(prs); print("edited  Two Approaches")
    edit_remaining_work(prs); print("edited  Remaining Work")
    model_table = [sh for sh in find(prs, "Two Approaches").shapes if sh.has_table][0].table
    plan = [
        ("Two Approaches", lambda: slide_free(prs)),
        ("P2P vs Teleop: Free Space", lambda: slide_taps_holds(prs)),
        ("P2P vs Teleop: Taps and Holds", lambda: slide_contact(prs)),
        ("P2P vs Teleop: Rigid vs Soft", lambda: slide_scorecard(prs, model_table)),
    ]
    lst = prs.slides._sldIdLst
    for anchor, make in plan:
        make()
        slide = prs.slides[-1]
        idx = [first_line(s) for s in prs.slides].index(anchor)
        el = list(lst)[-1]
        lst.remove(el)
        lst.insert(idx + 1, el)
        print("inserted %-32r after %-32r -> position %d" % (first_line(slide), anchor, idx + 2))
    prs.save(DECK)
    print("saved %s (%d slides)" % (os.path.basename(DECK), len(list(lst))))


if __name__ == "__main__":
    main()
