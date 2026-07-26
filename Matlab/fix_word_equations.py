from copy import deepcopy
from pathlib import Path
from shutil import copyfile

from docx import Document
from docx.oxml import OxmlElement
from docx.text.paragraph import Paragraph


DOCX_PATH = Path("Method_Section_CN.docx")
BACKUP_PATH = Path("Method_Section_CN_before_equations.docx")


FORMULA_REPLACEMENTS = {
    "M = cam2world(m, ocam_model),    M = [Mx, My, Mz]^T": [
        "M = cam2world(m, ocam_model),    M = [M_x, M_y, M_z]^T",
    ],
    "P_C = H P_L,    H = R_C [ r1  r2  t ],    t = [0, 0, las_dist]^T": [
        "P_C = H P_L,    H = R_C [r_1  r_2  t],    t = [0, 0, las_dist]^T",
    ],
    "λ M = H [X, Y, 1]^T": [
        "λM = H[X, Y, 1]^T",
    ],
    "a1 X + b1 Y + c1 = 0\na2 X + b2 Y + c2 = 0\nY = (a2 c1 - a1 c2) / (a1 b2 - a2 b1),    X = (-c1 - b1 Y) / a1": [
        "a_1X + b_1Y + c_1 = 0",
        "a_2X + b_2Y + c_2 = 0",
        "Y = (a_2c_1 - a_1c_2)/(a_1b_2 - a_2b_1),    X = (-c_1 - b_1Y)/a_1",
    ],
    "P_W = R_z(-CVsyst_rot) [X, Y, 1]^T + T_CV": [
        "P_W = R_z(-CVsyst_rot)[X, Y, 1]^T + T_CV",
    ],
    "v_ref = direction · v_target\n||v_ref(k) - v_ref(k-1)|| <= min(MAX_CHASSIS_ACCEL · dt, MAX_CHASSIS_REF_STEP)": [
        "v_ref = direction · v_target",
        "||v_ref(k) - v_ref(k-1)|| <= min(MAX_CHASSIS_ACCEL · dt, MAX_CHASSIS_REF_STEP)",
    ],
    "v0 = vy - vx + kω wz\nv1 = vy + vx - kω wz\nv2 = vy - vx - kω wz\nv3 = vy + vx + kω wz": [
        "v_0 = v_y - v_x + k_ωw_z",
        "v_1 = v_y + v_x - k_ωw_z",
        "v_2 = v_y - v_x - k_ωw_z",
        "v_3 = v_y + v_x + k_ωw_z",
    ],
    "PWM_ff = v_ref / MOTOR_GAIN": [
        "PWM_ff = v_ref/MOTOR_GAIN",
    ],
    "e = r - y,    ec = (e(k) - e(k-1)) / dt": [
        "e = r - y,    ec = (e(k) - e(k-1))/dt",
    ],
    "e_level = min(|e| / E_MAX, 1)\nec_level = min(|ec| / EC_MAX, 1)\ng = 0.5 · (1 + tanh(e · ec / 80000))": [
        "e_level = min(|e|/E_MAX, 1)",
        "ec_level = min(|ec|/EC_MAX, 1)",
        "g = 0.5 · (1 + tanh(e · ec/80000))",
    ],
    "Kp = 0.022 + 0.026 e_level + 0.008 g\nKi = 0.0010 + 0.0060 (1 - e_level)^2\nKd = 0.0020 + 0.0050 ec_level + 0.0025 (1 - e_level)": [
        "K_p = 0.022 + 0.026e_level + 0.008g",
        "K_i = 0.0010 + 0.0060(1 - e_level)^2",
        "K_d = 0.0020 + 0.0050ec_level + 0.0025(1 - e_level)",
    ],
    "PWM_PID = Kp e + Ki ∫e dt + Kd d(e)/dt": [
        "PWM_PID = K_pe + K_i∫e dt + K_d d(e)/dt",
    ],
    "PWM_cmd = sat(PWM_ff + PWM_PID, -PWM_max, PWM_max)": [
        "PWM_cmd = sat(PWM_ff + PWM_PID, -PWM_max, PWM_max)",
    ],
}


def clear_paragraph_content(paragraph):
    p = paragraph._p
    p_pr = p.pPr
    for child in list(p):
        if child is not p_pr:
            p.remove(child)


def add_math_to_paragraph(paragraph, text):
    p = paragraph._p
    math_para = OxmlElement("m:oMathPara")
    math = OxmlElement("m:oMath")
    run = OxmlElement("m:r")
    text_el = OxmlElement("m:t")
    text_el.text = text
    run.append(text_el)
    math.append(run)
    math_para.append(math)
    p.append(math_para)


def insert_equation_after(paragraph, text):
    new_p = OxmlElement("w:p")
    if paragraph._p.pPr is not None:
        new_p.append(deepcopy(paragraph._p.pPr))
    paragraph._p.addnext(new_p)
    new_para = Paragraph(new_p, paragraph._parent)
    add_math_to_paragraph(new_para, text)
    return new_para


def replace_with_equations(paragraph, equations):
    clear_paragraph_content(paragraph)
    add_math_to_paragraph(paragraph, equations[0])
    current = paragraph
    for equation in equations[1:]:
        current = insert_equation_after(current, equation)


def main():
    if not DOCX_PATH.exists():
        raise FileNotFoundError(DOCX_PATH)
    if not BACKUP_PATH.exists():
        copyfile(DOCX_PATH, BACKUP_PATH)

    doc = Document(DOCX_PATH)
    replaced = 0
    for paragraph in list(doc.paragraphs):
        equations = FORMULA_REPLACEMENTS.get(paragraph.text)
        if equations:
            replace_with_equations(paragraph, equations)
            replaced += 1

    if replaced != len(FORMULA_REPLACEMENTS):
        raise RuntimeError(
            f"Expected {len(FORMULA_REPLACEMENTS)} formula paragraphs, replaced {replaced}."
        )

    doc.save(DOCX_PATH)
    print(f"Replaced {replaced} formula paragraphs with Word OMML equations.")
    print(f"Backup: {BACKUP_PATH}")


if __name__ == "__main__":
    main()
