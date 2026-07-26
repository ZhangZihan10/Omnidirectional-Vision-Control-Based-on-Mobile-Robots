from pathlib import Path
import shutil
import zipfile


DOCX_PATH = Path("Method_Section_CN.docx")
TMP_PATH = Path("Method_Section_CN_equation_spacing.tmp.docx")

REPLACEMENTS = {
    "k_ωw_z": "k_ω w_z",
    "v_ref/MOTOR_GAIN": "v_ref / MOTOR_GAIN",
    "(e(k) - e(k-1))/dt": "(e(k) - e(k-1)) / dt",
    "|e|/E_MAX": "|e| / E_MAX",
    "|ec|/EC_MAX": "|ec| / EC_MAX",
    "e · ec/80000": "e · ec / 80000",
    "0.026e_level": "0.026 e_level",
    "0.008g": "0.008 g",
    "0.0060(1 - e_level)^2": "0.0060 (1 - e_level)^2",
    "0.0050ec_level": "0.0050 ec_level",
    "K_pe": "K_p e",
    "K_i∫e dt": "K_i ∫ e dt",
}


def main():
    with zipfile.ZipFile(DOCX_PATH, "r") as zin, zipfile.ZipFile(
        TMP_PATH, "w", zipfile.ZIP_DEFLATED
    ) as zout:
        for info in zin.infolist():
            data = zin.read(info.filename)
            if info.filename == "word/document.xml":
                text = data.decode("utf-8")
                for old, new in REPLACEMENTS.items():
                    text = text.replace(old, new)
                data = text.encode("utf-8")
            zout.writestr(info, data)
    shutil.move(TMP_PATH, DOCX_PATH)
    print("Polished equation spacing in OMML math text.")


if __name__ == "__main__":
    main()
