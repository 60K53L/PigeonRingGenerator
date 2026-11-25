# -*- coding: utf-8 -*-
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC_CS = os.path.join(ROOT, "src", "PigeonRingGenerator.cs")
FONTS_DIR = os.path.join(ROOT, "Fonts")
TEMPLATES_DIR = os.path.join(ROOT, "Templates")
ASSETS_DIR = os.path.join(ROOT, "Assets")
DIST_DIR = os.path.join(ROOT, "dist")
os.makedirs(DIST_DIR, exist_ok=True)

OUT_EXE = os.path.join(DIST_DIR, "PigeonRingGenerator.exe")
RSP_PATH = os.path.join(DIST_DIR, "build.rsp")

csc = r"C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
wpf = r"C:\Windows\Microsoft.NET\Framework64\v4.0.30319\WPF"

if not os.path.exists(csc):
    print("HATA: .NET Framework 4.0 csc.exe bulunamadi:", csc)
    sys.exit(1)

fonts = sorted(
    os.path.join(FONTS_DIR, x)
    for x in os.listdir(FONTS_DIR)
    if x.lower().endswith((".ttf", ".otf"))
)
templates = sorted(
    os.path.join(TEMPLATES_DIR, x)
    for x in os.listdir(TEMPLATES_DIR)
    if x.lower().endswith(".3mf.bak")
)

lines = [
    "/nologo",
    "/optimize+",
    "/target:winexe",
    f'/win32icon:"{os.path.join(ASSETS_DIR, "pigeon_app_2.ico")}"',
    f'/win32manifest:"{os.path.join(ASSETS_DIR, "unaware.manifest")}"',
    "/r:System.Drawing.dll",
    "/r:System.Windows.Forms.dll",
    "/r:System.Xml.dll",
    "/r:System.IO.Compression.dll",
    "/r:System.IO.Compression.FileSystem.dll",
    f'/r:"{os.path.join(wpf, "PresentationCore.dll")}"',
    f'/r:"{os.path.join(wpf, "WindowsBase.dll")}"',
]

for fp in fonts + templates:
    lines.append(f'/resource:"{fp}",{os.path.basename(fp)}')

lines.append(f'/out:"{OUT_EXE}"')
lines.append(f'"{SRC_CS}"')

with open(RSP_PATH, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

r = subprocess.run([csc, "@" + RSP_PATH], capture_output=True, text=True, errors="replace")
if r.stdout:
    print(r.stdout)
if r.stderr:
    print(r.stderr)

if r.returncode != 0:
    print("DERLEME HATASI:", r.returncode)
    sys.exit(r.returncode)

print(f"BASARILI: {OUT_EXE} ({os.path.getsize(OUT_EXE)} bayt)")
print(f"Gomulu varliklar: {len(fonts)} font, {len(templates)} sablon")
