# PigeonRingGenerator

Güvercin bilezik / klips (9 mm tek parça gövde) için silindirik kabartma yazılı çok tablalı `.3mf` proje üretici.

## Özellikler
- **Tek Parça 9 mm Gövde (`BOS_KLIPS_9MM.3mf.bak`):** Gövde kesilmeden tek parça kalır; üst ve alt yazılar ilgili 4.5 mm yarıların tam ortasına (`+2.25 mm` / `-2.25 mm`) hizalanır.
- **Çoklu Mod:** İsimli Klips, Marka Tipi ve Sıralı Numara üretimi.
- **34 Gömülü Font & 6 Şablon:** Tek `.exe` olarak taşınabilir; ek dosya gerekmez.
- **Snapmaker U1 / OrcaSlicer Uyumlu:** Proje bazlı izole filament profilleri (`Generic PLA / PETG`), `classic` duvar üreticisi ve çok renkli baskı desteği.

## Klasör Yapısı
- `src/PigeonRingGenerator.cs` — Ana uygulama kaynak kodu (C# / WinForms + WPF Geometry)
- `Fonts/` — Gömülü 34 adet `.ttf` / `.otf` font dosyası
- `Templates/` — Gömülü `.3mf.bak` şablon dosyaları
- `Assets/` — Uygulama ikonu ve DPI manifest dosyası
- `build.py` / `build.bat` — Tek tıkla taşınabilir `dist/PigeonRingGenerator.exe` derleme betiği

## Derleme (Windows)
Gereksinimler:
- Windows (.NET Framework 4.x `csc.exe`, Windows ile yerleşik gelir)
- Python 3

Derlemek için:
```bat
build.bat
```
veya:
```bat
python build.py
```
Derlenen çalıştırılabilir dosya `dist\PigeonRingGenerator.exe` konumunda oluşur. Hazır derlenmiş `.exe` için GitHub **Releases** bölümüne bakabilirsiniz.
