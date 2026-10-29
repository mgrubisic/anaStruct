# MEMORY — stanje projekta i nastavak rada

> Zapis stanja za brzo hvatanje konteksta u sljedećoj sesiji. Počni s `README.md` (ulazna točka,
> workflow, arhitektura), detalji i izvodi su u `SPOZNAJE.md`, upute za korištenje u `SKILLS.md`.

**Stanje (2026-10-29):** `master` == `origin/master` nakon commita "Fix the inherited upstream
defects ..." (`a2925b2`) i commita s dokumentacijom iza njega. Radno stablo čisto. Cijeli suite:
**430 passed**; `black --check .` čist; `pylint anastruct` 10.00/10; `mypy anastruct` samo
naslijeđena greška `basic.py:41`; primjeri `ex_27..29` i `ex_4` se izvršavaju.

## Što je gotovo (verificirano testovima i neovisnim pregledima Sonnet 5.5 + Opus 5.5)

- Prisilni pomaci/rotacije oslonaca (`imposed_displacement`, i u `LoadCase`) i toplinska
  opterećenja (`thermal_load`, uniformna + gradijent, i u `LoadCase`).
- Egzaktna krutost zglobova i rotacijskih opruga na krajevima elemenata (kondenzacija, opruge u
  seriji) za sva opterećenja, uz odgovarajuće krajnje momente q-opterećenja.
- Detekcija unutarnjih zglobova neovisna o redoslijedu elemenata; `discretize()` za bilo koji
  model; `insert_node()` kopira q-opterećenja ispravno.
- Faktor izvijanja i drugi red: svojstvena zadaća, geometrijska matrica (simetrija, kosi
  oslonci, zglobovi/opruge, rešetka), plastični zglobovi u drugom redu, `LoadCombination`.
- Opterećenje na čvoru s kosim valjkastim osloncem; izmjene modela nakon `solve`; poruke za
  zadane pomake u smjeru koji nijedan oslonac ne sprječava.
- **Naslijeđeni nedostaci upstreama (zadnji krug):** plastični zglob kao rotacijska opruga u seriji
  (egzaktna preraspodjela, rasterećenje, mehanizam → `FEMException`); rupe u ID-jevima čvorova
  (`dof_index`, novi ID = max + 1); `LoadCase.dead_load`; `validate()` prije prvog `solve()`;
  `float64` koordinate; element nulte duljine i opruga s beskonačnom krutošću; dosljedan
  `invert_y_loads=False`.
- Copilotove izmjene na GitHubu (RTD workflow ne pada bez secreta; tipizacija geometrijske
  matrice) pregledane i zadržane.
- VAŽNO: testovi su obične `test_*` funkcije; ugniježđeni `describe_/context_/it_` blokovi
  (upstream testovi) se u ovom okruženju ne izvršavaju (vidi SPOZNAJE.md); izvršive verzije
  ključnih su u `tests/test_upstream_examples.py`.

## Odluke koje je korisnik donio

1. Prisilni pomaci = zasebna metoda `imposed_displacement()` (ne kwargs na `add_support_*`).
2. Toplina = uniformno + gradijent s eksplicitnim `alpha` i `h`.
3. Predznaci prate konvenciju odgovarajućih sila: pozitivan `Fy`, `q` i `uy` djeluju u smjeru
   +y (vidi SPOZNAJE.md, sekcija 4); pozitivan gradijent = toplije lice lijevo od smjera
   čvor1→čvor2 (gornje za vodoravni element).
4. Greška (FEMException) umjesto upozorenja kad se zadaje pomak u smjeru koji oslonac ne
   sprječava (rezultat bi bio pogrešan).
5. Sve ispravke se rade bez force-pusha; prije pusha `git fetch` i merge ako je remote odmaknuo
   (Copilot i korisnik također commitaju na `master`).
6. Svaka veća promjena prolazi neovisni pregled (Sonnet 5.5 + Opus 5.5) prije commita; nalazi se
   provjeravaju pokretanjem koda (jedna rana pogrešna odluka o predznaku ispravljena je tako).

## Otvoreno / mogući sljedeći koraci

- [ ] Plotter: prikaz toplinskog opterećenja i prisilnih pomaka.
- [ ] `deflection`/`extension` krivulje ne uključuju slobodnu toplinsku zakrivljenost/istezanje.
- [ ] `load_factor=0` daje `AssertionError("There are no forces on the structure")`.
- [ ] `validate()` radi `copy.copy(self)` (plitko): dijeli popise oslonaca s izvornim sustavom.
- [ ] `Vertex.__eq__` s `tuple`-om je točna usporedba (bez zaokruživanja na 12 znamenki).
- [ ] Član sa zglobovima na oba kraja ne izvija između čvorova bez `discretize_kwargs`
      (dokumentirano); geometrijska matrica zglobova/opruga je kondenzirana na razini prvog reda
      (točnost ~0,2 % za jedan element po članu, konvergira s `discretize`).
- [ ] Upstream nested testovi (`tests/test_e2e.py`, `test_beam.py`, `test_truss.py`, ...) nisu
      izvršni; ako se žele izvršavati, treba ih "raspakirati" u obične funkcije (moguće da dio
      očekivanja tada padne — provjeriti neovisno, kao u `test_upstream_examples.py`).
- [ ] Cython (`anastruct/fem/cython/celements.pyx`) nije dirnut: ne koristi plastični zglob ni
      nove oslonce; ako se Cython put koristi, treba ga uskladiti ili provjeriti.
- [ ] CI (`test.yaml`) se pokreće samo na pull requestovima; push na `master` ga preskače, pa
      lokalno pokrenuti `black`, `pylint`, `mypy`, `pytest` prije pusha.

## Git

Commit i push na https://github.com/mgrubisic/anaStruct (`master`). `commit_push.bat` u rootu
je osobna skripta za taj fork (pita potvrdu prije commita i pusha); nije za upstream. Oprez:
commit poruke završavaju s `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>`.
U rootu postoji i `anaStruct-master (1).zip` (arhiva upstreama; `*.zip` je u `.gitignore`).

## Kako pokrenuti

```bash
cd C:\GITHUB_Master\anaStruct_KIMI
MPLBACKEND=Agg python -m pytest                    # cijeli suite (430)
python -m pytest tests/test_imposed_thermal.py -v  # jedna datoteka
black --check . && python -m pylint anastruct && python -m mypy -p anastruct
```

Pažnja: skripte izvan roota pokretati s `PYTHONPATH=.` — inače se importa instalirani
anaStruct umjesto radne kopije.

## Okruženje

- Python 3.12 (conda), Windows 11, Git Bash / PowerShell.
- Lokalne verzije: numpy 1.26, scipy 1.12, matplotlib 3.8, pytest 9.0, pytest-describe 3.1,
  black 25.9, pylint 3.3, mypy 2.1. `pyproject.toml` (CI) pinira novije (black 26.10, pylint 4.1.2,
  mypy 2.4, pytest-describe 3.2) — razlike u formatiranju/lintu mogu se pokazati tek u CI-ju.
- pytest + pytest-describe + pytest-raises + pytest-cov (konfigurirano u `pyproject.toml`,
  `addopts = "--cov=anastruct"`).
- Paket se importira izravno iz repozitorija (nije potrebna instalacija).
