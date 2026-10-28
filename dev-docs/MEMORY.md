# MEMORY — stanje projekta i nastavak rada

> Zapis stanja za brzo hvatanje konteksta u sljedećoj sesiji. Detalji i izvodi su u
> `SPOZNAJE.md`, upute za korištenje u `SKILLS.md`, mapa promjena u `README.md`.

## Što je gotovo (verificirano testovima i neovisnim pregledima)

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
- Cijeli suite: **391 passed**. VAŽNO: testovi su obične `test_*` funkcije; ugniježđeni
  `describe_/context_/it_` blokovi (upstream testovi) se u ovom okruženju ne izvršavaju
  (vidi SPOZNAJE.md); izvršive verzije ključnih su u `tests/test_upstream_examples.py`.
- `mypy anastruct` — bez novih grešaka (1 naslijeđena u `basic.py:41`).
- Primjeri `examples/ex_27..29` su pokrenuti i provjereni.

## Odluke koje je korisnik donio

1. Prisilni pomaci = zasebna metoda `imposed_displacement()` (ne kwargs na `add_support_*`).
2. Toplina = uniformno + gradijent s eksplicitnim `alpha` i `h`.
3. Predznaci prate konvenciju odgovarajućih sila: pozitivan `Fy`, `q` i `uy` djeluju u smjeru
   +y (vidi SPOZNAJE.md, sekcija 4); pozitivan gradijent = toplije lice lijevo od smjera
   čvor1→čvor2 (gornje za vodoravni element).
4. Greška (FEMException) umjesto upozorenja kad se zadaje pomak u smjeru koji oslonac ne
   sprječava (rezultat bi bio pogrešan).

## Otvoreno / mogući sljedeći koraci

- [ ] Plotter: prikaz toplinskog opterećenja i prisilnih pomaka.
- [ ] `deflection`/`extension` krivulje ne uključuju slobodnu toplinsku zakrivljenost/istezanje.
- [ ] Nasljeđeni problemi upstreama koji nisu mijenjani (popis u SPOZNAJE.md, kraj): model
      plastičnog zgloba (`update_stiffness`), čvorovi nakon `remove_element` (gusti indeksi),
      `LoadCase.dead_load`, `validate()` prije prvog `solve`, `float32` koordinate.
- [ ] Geometrijska matrica zglobova/opruga je kondenzirana na razini prvog reda (točnost
      ~0,2 % za jedan element po članu, konvergira s `discretize`).
- [ ] Upstream nested testovi (`tests/test_e2e.py`, ...) nisu izvršni; ako se žele izvršavati,
      treba ih "raspakirati" u obične funkcije.

## Git

Commit i push na https://github.com/mgrubisic/anaStruct (`master`). `commit_push.bat` u rootu
je osobna skripta za taj fork (pita potvrdu prije commita i pusha); nije za upstream.

## Kako pokrenuti

```bash
cd C:\GITHUB_Master\anaStruct_KIMI
MPLBACKEND=Agg python -m pytest                    # cijeli suite
python -m pytest tests/test_imposed_thermal.py -v  # jedna datoteka
```

Pažnja: skripte izvan roota pokretati s `PYTHONPATH=.` — inače se importa instalirani
anaStruct umjesto radne kopije.

## Okruženje

- Python 3.12 (conda), Windows, Git Bash.
- pytest + pytest-describe + pytest-raises + pytest-cov (konfigurirano u `pyproject.toml`,
  `addopts = "--cov=anastruct"`).
- Paket se importira izravno iz repozitorija (nije potrebna instalacija).
