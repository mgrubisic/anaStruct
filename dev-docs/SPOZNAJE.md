# SPOZNAJE — tehnički nalazi o anaStruct arhitekturi i ovoj implementaciji

> Bilješke nastale tijekom implementacije imposed displacement + thermal load.
> Namjena: da se sljedeći put ne mora nanovo otkrivati kako modul radi iznutra.

## 1. Mapa toka podataka (pipeline)

```
SystemElements.<load/support metode>          anastruct/fem/system.py
        │  pohrana u self.loads_* dictove + atribute elemenata
        ▼
solve()
  ├─ process_supports()          assembly.py — oslonci → 0 u displacement vektoru,
  │                              kompilacija konstitutivnih matrica za zglobove,
  │                              NOVO: primjena loads_displacement
  ├─ prep_matrix_forces()        assembly.py — loads_* → ekvivalentne čvorne sile
  │                              (system_force_vector) i element_primary_force_vector;
  │                              NOVO: apply_temperature_load()
  ├─ assemble_system_matrix()    globalna K + opruge iz system_spring_map
  ├─ process_conditions()        brisanje obustavljenih DOF-ova;
  │                              NOVO: korekcija F_f -= K_fr @ u_r
  ├─ np.linalg.solve()           reducirani sustav
  ├─ scatter                     NOVO: upis zadanih vrijednosti na prescribed indekse
  └─ postprocess.py              element_force_vector = K_el @ u_el, zatim
                                 node_results → reaction_forces → element_results
```

Ključno svojstvo dizajna: **čvorne rezultate dobivamo iz elemenata**
(`element_force_vector + element_primary_force_vector`), a sistemske čvorne
sile i reakcije iz njihovog zbrajanja/negiranja. Zato obje nove
funkcionalnosti rade "besplatno" kroz post-procesiranje čim su točno
implementirane na razini assemblyja.

## 2. Imposed displacement — mehanika implementacije

- `set_displacement_vector()` (assembly.py) obustavljene DOF-ove postavlja na
  **hardkodiranu 0**, slobodni ostaju `NaN`.
- Obustavljeni DOF-ovi se određuju **jednom**, u `process_supports()`, i pamte u
  `system._restrained_dofs` (maska: `vector == 0` ili zadani pomak); `process_conditions()`
  koristi tu masku. ⚠️ Vektor pomaka se ne smije čitati nakon `solve()`: sadrži rješenje, a
  svaki DOF čije je rješenje točno 0 (poprečni pomaci aksijalno opterećenog stupa) izgledao
  bi obustavljen (to je davalo pogrešan faktor izvijanja pri ponovljenom računu). Isto tako
  se `_remainder_indexes` resetira pri svakom pozivu.
- Korekcija reduciranog sustava (udžbenički postupak zadanih pomaka):
  `K_ff · u_f = F_f − K_fr · u_r`, implementirano prije brisanja redaka/stupaca
  preko `np.ix_(_remainder_indexes, prescribed_indexes)`.
- Nakon solvea, zadane vrijednosti se upisuju natrag u
  `system_displacement_vector` → elementi ih pokupe u
  `element_displacement_vector` → krajnje sile i reakcije ispadnu točne bez
  ijedne dodatne izmjene.
- `validate()` ima assert "There are no forces on the structure" koji bi pao za
  analizu čistog slijeganja — proširen da prolazi kad postoje ne-nula
  prescribed pomaci.
- `postprocess.reaction_forces()` je ranije na čvorovima oslonaca hardkodirao
  `ux=uy=phi_z=0` — uklonjeno da se vidi zadani pomak. ⚠️ Posljedica (promjena prema
  upstreamu): `reaction_forces[i].ux/uy/phi_z` su sada stvarni pomaci čvora, pa su ne-nula i
  u slobodnim smjerovima oslonaca (npr. rotacija zglobnog oslonca, pomak kotljača u slobodnom
  smjeru), dok su prije uvijek bili 0.

## 3. Thermal load — izvod ekvivalentnih čvornih sila

Eigen-deformacije elementa (u generaliziranim koordinatama koje koristi
`Element.kinematic_matrix` A: [izduženje, rotacija kraja 1 rel. tetivi,
rotacija kraja 2 rel. tetivi]):

```
e0 = [α·ΔT·l , −κ·l/2 , −κ·l/2]        κ = α·ΔT_grad / h
p  = Aᵀ · C · e0                        (6-vektor, globalne koordinate)
```

`p` je vektor fiksnih (upetih) krajnjih sila. Primjena identična q-load
patternu: `element_primary_force_vector -= p`, a `system_force_vector += p` na
čvorove. Prednosti ovog formalizma naspram ručnog izvoda po komponentama:

- automatski točno za **internal hinge / oprugu na kraju elementa** (C je već
  modificirana),
- radi za bilo koju orijentaciju elementa (A nosi kutove a1/a2),
- za `truss` elemente rotacijske komponente e0 se nuliraju.

Ručna provjera (fiksno-fiksni vodoravni štap): uniformni ΔT →
p = EAαΔT·[−1,0,0,1,0,0] (čvorove gura van = ekvivalentno opterećenje širenja
✓); reakcije kroz post-procesiranje ispadnu ±EAαΔT prema unutra ✓, N = −EAαΔT
(tlak) ✓. Gradijent → p = [0,0,+EIκ,0,0,−EIκ] (samo momenti, bez poprečnih
sila ✓), konstantan |M| = EIκ ✓. Predznak −κ u e0 slijedi iz kinematičke matrice
(relativne rotacije krajeva su pozitivne za zakrivljenost prema desnoj strani
smjera čvor1→čvor2, tj. prema hladnijem licu): pozitivan gradijent = toplije lice
lijevo (gornje za vodoravne elemente). Neovisna provjera: vrh konzole −κL²/2 (dolje),
slobodno oslonjena greda +κL²/8 (nadvišenje), upeta greda: progibni moment EIκ.

## 4. Konvencije predznaka (najskuplje naučena lekcija)

Ova sekcija je ispravljena nakon neovisnog pregleda; ranija verzija je tvrdila da je
pozitivan `Fy`/`q` usmjeren dolje i da je pozitivan `uy` slijeganje, a to je POGREŠNO.

**Ulazi** (uz zadani `invert_y_loads=True`): pozitivan `Fy`, `q` i zadani `uy` djeluju
u smjeru **+y geometrije (prema gore)**; `Fx` u smjeru +x; opterećenje prema dolje je
negativno, slijeganje je negativan `uy`. Uz `invert_y_loads=False` pozitivan `Fy`
djeluje u smjeru −y (gravitacija). Neovisno sidro: `Fy=+1` na vrhu okomitog stupa
(y=+L) daje `N=+1` (vlak), a `Fy=−1` tlak (faktor izvijanja je konačan samo za tlak).

**Izlazi** (okvir crteža: +x desno, +y gore):

| veličina | `system_displacement_vector` | `node_map[i]` | `get_node_results_system` | `get_node_displacements` |
|---|---|---|---|---|
| ux | +x | −x | −x | **+x** |
| uy | +dolje | +gore | +dolje | **+gore** |
| phi_z | u smjeru kazaljke | | | u smjeru kazaljke |

- `get_node_displacements` je u okviru crteža (koristiti njega za fizikalne provjere).
- `phi_z` je u SVIM getterima pozitivan u smjeru kazaljke: `Tz=+1` na vrhu konzole
  podiže vrh, a `phi_z` se prijavljuje kao `−ML/EI`. Zadana rotacija `phi_z=+0.01`
  (suprotno kazaljci, kao `Tz`) se zato prijavljuje kao `−0.01`.
- Reakcije: `reaction_forces[i].Fy` je pozitivan prema DOLJE (slobodno oslonjena greda
  s `q=−1` daje `Fy=−5`), `Fx` je u okviru crteža (+desno), `Tz` suprotno kazaljci.
- Momenti: **progib je negativan** (greda s `q=−10` ima `M=−qL²/8`); progib elementa
  `w` je pozitivan prema dolje.

Odluka: ulazi `imposed_displacement`/`thermal_load` prate **konvenciju
odgovarajućih sila** (`ux`~`Fx`, `uy*orientation_cs`~`Fy`, `phi_z`~`Tz`), jer
su tako strukturirane sve ostale ulazne metode. Pokriveno testovima.

Ostale kvarkije:
- `add_support_roll(direction=...)` — direction je **slobodni** smjer kotrljanja,
  ne obustavljeni! `direction="x"` (default) → obustavlja `uy`.
- `add_support_roll(node_id=[2,3])` pada na assertu jer se `direction/angle/rotate`
  ne broadcastaju — ili liste za sve, ili zasebni pozivi.
- Zadani pomak je teorijski moguć samo u smjeru koji oslonac **sprječava**: u slobodnom
  smjeru se čvor pomiče kako konstrukcija zahtijeva, pa je pomak rezultat, a ne uvjet.
  Zato nije moguć u slobodnom smjeru kotljača, za rotaciju zglobnog oslonca, na opružnom
  osloncu ni u čvoru bez oslonca. Kosi valjkasti oslonac sprječava samo translaciju
  okomito na slobodni smjer, a zadani `ux`/`uy` su globalni (imaju komponentu u slobodnom
  smjeru) pa se na njemu ne mogu zadati. U svim tim slučajevima baca se `FEMException`
  s porukom koja objašnjava što koji oslonac sprječava (`describe_unrestrained_dof`).
- `invert_y_loads=False`: `q_load(direction="x")` i `point_load(rotation=...)` ne prate
  istu konvenciju kao uz zadanu vrijednost (naslijeđeno iz upstreama, nije mijenjano).

## 5. Zamke na koje se naišlo

1. **Import instaliranog paketa umjesto radne kopije**: skripta izvan roota
   (`python /tmp/x.py`) stavlja direktorij skripte na `sys.path`, ne cwd →
   importa se instalirani anaStruct bez novih featurea. Rješenje: `PYTHONPATH=.`.
2. **`validate()` radi `copy.copy(self)` (shallow)**: `prep_matrix_forces` na
   kopiji mutira `element_primary_force_vector` DIJELJENIH elemenata — neškodljivo
   jer `solve()` odmah poslije radi `el.reset()` + svježi `prep_matrix_forces`.
   Isto pravilo vrijedi i za novi `apply_temperature_load` — zato je bitno da
   se sve primary-force izmjene događaju kroz taj isti tok.
3. **`process_supports` se izvršava kad je `system_displacement_vector is None`**, tj.
   pri prvom `solve` i nakon svake izmjene modela: `_invalidate_solution()` (poziva se iz
   `add_element`, `remove_element`, `add_support_*`, `add_internal_hinge`) resetira vektor,
   masku upetih stupnjeva, zadane pomake i plastično stanje. Zadani pomak se može zadati samo
   dok rješenje ne postoji (inače `FEMException`).
4. **`LoadCase` exec-mehanika** (`apply_load_case`): spec dict se stringificira
   pa se execa `self.<metoda>(kwargs)` — radi samo sa skalarnim/None/list
   argumentima. Zato `LoadCase.thermal_load` čuva čiste skalare.
5. **`Element.__add__`, `Node.__add__`, `Node.add_results`** se više ne koriste u biblioteci
   (kombinacija je jedan račun, `_add_loads`); ostavljeni su zbog kompatibilnosti API-ja
   (`Element.__add__` zbraja i `temperature_load`).

## 6. Reproducibilni diff prema upstreamu

Posljednji upstream commit prije forka je `f85d14a`:

```bash
git diff f85d14a --stat
git diff f85d14a -- anastruct/fem/system.py
```

Fork je značajno veći od prve verzije (dodaci + ispravci opisani u ovoj datoteci): oko 12
izvornih datoteka i 6 novih testnih datoteka; sve promjene ponašanja prema upstreamu navedene
su u ovoj datoteci i u korisničkoj dokumentaciji.

## Review nalazi (Sonnet 5.5 + Opus 5.5, naknadni pregled)

- **Testovi:** u ovom okruženju (pytest-describe 3.1.0) `context_*` funkcije
  postaju samostalni testovi, a ugniježđeni `it_*` se nikad ne izvrše. Prvotnih
  13 testova zato nije provjeravalo asercije; testovi su prepisani u obične
  `test_*` funkcije (svi prolaze). Isti obrazac koriste i upstream testovi
  (`tests/test_analytical.py` itd.) — njihove asercije se također ne izvršavaju.
- **Predznaci (ispravljeno — ranija verzija ove bilješke bila je POGREŠNA):**
  uz zadani `invert_y_loads=True` pozitivan `Fy`, `q` i zadani `uy` djeluju u smjeru
  **+y (prema gore)**, a ne u smjeru gravitacije (docstring konstruktora je bio
  obrnut). Zato je slijeganje negativan `uy`. Neovisno sidro: `Fy=+1` na vrhu okomitog
  stupa (y=+L) daje `N=+1` (vlak). `get_node_displacements()` je u okviru crteža
  (+x desno, +y gore); ostali getteri se razlikuju (tablica u sekciji 4). Predznak
  termičkog gradijenta je ispravljen u kodu
  (`apply_temperature_load`): pozitivan gradijent = toplije lice lijevo od smjera
  čvor1→čvor2 (gornje za vodoravne elemente), vrh konzole ide prema dolje (−κL²/2), a
  slobodno oslonjena greda se nadvisuje (+κL²/8). Momenti: progib je negativan u
  anaStruct.
- **Ispravljeno:** dvostruko brojanje zadanog pomaka u `LoadCombination`
  (`LoadCase.imposed_displacement` + faktor), pad u `insert_node` nakon
  `thermal_load`, tihi gubitak opterećenja u `discretize()`/`remove_loads()`,
  tihi no-op `imposed_displacement` nakon `solve()`, kosi valjkasti oslonac i
  rotacija na unutarnjem zglobu (sada `FEMException`).
- **Zglobovi i opruge (ispravljeno):** `constitutive_matrix` je u upstreamu za
  zglob samo nulirao retke (4EI/l umjesto 3EI/l; npr. progib Gerberove grede
  0,08 umjesto 0,1067), a za konačnu oprugu davao dijeljenje s nulom/NaN. Sada se
  zglob eliminira statičkom kondenzacijom (Schurov komplement), a opruga se
  dodaje u seriju preko fleksibilnosti: `C = C_el + diag(1/k1, 1/k2)`; zglob
  ima prednost pred oprugom. Vrijedi za sva opterećenja i unutarnje zglobove.
  Testovi: `tests/test_hinges_springs.py`.
- **Fiksni krajnji momenti q-opterećenja:** `apply_perpendicular_q_load` je
  `det_moment/det_shear` hranio s `constitutive_matrix[i][i] * 1e6`, što je
  zanemarivalo konačne opruge (moment ostao kao kod upetog kraja). Sada se koristi
  `Element.end_rotational_stiffness`: 0 za zglob, `k` za oprugu, `D*1e6` za kruti
  kraj. Za plastificirane krajeve (`nodes_plastic`) vrijedi stara formula iz
  trenutne (skalirane) matrice da iteracija konvergira prema `mp`; rešetkasti
  elementi imaju (0, 0) (inače krivi posmici kod trapeznog opterećenja).
- **`discretize()` (ispravljeno):** upstream je pretpostavljao lanac elemenata
  (`(id-1)*n+1` za čvorove, q-opterećenje samo na podelement s originalnim ID-jem,
  isti `spring` rječnik na svim podelementima, kosi valjkasti oslonci i eksplicitni
  unutarnji zglobovi su se gubili, rešetkasti elementi postajali opći). Sada se
  grade eksplicitne mape stari→novi ID čvorova/elemenata; opruge, zglobovi i `mp`
  idu samo na vanjske podelemente; q-opterećenje (i trapezno, i vlastita težina) se
  linearno interpolira; kosi oslonci, opružni oslonci, eksplicitni zglobovi,
  toplinska opterećenja i zadani pomaci se prenose; plotter ostaje vezan na sustav.
  ID-jevi čvorova i elemenata u novoj mreži su drukčiji (koristiti `find_node_id`).
  Rešetkasti elementi se ne dijele (čvorovi između kolinearnih štapova bili bi
  mehanizmi); plotter se kopira da `solve(geometrical_non_linear=True)` ne
  preusmjeri plotter korisnikova sustava na kopiju.
- **Ostalo (ispravljeno u istom koraku):** `add_support_spring` razrješava negativne
  ID-jeve čvorova odmah; `support_check` se za oprugu zove samo za rotaciju
  (translation=3); `node_results_system` čuva `node.elements` i `node.hinge` kad
  zamjenjuje objekte čvorova (inače `remove_element`/`insert_node` nakon `solve()`
  bacaju KeyError i zglobovi nestaju nakon drugog `solve()`).
- **`check_internal_hinges` (ispravljeno):** status čvora se svaki put računa iz
  korisnikovih opruga (koje se više ne mijenjaju) i ne ovisi o redoslijedu
  dodavanja elemenata (T-spoj: stup prije ili poslije otpuštene grede daje isti
  rezultat). Čvor je zglob (kondenzira se, rotacija
  ograničena) ako je deklariran (`add_internal_hinge`, `_explicit_internal_hinges`)
  ili ako su SVI spojeni elementi otpušteni. Jedan kruti element + otpušteni
  elementi: čvor se rješava vlastitom rotacijom (moment-opterećenje i rotacijski
  oslonac u čvoru rade; staro pravilo "najviše jedan kruti" ih je tiho gubilo);
  plotter i dalje crta simbol zgloba na svakom otpuštenom kraju. Elementi u
  `add_element` kopiraju `spring`/`mp` rječnike, `remove_element` ponovno provjerava
  preostale čvorove; `Element.springs`/`Element.hinges` više ne dobivaju
  `{1:0}`/`{2:0}` na zglobnim čvorovima (stanje je u `node.hinge`/`internal_hinges`). Čvorovi u `internal_hinges` se uspoređuju po ID-ju jer
  postprocesiranje zamjenjuje objekte u `node_map` (i briše `node.hinge`, koji
  `solve()` svaki put ponovno postavlja).
- **Faktor izvijanja (ispravljeno):** `det_linear_buckling` je imao tri nepovezana
  uzroka netočnih faktora (npr. 6697 umjesto Eulerovih 493,5 za `ex_18`):
  1. `np.linalg.inv(kg) * k0` je množenje po elementima, a svojstvena zadaća traži
     matrični umnožak; usto je `kg` singularan kad neki članovi nemaju aksijalnu silu.
     Sada: `smallest_buckling_factor` rješava `K0 x = -λ Kg x` preko Choleskog
     (svojstvene vrijednosti `L⁻¹ Kg L⁻ᵀ`, bez invertiranja `Kg`) i vraća najmanji
     pozitivni λ (samo tlak izvija); bez tlaka je faktor `inf`.
  2. `process_conditions` je pri svakom ponovnom `solve()` čitao upete stupnjeve
     slobode iz vektora pomaka; nakon prvog `solve()` je svaki stupanj čije je
     rješenje točno 0 (poprečni pomaci aksijalno opterećenog stupa) izgledao upet.
     Sada se maska upetih stupnjeva (`_restrained_dofs`) postavlja jednom u
     `process_supports`, a `_remainder_indexes` se resetira (dopisivao se pri svakom
     solveu).
  3. Geometrijska matrica krutosti elementa: konverzija koordinatnog sustava množila
     je samo stupce (`K · S` umjesto `S K S`), pa je matrica bila nesimetrična za sve
     elemente koji nisu okomiti (vodoravni stup: faktor 240000 umjesto 493,5);
     tipfeler `s1*c2` (umjesto `s2*c2`); za kose valjkaste oslonce (`a1 != a2`)
     matrica se sada računa za os elementa i zaokreće u sustav čvora
     (`R(a_čvor − kut)`, izvedeno iz kinematičke matrice); za otpuštene krajeve
     (zglob, opruga) se geometrijska matrica kondenzira preko `G = inv(D_greda)·D`.
  Provjera: Euler (zglobni 493,48, konzola 123,37, upeto-zglobni 1009,54, obostrano
  upeti 1973,95 uz n=20), vodoravni i kosi stupovi, portal s krutom gredom, opruga na
  dnu stupa (μL·tan(μL) = kL/EI), zglob na kraju elementa; testovi u
  `tests/test_buckling.py`.
- **Ograničenja ispravljena u zadnjem koraku:**
  1. *Opterećenje na čvoru s kosim valjkastim osloncem.* Stupnjevi slobode takvog čvora su
     u sustavu oslonca (`u_glob = R(δ) u_oslonca`, δ = `inclined_roll[čvor]`); opterećenje
     se u `apply_point_load` prebacuje `f_oslonca = R(δ)ᵀ f_glob`, zabrana u `point_load`
     je uklonjena. Pretvorba čvornih sila elementa u `ElementLevel.node_results` nije
     bila rotacija (`Fx' = -(cFx + sFy)` umjesto `cFx - sFy`; greška se vidjela samo kad je
     komponenta sile u slobodnom smjeru ≠ 0, tj. uz opterećenje na čvoru; isto za
     pomake). Provjera statikom za kutove 30°–225° i −40°, s opterećenjem Fx/Fy.
  2. *`mp=` + `geometrical_non_linear`.* Prije se `solve` vraćao kroz `stiffness_adaptation`
     (prvi red, `buckling_factor=None`). Sada je svaki korak plastične iteracije drugog
     reda (`second_order_step`: prvi red → N0 → `K + Kg(N0)`); faktor izvijanja je onaj
     elastične konstrukcije.
  3. *`LoadCombination` + drugi red / nelinearni elementi.* Rezultati se ne mogu
     superponirati; kombinacija je sada jedan račun sustava sa zbrojem opterećenja svih
     slučajeva (`_add_loads`: točkasta, momentna, linijska — i trapezna i pod kutom —,
     toplinska i zadani pomaci, svako s faktorom) i ima vlastiti `buckling_factor`.
     Linearna kombinacija je i dalje superpozicija.
  4. *Izvijanje štapa rešetke.* `truss` element nema savojnu krutost; štap se modelira
     općim elementom sa zglobovima na oba kraja (`spring={1: 0, 2: 0}`, ponaša se kao
     štap), a `discretize` ga dijeli sa zglobovima samo na krajevima. Euler: 493,48 (štap),
     8723,59 (dvoštapna rešetka). Geometrijska matrica `truss` elementa je matrica žice.
  5. *Izmjene modela nakon `solve`.* `process_supports` se izvršavao samo jednom: oslonac
     dodan nakon rješavanja se tiho zanemarivao, a `insert_node`/`add_element` su bacali
     `IndexError`. `_invalidate_solution()` (poziva se iz `add_element`, `remove_element`,
     `add_support_*`, `add_internal_hinge`) resetira vektor pomaka, masku upetih stupnjeva i
     plastično stanje, pa sljedeći `solve` obrađuje izmijenjeni model.
  6. *Dijeljeni `spring`/`mp` rječnici* (`add_multiple_elements`, `add_element_grid`,
     `insert_node_old`): `add_element` kopira rječnike.
- **Nalazi neovisnog pregleda tog koraka (ispravljeno):**
  - *Zglob dodan nakon `solve`* (`add_internal_hinge`) ponašao se kao upetost: kopije čvorova
    u `element.node_map` (postprocesiranje) nose status zgloba iz zadnjeg `solve`, a
    `compile_constitutive_matrix` ih čita. `Element.refresh_nodes` se poziva iz
    `_invalidate_solution` i `process_supports`; `add_internal_hinge` prvo provjerava zglob pa
    poništava rješenje.
  - *`force_linear=True` + `geometrical_non_linear=True`* s `mp=` je davao plastični rezultat:
    unutarnji `solve()` pozivi sada primaju `force_linear=True`.
  - *Ponovljeni računi s `mp=`:* faktor izvijanja je bio drukčiji pri drugom pozivu (plastično
    reducirane matrice djeljene s kopijom za izvijanje), a plastična iteracija je kretala od
    reducirane krutosti (pomak ~0,1 %). Faktor izvijanja se računa s elastičnom krutošću
    (elementi se nakon toga vraćaju), a svaka plastična iteracija kreće od elastičnog stanja.
  - *Nekonvergencija plastične iteracije* je vraćala zastarjele ili miješane rezultate uz
    upozorenje u logu; sada je to `FEMException("ConvergenceError", ...)` (mehanizam).
  - *`LoadCombination`:* linearna i nelinearna kombinacija su različito brojile opterećenja
    sustava (n puta naspram jednom). Sada su oba puta jedan račun sa zbrojem opterećenja
    (isto kao superpozicija za linearni račun), opterećenja sustava se broje jednom.
  - `buckling_factor` se briše pri izmjeni modela; `second_order_step` prvo vraća krutost
    bez starog `Kg`.
- **Poznato, naslijeđeno iz upstreama (nije mijenjano):** vidi popis "Ostale kvarkije"
  u sekciji 4; geometrijska matrica plastično reduciranog zgloba je približna;
  `add_multiple_elements(..., spring={1: 0, 2: 0})` daje svakom podelementu oba otpuštanja;
  `loads_q` kombinacije (samo za crtanje) sadrži zadnji slučaj.
- Krivulje `deflection`/`extension` elementa ne uključuju slobodnu toplinsku
  zakrivljenost/istezanje (dokumentirano u docstringu).

## Drugi neovisni pregled (Sonnet 5.5 + Opus 5.5 nad cijelim forkom)

Ispravljeno:
- *Plastično stanje curilo je u kasniji `force_linear` račun* (i u `LoadCombination`): linearni
  račun sada uvijek kreće od elastične krutosti (`_reset_plastic_state`).
- *`LoadCombination` + zadani pomak kad je osnovni sustav već riješen*: kopije se poništavaju
  (`_invalidate_solution`).
- *`smallest_buckling_factor` bez slobodnih stupnjeva*: `inf`.
- *Geometrijska matrica zglobova/opruga*: sada se zbroj elastične i geometrijske krutosti grede
  kondenzira zajedno (rotacija kraja grede je unutarnji stupanj slobode, opruga k ili zglob 0);
  jedan element po članu je unutar ~0,2 % (prije do 8,6× previsok faktor), konvergira s
  `discretize`. Statička kondenzacija na razini prvog reda ne može uhvatiti oblik izvijanja
  koji koristi sam unutarnji stupanj (član sa zglobovima na oba kraja je žica dok se ne podijeli).
- *Geometrijska matrica koristi srednji N* (`(N_1 + N_2)/2`), pa ne ovisi o smjeru crtanja
  elementa pri promjenjivoj aksijalnoj sili (vlastita težina).
- *`insert_node` q-opterećenje*: ponovna primjena `-q_load` udvostručavala je faktor opterećenja
  i okretala predznak uz `invert_y_loads=False`, a rešetka je postajala opći element; sada se
  pohranjene vrijednosti kopiraju izravno.
- *`mp=0`* se tiho tretirao kao elastično (provjera je iterirala ključeve): `AssertionError`.
- *Moment na unutarnjem zglobu ili na čvoru samo s rešetkom* nestajao je bez poruke: `FEMException`.
- *`remove_loads` nije brisao `q_perp_load`*.
- *`ConvergenceError`* kaže da se `max_iter` može povećati (nije nužno mehanizam).
- *Kombinacija*: opterećenja slučajeva se **dodaju** opterećenjima sustava (jedan slučaj po
  čvoru/elementu ne oduzima opterećenje sustava više puta); prvi q-slučaj na elementu zadržava
  svoj rastav; šum 1e-17 se zaokružuje na 0.
- *`check_internal_hinges`*: popis zglobova je O(1) po čvoru (rječnik `_internal_hinge_index`);
  prije je svaki poziv gradio popis iznova.
- *Testovi*: horizontalni stup riješen više puta (maska upetih DOF-ova), mutacijski nezaštićene
  grane (oslonci `fixed`/`spring` nakon solvea, apsorpcija osnovnih momenata i pomaka u
  kombinaciji, smjer q pri `discretize`); upstream primjeri 4 i 7 (zastarjela očekivanja: stara
  krutost zgloba 4EI/l i pogrešan niz opruge) provjereni neovisno i izvršivi u
  `tests/test_upstream_examples.py`.

Nasljeđeno iz upstreama, **nije mijenjano** (reproducirano i na upstream kodu):
- Model plastičnog zgloba (`Element.update_stiffness` skalira članove matrice umjesto serijske
  opruge) daje odstupanja preraspodjele i rotacija (od nekoliko do ~25 %).
- `remove_element` ostavlja rupe u ID-jevima čvorova, a `solve` ih koristi kao guste indekse
  matrice (`IndexError`); `insert_node` nakon uklanjanja može isto.
- `LoadCase.dead_load` poziva nepostojeću metodu `SystemElements.dead_load`.
- `validate()` prije prvog `solve()` pada (`AssertionError`), jer oslonci još nisu obrađeni.
- Elementi nulte duljine (`ZeroDivisionError`), opruga s `k=inf` (`LinAlgError`), `load_factor=0`.
- `validate()` koristi gustu `np.linalg.eig` (O(n³)); `assert np.any(u < 1e6)` nikad ne pada.
- Koordinate čvorova su `float32` (`vertex.py`): točnost ~1e-7–1e-5 relativno.
- `point_load(rotation=...)` ne primjenjuje pravu rotacijsku matricu; `copy.copy` dijeli popise
  oslonaca s izvornim sustavom.
- Krivulje `deflection`/`extension` ne uključuju slobodnu toplinsku zakrivljenost (dokumentirano).
- Član s zglobovima na oba kraja ne izvija između čvorova bez `discretize_kwargs` (dokumentirano).

