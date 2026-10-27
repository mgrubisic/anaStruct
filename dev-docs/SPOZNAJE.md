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
- `process_conditions()` klasificira DOF kao obustavljen ako je
  `vector[i] == 0` **ili** `i in system._prescribed_displacements`.
  ⚠️ Ne smije se koristiti `isnan()` za klasifikaciju: `solve()` se poziva
  više puta (npr. `geometrically_non_linear` u `solver.py`), a nakon prvog
  solvea vektor sadrži rezultate bez ijednog NaN — sve bi se pogrešno
  proglasilo obustavljenim.
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
  `ux=uy=phi_z=0` — uklonjeno da se vidi zadani pomak (za obične oslonce
  vrijednosti i dalje ispadaju ~0).

## 3. Thermal load — izvod ekvivalentnih čvornih sila

Eigen-deformacije elementa (u generaliziranim koordinatama koje koristi
`Element.kinematic_matrix` A: [izduženje, rotacija kraja 1 rel. tetivi,
rotacija kraja 2 rel. tetivi]):

```
e0 = [α·ΔT·l ,  κ·l/2 ,  κ·l/2]        κ = α·ΔT_grad / h
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
(tlak) ✓. Gradijent → p = [0,0,−EIκ,0,0,+EIκ] (samo momenti, bez poprečnih
sila ✓), konstantan |M| = EIκ ✓.

## 4. Konvencije predznaka (najskuplje naučena lekcija)

Izlazni getteri i interni vektori NEMAJU isti predznak:

| veličina | interni vektor | `get_node_displacements` |
|---|---|---|
| ux | +x | isto kao interno |
| uy | +gore | **−interno** (default: + = dolje) |
| phi_z | + = u smjeru kazaljke | **−interno** |

Uz defaultni `invert_y_loads=True` (`orientation_cs = -1`): pozitivan `Fy`/`q`
djeluje dolje i daje pozitivni prijavljeni `uy`; pozitivan `Tz` daje
**negativan** prijavljeni `phi_z`. Momenti/rotacije su dakle interno "obrnute"
u odnosu na izlaz — to je postojeća kvirkica biblioteke, ne bug ove
implementacije.

Odluka: ulazi `imposed_displacement`/`thermal_load` prate **konvenciju
odgovarajućih sila** (`ux`~`Fx`, `uy*orientation_cs`~`Fy`, `phi_z`~`Tz`), jer
su tako strukturirane sve ostale ulazne metode. Posljedica koju treba znati:
`get_node_displacements(...)["phi_z"]` prijavljuje zadanu rotaciju s obrnutim
predznakom (kao i kod momentnog opterećenja). Pokriveno testom s komentarom.

Ostale kvarkije:
- `add_support_roll(direction=...)` — direction je **slobodni** smjer kotrljanja,
  ne obustavljeni! `direction="x"` (default) → obustavlja `uy`.
- `add_support_roll(node_id=[2,3])` pada na assertu jer se `direction/angle/rotate`
  ne broadcastaju — ili liste za sve, ili zasebni pozivi.
- `reaction_forces[i].Fy` > 0 znači silu oslonca NA konstrukciju prema gore.

## 5. Zamke na koje se naišlo

1. **Import instaliranog paketa umjesto radne kopije**: skripta izvan roota
   (`python /tmp/x.py`) stavlja direktorij skripte na `sys.path`, ne cwd →
   importa se instalirani anaStruct bez novih featurea. Rješenje: `PYTHONPATH=.`.
2. **`validate()` radi `copy.copy(self)` (shallow)**: `prep_matrix_forces` na
   kopiji mutira `element_primary_force_vector` DIJELJENIH elemenata — neškodljivo
   jer `solve()` odmah poslije radi `el.reset()` + svježi `prep_matrix_forces`.
   Isto pravilo vrijedi i za novi `apply_temperature_load` — zato je bitno da
   se sve primary-force izmjene događaju kroz taj isti tok.
3. **`process_supports` se izvršava samo jednom** (kad je
   `system_displacement_vector is None`). Podršci i imposed pomaci zadani nakon
   prvog solvea se ne primjenjuju — isto ponašanje kao i za postojeće oslonce;
   dokumentirano u docstringu.
4. **`LoadCase` exec-mehanika** (`apply_load_case`): spec dict se stringificira
   pa se execa `self.<metoda>(kwargs)` — radi samo sa skalarnim/None/list
   argumentima. Zato `LoadCase.thermal_load` čuva čiste skalare.
5. **`Element.__add__`** (za LoadCombination) zbraja rezultate i q-loadove;
   `temperature_load` dodan u zbrajanje radi konzistentnosti.

## 6. Reproducibilni diff prema upstreamu

Original je u `anaStruct-master (1).zip`. Točan skup izmjena:

```bash
mkdir -p /tmp/anastruct_orig && unzip -oq "anaStruct-master (1).zip" -d /tmp/anastruct_orig
diff -u /tmp/anastruct_orig/anaStruct-master/anastruct/fem/system.py anastruct/fem/system.py
# (isto za elements.py, postprocess.py, util/load.py, system_components/assembly.py)
```

Diff je namjerno minimalan: 6 postojećih datoteka dirnuto, 0 izbrisane linije
logike (osim 3 linije nuliranja pomaka u `reaction_forces`), sve ostalo su
dodaci.
