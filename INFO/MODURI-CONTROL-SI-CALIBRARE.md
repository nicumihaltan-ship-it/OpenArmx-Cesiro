# Moduri de control și calibrare — RobStride / OpenArmX

Ce face fiecare mod de control al actuatoarelor RobStride, ce registre îl
guvernează, cum se ajunge în el din aplicație, și pașii de calibrare — de la
encoder până la zerourile articulațiilor brațului.

Registrele citate sunt din intervalul `0x7005`–`0x702E` (control în timp real),
**verificat identic pe RS00, RS03 și RS04**. Restul spațiului de adrese
(`0x20xx` configurare, `0x30xx` observație) diferă de la model la model — vezi
[PARAMETERS.md](../PARAMETERS.md). Cu alte cuvinte: tot ce urmează despre control
este portabil între modele; calibrarea atinge pe alocuri și registre care nu
sunt.

> **Înainte de orice.** Acestea sunt actuatoare de până la 120 Nm. Motorul stă
> dezactivat până când îl activezi explicit, iar `STOP` rămâne accesibil
> permanent, dar orice pas de mai jos poate pune articulația în mișcare.
> Prima punere în funcțiune se face cu motorul pe banc, nedemontat pe braț.

---

## 1. Starea motorului, înainte de moduri

Fiecare cadru de feedback (tip 2) raportează una din trei stări, vizibilă în
tabul **Control** la câmpul `Mode`:

| Stare | Ce înseamnă |
|---|---|
| `RESET` | motor dezactivat, fără cuplu. Starea de după alimentare și după `STOP` |
| `CALI` | motorul rulează calibrarea de inițializare a encoderului |
| `RUN` | motor activ, bucla de control rulează |

Tranzițiile se fac cu cadre dedicate, nu prin parametri:

| Acțiune | Tip cadru | Buton în aplicație |
|---|---|---|
| Activare | 3 | `Enable` |
| Oprire | 4 | `STOP` |
| Oprire + ștergere defecte latch-uite | 4 cu octet 1 | `Clear faults` |
| Setare zero mecanic | 6 | `Set zero here` |
| Salvare `0x20xx` în flash | 22 | `Save to flash` (tabul Parameters) |

**Distincția importantă:** `run_mode` (`0x7005`) spune *ce lege de control*
rulează; `Enable`/`STOP` spun *dacă* rulează ceva. Sunt independente. Un motor
poate fi în modul viteză și complet dezactivat.

---

## 2. Cele cinci moduri de control

`run_mode` este parametrul `0x7005`, uint8:

| Valoare | Mod | Comanda principală | Când îl folosești |
|---|---|---|---|
| 0 | Operation control (stil MIT) | cadru tip 1 | control de impedanță, teleoperare, mișcare coordonată la 100 Hz+ |
| 1 | Poziție PP (profile position) | `0x7016` | deplasare punct-la-punct cu profil trapezoidal propriu |
| 2 | Viteză | `0x700A` | jog, rulare continuă, teste |
| 3 | Curent (Iq) | `0x7006` | control direct de cuplu, identificare frecări, teste de banc |
| 5 | Poziție CSP (cyclic synchronous) | `0x7016` | traiectorie interpolată de host, punct cu punct |

Nu există valoarea 4.

### 2.1 Operation control (`run_mode = 0`)

Singurul mod care nu se comandă prin parametri, ci printr-un cadru dedicat de
tip 1, cu cinci mărimi într-un singur pachet de 8 octeți:

```
t_ref = kd * (v_set - v_act) + kp * (p_set - p_act) + t_ff
```

- `p_set`, `v_set` — poziția și viteza țintă
- `kp`, `kd` — rigiditatea și amortizarea buclei
- `t_ff` — cuplu feed-forward, transportat în câmpul „data area 2” al ID-ului

Cazuri limită utile:

- `kp = 0`, `kd > 0` → pur amortizare de viteză (util pentru gravity
  compensation și pentru a face brațul „moale” dar stabil)
- `kp = 0`, `kd = 0` → comandă pură de cuplu, echivalentă funcțional cu modul
  curent, dar la rata cadrului tip 1
- `kp` mare, `kd` mic → articulație rigidă, cu risc de oscilație

**Capcana de scalare.** Toate cele cinci mărimi călătoresc ca uint16 scalate
față de limite **per model**. `KP_MAX`/`KD_MAX` sunt **500/5 pe RS00–RS02** și
**5000/100 pe RS03/RS04** — un factor de 10. Un Kp valid pentru RS04 trimis
către un RS02 înseamnă cu totul altceva decât crezi. Aplicația reglează
automat intervalele câmpurilor după modelul selectat în panoul de conexiune,
**dar numai dacă modelul a fost setat corect acolo**.

În aplicație: `Enter mode` intră în modul operation și activează motorul,
`Send` trimite un cadru, iar `Stream at 100 Hz` îl repetă la fiecare 10 ms.
Pentru streaming continuu merită setat și watchdog-ul CAN (`0x7028`, vezi
secțiunea 5) — altfel, dacă host-ul tace, motorul rămâne pe ultima comandă.

### 2.2 Poziție PP (`run_mode = 1`)

Motorul își generează singur profilul trapezoidal către țintă.

| Registru | Nume | Rol |
|---|---|---|
| `0x7016` | `loc_ref` | unghiul țintă, rad |
| `0x7024` | `vel_max` | viteza de croazieră a profilului (implicit 10) |
| `0x7025` | `acc_set` | accelerația profilului (implicit 10) |
| `0x702E` | `dcc_set` | decelerația profilului (implicit 10) |
| `0x7018` | `limit_cur` | plafonul de curent |

Ordinea corectă: întâi limitele (`vel_max`, `acc_set`), abia apoi ținta
(`loc_ref`). Ținta scrisă prima pornește mișcarea cu profilul vechi.

**PP blochează setarea zeroului.** Firmware-ul refuză cadrul de tip 6 în acest
mod. Dacă vrei să calibrezi zeroul, treci întâi în CSP sau operation.

### 2.3 Viteză (`run_mode = 2`)

| Registru | Nume | Rol |
|---|---|---|
| `0x700A` | `spd_ref` | viteza comandată, rad/s |
| `0x7022` | `acc_rad` | accelerația (implicit 15) |
| `0x7018` | `limit_cur` | plafonul de curent — și limitatorul de cuplu efectiv |

Este modul cel mai potrivit pentru primele mișcări ale unui motor nou: comanzi
o viteză mică, cu `limit_cur` scăzut, și articulația nu poate dezvolta forță
mare dacă ai greșit ceva.

**Butoanele de jog forțează acest mod**, indiferent ce e selectat în dropdown.

### 2.4 Curent (`run_mode = 3`)

`0x7006` (`iq_ref`) este comanda de curent pe axa q, în amperi — proporțională
cu cuplul. Fără nicio buclă de poziție sau viteză deasupra: motorul va
accelera până la limita mecanică dacă articulația e liberă. De folosit doar cu
articulația încărcată sau cu opritori.

Utilizări legitime: măsurarea frecării statice, verificarea constantei de
cuplu, identificarea gravitației pe articulație.

### 2.5 Poziție CSP (`run_mode = 5`)

Poziție ciclică sincronă: host-ul trimite puncte succesive dintr-o traiectorie
pe care el o interpolează, iar motorul le urmărește direct, fără profil
propriu.

| Registru | Nume | Rol |
|---|---|---|
| `0x7016` | `loc_ref` | punctul curent al traiectoriei, rad |
| `0x7017` | `limit_spd` | limita de viteză (**alt registru decât la PP**) |
| `0x7018` | `limit_cur` | plafonul de curent |

Diferența PP vs CSP în două rânduri: **PP** primește o destinație și își face
singur drumul (`0x7024`); **CSP** primește drumul de la tine, punct cu punct
(`0x7017`). A confunda registrele de limită între ele este cea mai frecventă
greșeală — o limită scrisă în registrul modului celuilalt este pur și simplu
ignorată.

---

## 3. Comutarea între moduri — regula care nu se încalcă

**Se schimbă modul numai cu motorul oprit.** Manualul spune explicit că o
schimbare de mod în timpul rulării duce la comportament nedefinit.

Secvența corectă:

```
STOP  →  scrie 0x7005  →  Enable  →  scrie setpoint-urile
```

Aplicația face asta singură: fiecare buton `Apply ...` din tabul Control
citește `0x7005`, iar dacă nu e modul necesar face oprire → scriere mod →
activare. Dacă motorul e deja în modul corect, dansul se sare intenționat: un
disable/enable inutil face articulația să scape sarcina și s-o reprindă.

Butonul `Apply mode`, în schimb, **doar scrie `0x7005` și lasă motorul oprit** —
este pentru cazul în care vrei să pregătești modul fără să miști nimic.

**A doua regulă, mai perfidă:** un setpoint scris în modul greșit este
**ignorat în tăcere**, fără eroare și fără răspuns negativ. Un `loc_ref` scris
în modul viteză nu face nimic. Dacă o comandă „nu are efect”, primul lucru de
verificat este `0x7005` — butonul `Read` de lângă dropdown îl recitește, iar
eticheta devine portocalie când modul real diferă de cel selectat.

---

## 4. Buclele de reglaj

Cascadă clasică: curent înăuntru, viteză deasupra, poziție la exterior.

| Registru | Nume | Buclă | Implicit |
|---|---|---|---|
| `0x7010` | `cur_kp` | curent | 0.17 |
| `0x7011` | `cur_ki` | curent | 0.012 |
| `0x7014` | `cur_filt_gain` | filtru curent | 0.1 |
| `0x701F` | `spd_kp` | viteză | 6 |
| `0x7020` | `spd_ki` | viteză | 0.02 |
| `0x7021` | `spd_filt_gain` | filtru viteză | 0.1 |
| `0x701E` | `loc_kp` | poziție | 60 |
| `0x700B` | `limit_torque` | plafon de cuplu | — |

Reguli de acordare, dacă chiar trebuie:

1. **Nu umbla la bucla de curent.** Este acordată pentru electronica motorului,
   nu pentru sarcina ta.
2. Acordează dinspre interior spre exterior: viteză înainte de poziție.
3. Notează valorile inițiale înainte de a schimba ceva — tabul Parameters le
   exportă în JSON/CSV, ceea ce e mai sigur decât memoria.
4. Câștigurile `0x70xx` sunt volatile. Ce vrei să supraviețuiască alimentării
   trebuie scris în perechea din `0x20xx` și salvat cu tip 22.

---

## 5. Parametri de siguranță care merită setați o dată

| Registru | Nume | De ce contează |
|---|---|---|
| `0x7028` | `canTimeout` | watchdog: motorul se oprește dacă host-ul tace. `20000` = 1 s. **Implicit 0 = dezactivat.** Obligatoriu pentru streaming la 100 Hz |
| `0x7018` | `limit_cur` | plafonul real de forță în modurile viteză și poziție. Ține-l mic la punerea în funcțiune |
| `0x700B` | `limit_torque` | plafon global de cuplu |
| `0x7026` | `EPScan_time` | intervalul raportării active: `1` = 10 ms, fiecare unitate în plus adaugă 5 ms |

Și trei pe care **nu** le atingi: `0x2007` (limitare de cuplu), temperatura de
protecție și timpul de supratemperatură, plus `damper` (`0x702A`) — pus pe 1,
dezactivează protecția anti-backdrive de după oprirea alimentării, care există
tocmai ca să prevină supratensiuni când articulația e învârtită rapid
nealimentată. RobStride își declină răspunderea pentru daunele provocate de
modificarea lor.

---

## 6. Calibrarea — trei niveluri distincte

Cuvântul „calibrare” acoperă trei lucruri diferite, la scări diferite. Se fac
în ordinea de mai jos, pentru că fiecare îl presupune pe cel dinainte.

| Nivel | Ce stabilește | Unde trăiește | Cât de des |
|---|---|---|---|
| A. Encoder magnetic | relația dintre cele două encodere și unghiul absolut | `0x2005`, `0x2006`, flash | din fabrică; se repetă doar după desfacerea motorului sau schimbarea ordinii fazelor |
| B. Zero mecanic per motor | unde este „0 rad” pentru articulația respectivă | cadru tip 6 → flash | la montarea pe braț, după orice demontare |
| C. Offseturi cinematice | eroarea reziduală a zerourilor, văzută din vârful sculei | fișier de configurare al aplicației | la punerea în funcțiune și după coliziuni |

### A. Calibrarea encoderului magnetic

Actuatorul are două encodere: unul pe rotor și unul pe ieșire (chasu), cu
raportul reductorului între ele. La alimentare, firmware-ul le combină ca să
recupereze poziția absolută multi-tură, folosind offsetul calibrat
`chasu_offset` (`0x2006`).

**Nu se face din această aplicație și nu se scriu registrele acelea de mână.**
Sunt rezultatele procedurii de calibrare a producătorului. Scrise manual,
desincronizează bootstrap-ul de poziție și motorul se trezește crezând că e
altundeva decât este.

Ce poți face este să **diagnostichezi**, atunci când o articulație sare la
activare, raportează o poziție decalată, sau citește diferit după fiecare
ciclu de alimentare:

1. `faultSta`, bitul 7 — „encoder necalibrat” — și bitul 9 — eroare de
   inițializare a poziției. **Atenție la index: `0x3023` pe RS04, dar `0x3022`
   pe RS00 și RS03**, unde `0x3023` este `warnSta`. Caută-l după nume în tabul
   Parameters, nu după adresă.
2. `chasu_offset` (`0x2006`) — zero sau vizibil aiurea înseamnă calibrare
   neterminată sau nesalvată în flash.
3. `mech_angle_rotat` (`0x3037`), citit la **aceeași poziție fizică** după
   cinci cicluri de alimentare. Trebuie să dea de fiecare dată același număr de
   tură. Dacă rătăcește, cauza e de obicei mecanică — joc în reductor, magnet
   de encoder slăbit — sau o calibrare ratată.
4. `mech_angle_init2` (`0x3036`) față de unde se află fizic articulația. O
   nepotrivire de circa 2π/raport reductor (≈0.7 rad la 9:1) este semnătura
   unei ture rezolvate greșit.

Remediul din manual: recalibrarea encoderului magnetic. Opțional, `iq_test`
(`0x702D`) pus pe 1 lungește inițializarea în schimbul unei referințe mai
exacte.

Detalii complete despre acest bloc de registre: [PARAMETERS.md](../PARAMETERS.md),
secțiunea despre arhitectura encoderului.

### B. Zeroul mecanic al fiecărui motor

Aici lucrezi efectiv, la fiecare montaj.

**Pași:**

1. **Alege convenția de interval** — `zero_sta` (`0x7029`): `0` înseamnă
   0…2π, `1` înseamnă −π…+π. Pentru articulații care trec prin zero în ambele
   sensuri, `1` e aproape întotdeauna alegerea corectă. Setează-l **înainte**
   de a stabili zeroul, nu după.
2. **Treci motorul în CSP sau operation.** În PP firmware-ul refuză cadrul de
   zero. Verifică `0x7005` cu butonul `Read`.
3. **Adu articulația în poziția de referință.** Fie manual, cu motorul
   dezactivat, fie prin jog lent cu `limit_cur` mic până la opritorul mecanic —
   apoi retrage-te cu unghiul cunoscut până la referință. Opritorul e
   repetabil; ochiul, nu.
4. **`Set zero here`.** Trimite cadrul de tip 6 și confirmă.
5. **Verifică imediat:** poziția afișată trebuie să fie ~0. Mișcă articulația
   cu mâna într-un sens și verifică **semnul** — dacă crește invers față de
   convenția din URDF, corectează din maparea de semn a articulației, nu prin
   recalibrare.
6. **Salvează în flash** dacă vrei să supraviețuiască alimentării: `Save to
   flash` (tip 22) în tabul Parameters.
7. **Ciclu de alimentare și recitire.** Fără acest pas nu știi dacă zeroul e
   salvat sau doar volatil. Poziția trebuie să revină aceeași.

**Corecția fină** se face cu `add_offset` (`0x702B`, rad), fără să reiei
procedura: e un offset adunat la citirea de poziție. Util când zeroul e bun în
proporție de 99% și vrei să elimini ultimele zecimi de grad.

**Ordinea pe braț:** calibrează dinspre bază spre vârf. Un zero greșit la
articulația 1 mută fizic toate articulațiile de după ea, așa că orice referință
vizuală luată în aval devine falsă.

### C. Offseturile cinematice ale brațului

> **Notă:** tabul Kinematics este lucru în curs și încă nu este pe `main`.
> Secțiunea descrie procedura pe care o implementează.

Chiar cu fiecare motor zerorat corect, rămâne o eroare reziduală: toleranțe de
montaj, alinierea flanșelor, mici abateri față de modelul URDF. Ea se vede cel
mai bine acolo unde se acumulează — în vârful sculei.

Procedura este o potrivire prin cele mai mici pătrate: pui brațul în mai multe
poziții, măsori unde ajunge vârful, iar rezolvatorul găsește offseturile
articulațiilor care explică diferența față de cinematica directă.

**Pași:**

1. Încarcă URDF-ul brațului și alege cadrul vârfului.
2. Mapează fiecare articulație din URDF pe motorul ei (id CAN și semn).
3. Activează raportarea activă pe motoarele mapate — altfel citirile de
   poziție sunt vechi și eșantioanele ies false.
4. Pentru fiecare eșantion: pune brațul într-o poziție, **măsoară fizic**
   coordonatele X/Y/Z ale vârfului în cadrul bazei din URDF, tastează-le,
   apasă `Capture sample`.
5. Repetă cu poziții **bine împrăștiate**. Regula de cardinalitate: fiecare
   eșantion dă trei ecuații, deci ai nevoie de cel puțin `ceil(N/3)` poziții
   pentru N articulații — practic, mai multe, și cât mai diferite între ele.
   Poziții aproape identice dau un reziduu mic și offseturi lipsite de sens.
6. `Solve offsets`. Rezultatul arată RMS-ul erorii înainte și după, plus
   corecția per articulație. **Dacă nu a convers, nu-l aplica** — mai adaugă
   eșantioane.
7. Aplicat, offsetul poate rămâne ca strat de corecție în aplicație, sau poate
   fi „copt” în motoare: du fiecare articulație la noul zero corectat și dă
   `Set zero here`.

Criteriu de acceptare: RMS-ul de după potrivire trebuie să fie comparabil cu
precizia instrumentului de măsură. Dacă instrumentul dă ±1 mm și RMS-ul rămâne
la 8 mm, problema nu e în offseturi — e în URDF, în mapare, sau în semne.

---

## 7. Punerea în funcțiune a unui motor nou — lista scurtă

Fiecare pas e verificabil, deci o eroare îți spune unde e problema.

1. `Scan bus` → motorul răspunde la un id.
2. **Setează modelul** în coloana `Model` din panoul de conexiune. Fără el,
   toate scalările sunt greșite plauzibil.
3. `Read all` în tabul Parameters. Verifică `AppCodeVersion` (`0x1003`) —
   firmware mai vechi de 0.0.2.6 folosește P_MAX 12.5, nu 12.57.
4. Verifică `faultSta` (după nume — `0x3023` pe RS04, `0x3022` pe RS00/RS03) —
   fără defecte înainte de a activa.
5. Uită-te în tabul CAN trace: cadrele trebuie să arate sănătos.
6. Setează `limit_cur` (`0x7018`) mic și `canTimeout` (`0x7028`) pe ~1 s.
7. Mod viteză, jog foarte lent, în ambele sensuri. Aici afli dacă semnul,
   scalarea și cablajul sunt corecte.
8. Zeroul mecanic — secțiunea 6B.
9. Ciclu de alimentare, recitire, confirmare.
10. Abia acum: modul real de lucru și câștigurile.

---

## 8. Depanare rapidă

| Simptom | Cauză probabilă | Verifică |
|---|---|---|
| O comandă nu are niciun efect | motorul e în alt mod; setpoint-ul e ignorat tăcut | `0x7005` cu butonul `Read` |
| Articulația sare la activare | bootstrap de poziție greșit | `faultSta` biții 7 și 9, `0x2006`, `0x3037` |
| Poziția diferă după fiecare alimentare | zero nesalvat sau tură rezolvată instabil | tip 22 salvat? `0x3037` repetabil? |
| Valori plauzibile dar greșite | model greșit selectat | coloana `Model`; Kp/Kd diferă de 10× între RS00–02 și RS03/04 |
| Motorul nu răspunde la scanare | comutat pe CANopen sau MIT | tip 25, revenire pe protocol privat, apoi ciclu de alimentare |
| Limita de viteză e ignorată în poziție | registru greșit pentru mod | PP folosește `0x7024`, CSP folosește `0x7017` |
| Motorul rămâne pe ultima comandă când host-ul tace | watchdog dezactivat | `0x7028`, implicit 0 |
| Oscilație la oprirea în poziție | `loc_kp` prea mare pentru sarcină | `0x701E`, coboară-l |

---

## 9. Unități

Selectorul `Angles` din bara de instrumente comută **doar afișarea** între
grade și radiani. Protocolul, tabelele de parametri și toate registrele
`0x70xx` rămân în radiani, iar exportul JSON/CSV este **întotdeauna** în
radiani, indiferent ce se vede pe ecran. Când verifici o valoare față de
manualul RobStride, comută pe radiani în loc să convertești în cap.

---

## 10. Ce nu a fost verificat pe hardware

Aplicația nu a fost niciodată rulată împotriva unui motor real: nu exista
adaptor sau driver PCAN pe mașina pe care a fost scrisă. Decodarea
protocolului, împachetarea ID-urilor și constantele de scalare sunt verificate
automat față de exemplele din manuale, dar stratul de transport, scanarea
magistralei și toate comenzile de motor sunt neîncercate.

Concret, pentru procedurile de mai sus: pornește cu **un singur motor pe
banc**, urmărește tabul CAN trace ca să confirmi că fiecare cadru arată cum
trebuie, și abia apoi conectează un braț întreg.
