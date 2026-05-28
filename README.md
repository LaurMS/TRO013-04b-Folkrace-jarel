# Moodul 04b: Lidar-põhine iseseisev sõitmine

> **Järeleaitamine** — see repo on mõeldud tudengitele, kes teevad ülesande hiljem.
> Täpsem õppematerjal on Moodle'is: **Moodul 04: Lidar-põhine iseseisev sõitmine**.

---

## Samm 1 — Forki see repo

1. Mine selle repo lehele GitHubis
2. Kliki paremal ülal nuppu **Fork**
3. Vali oma GitHub konto
4. Kliki **Create fork**

Sinu isiklik koopia tekib:
```
https://github.com/SINU-KASUTAJANIMI/TRO013-04b-Folkrace-jarel
```

---

## Samm 2 — Klooni konteinerisse

Ava noVNC terminal (`http://SERVER_IP:33000+N`) ja käivita:

```bash
cd /workspace/ros2_ws/src
git clone https://github.com/SINU-KASUTAJANIMI/TRO013-04b-Folkrace-jarel.git
cd TRO013-04b-Folkrace-jarel
```

---

## Samm 3 — Tee ülesanne

Kopeeri mall oma töökausta:

```bash
cp /workspace/ros2_ws/install/yahboom_webots/share/yahboom_webots/scripts/folkrace_juht.py \
   /workspace/ros2_ws/src/TRO013-04b-Folkrace-jarel/folkrace_juht.py
```

Muuda `_arvuta_kiirus()` funktsiooni nii, et robot sõidab folkrace rajal iseseisvalt.

**Nõuded:**
- Robot teeb vähemalt **ühe täisringi** ilma seintesse põrkamata
- Robot **reguleerib kiirust** (kaugel = kiire, lähedal = aeglane)
- Robot suudab üle **silla** minna

**Käivitamine:**
```bash
source /opt/mobros_ws/install/setup.bash

# Terminal 1 — simulatsioon
ros2 launch yahboom_webots webots.launch.py

# Terminal 2 — sinu skript
python3 /workspace/ros2_ws/src/TRO013-04b-Folkrace-jarel/folkrace_juht.py
```

**Silumiseks** (teises terminalis):
```bash
python3 /workspace/ros2_ws/install/yahboom_webots/share/yahboom_webots/scripts/folkrace_monitor.py
```

---

## Samm 4 — Commit ja push

```bash
cd /workspace/ros2_ws/src/TRO013-04b-Folkrace-jarel
git add .
git commit -m "Moodul 04b: folkrace_juht implementeeritud"
git push
```

> Git küsib parool — kasuta GitHubi **Personal Access Token**:
> GitHub → Settings → Developer settings → Personal access tokens → Generate new token (scopes: `repo`)

---

## Samm 5 — Vaata hindamistulemusi

Pärast push-i käivitub automaatne hindamine (~1-2 min).

**Vaata tulemusi:** oma repo → **Actions** → viimane töö

Hindamine otsib `folkrace_juht.py` faili **kõikidest kaustadest** ja kontrollib koodi sisu (lidar lugemine, kiiruse reguleerimine, pööramiseloogika).
