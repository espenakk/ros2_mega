# ros2_mega

Program for å få en UR-robot til å:
- Ta bilde av fargede kuber på bordet
- Detektere røde, gule og blå kuber
- Peke på kubene i riktig rekkefølge (rød → gul → blå)

---

## 📂 Prosjektstruktur

```bash
task_manager/
├── setup.py
├── setup.cfg
├── package.xml
├── resource/
│   └── task_manager
├── task_manager/
│   ├── __init__.py
│   ├── camera_node/
│   │   ├── __init__.py
│   │   ├── camera_node.py
│   │   └── cube_detection.py
│   ├── robot_controller_node/
│   │   ├── __init__.py
│   │   ├── robot_controller_node.py
│   │   └── move_to_position.py
│   └── task_manager_node/
│       ├── __init__.py
│       └── task_manager_node.py
├── config/
│   ├── camera_params.yaml
│   └── robot_params.yaml
├── launch/
│   ├── bringup.launch.py
│   ├── camera_pipeline.launch.py
│   └── move_to_home.launch.py
├── rviz/
│   └── cube_detection.rviz
├── urdf/
│   └── (hvis aktuelt)
├── README.md
```

---

## 🧩 ROS2 Noder

| Node | Ansvar |
|:---|:---|
| `camera_node` | Tar bilder og kjører fargedeteksjon. Publiserer posisjoner for kubene. |
| `robot_controller_node` | Flytter roboten til home-posisjon, mellomposisjoner og kubeposisjoner. |
| `task_manager_node` | Styrer rekkefølgen, feilhåndtering og oppgavelogikk. |
| `utils` | Hjelpefunksjoner for koordinattransformasjoner og vision-verktøy. |

---

## 🚀 Launch-filer

| Launch-fil | Ansvar |
|:---|:---|
| `bringup.launch.py` | Starter hele systemet: kamera, robot og task manager. |
| `camera_pipeline.launch.py` | Starter bare kamera og deteksjonssystem for testing. |
| `move_to_home.launch.py` | Flytter roboten til definert home-posisjon. |

---

## ⚙️ Konfigurasjonsfiler

| Fil | Innhold |
|:---|:---|
| `camera_params.yaml` | Parametere for kamera (f.eks. intrinsics, HSV-toleranser for farger). |
| `robot_params.yaml` | Robotens home-posisjon, hastigheter, akselerasjoner osv. |

---

## 👥 Arbeidsfordeling

| Person | Ansvar | Filer |
|:---|:---|:---|
| 1 | Kamera-node og fargedeteksjon | `camera_node.py`, `cube_detection.py` |
| 2 | Robot-kontroller | `robot_controller_node.py`, `move_to_position.py` |
| 3 | Task manager | `task_manager_node.py`, hjelpefiler i `utils/` |
| 4 | Launch-filer, konfigurasjon, testing, RViz, dokumentasjon | `*.launch.py`, `*.yaml`, `*.rviz`, `README.md` |

---

## 👥 Roller og ansvarsområder

Dette prosjektet er delt mellom fire hovedansvarsområder for å sikre effektivt samarbeid og god modulstruktur i ROS2.

### 👤 Person 1 – Kamera og kube-deteksjon

**Ansvar:** Utvikle og teste kamera-pipeline for bildeinnhenting og fargedeteksjon av kuber.

#### Oppgaver
- Konfigurere og teste kamera (RealSense, USB-kamera, o.l.)
- Utvikle `camera_node` som publiserer bilder
- Detektere røde, gule og blå kuber i bildet
- Transformere bildekoordinater til reelle posisjoner
- Publisere kube-posisjoner med `PoseStamped` eller egendefinert melding
- Kalibrere og justere fargeterskler via `camera_params.yaml`

#### Leveranser
- `src/camera_node/`
- `config/camera_params.yaml`
- Testverktøy for visualisering
- Rapport-del om visjonssystemet

---

### 👤 Person 2 – Robotkontroller og MoveIt

**Ansvar:** Kontrollere robotens bevegelser via MoveIt og implementere bevegelsesnoder.

#### Oppgaver
- Konfigurere robot og MoveIt-tilkobling
- Lage `robot_controller_node` for å motta posisjoner og utføre bevegelser
- Flytte til home-posisjon, mellomposisjoner og kube-posisjoner
- Lage og vedlikeholde `robot_params.yaml`
- Koble til RViz for testing og visualisering

#### Leveranser
- `src/robot_controller_node/`
- `config/robot_params.yaml`
- `launch/move_to_home.launch.py`
- Rapport-del om bevegelsesstrategier

---

### 👤 Person 3 – Oppgavelogikk og feilhåndtering

**Ansvar:** Koordinere flyt og sekvenser, og håndtere avvik i systemet.

#### Oppgaver
- Utvikle `task_manager_node` som styrer hele operasjonen
- Sekvens: oversiktsbilde → deteksjon → peking på kuber
- Håndtere feilsituasjoner, f.eks. manglende kube
- Starte fallback-strategier (flytt kamera, ta nytt bilde)
- Publisere statusmeldinger og/eller bruke ROS2-tjenester

#### Leveranser
- `src/task_manager_node/`
- System- og beslutningslogikk
- Diagram og refleksjon i rapporten

---

### 👤 Person 4 – Infrastruktur, testing og dokumentasjon

**Ansvar:** Oppsett, integrasjon, testing og dokumentasjon av systemet.

#### Oppgaver
- Lage og vedlikeholde alle launch-filer
- Sette opp RViz-konfigurasjon for visning av data
- Dokumentere struktur og bygge README.md
- Holde Git-repo strukturert med brancher, issues og pull requests
- Koordinere systemintegrasjon og gjennomføre systemtester

#### Leveranser
- `launch/`, `rviz/`, `README.md`
- GitHub-repo
- Testbeskrivelser og prosedyrer
- Struktur- og samarbeidsseksjon i rapporten

---

## ✨ Mulige Ekstrafunksjoner

- Automatisk leting etter kuber ved manglende deteksjon
- Systemstatus-varsling ved feil
- Implementering av sikkerhetssoner
- Plukke opp og sortere kuber
- GUI-plugin i RViz for visuell tilbakemelding

---

## 📚 Rapportstruktur

- **Introduksjon**: Problemstilling og målsetning
- **Teori**: Relevant bakgrunn for brukte metoder
- **Systembeskrivelse**: Arkitektur, noder og datastrøm
- **Resultater**: Testresultater og målinger
- **Diskusjon**: Vurdering av resultater mot mål
- **Gruppedynamikk**: Hvem gjorde hva, refleksjoner
- **Konklusjon**: Oppsummering av arbeidet

---

# 🚀 Kjøring av prosjektet

1. Koble roboten og nettbrettet som anvist.
2. Start robotens kontroll-node:
    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX robot_ip:=YYY.YYY.YYY.YYY use_mock_hardware:=false initial_joint_controller:=joint_trajectory_controller headless_mode:=true
    ```
3. Kjør prosjektet:
    ```bash
    ros2 launch task_manager bringup.launch.py
    ```

---

# Commands

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=143.25.150.7 use_mock_hardware:=true initial_joint_controller:=scaled_joint_trajectory_controller kinematics_params_file:="${HOME}/my_robot_calibration.yaml" launch_rviz:=false
```
