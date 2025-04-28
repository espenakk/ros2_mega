# ros2_mega

Program for å få en UR-robot til å:
- Ta bilde av fargede kuber på bordet
- Detektere røde, gule og blå kuber
- Peke på kubene i riktig rekkefølge (rød → gul → blå)

---

## 📂 Prosjektstruktur

```bash
ur_cube_pointer/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── bringup.launch.py
│   ├── camera_pipeline.launch.py
│   └── move_to_home.launch.py
├── config/
│   ├── camera_params.yaml
│   └── robot_params.yaml
├── src/
│   ├── camera_node/
│   │   ├── camera_node.cpp
│   │   └── cube_detection.cpp
│   ├── robot_controller_node/
│   │   ├── robot_controller_node.cpp
│   │   └── move_to_position.cpp
│   ├── task_manager_node/
│   │   └── task_manager_node.cpp
│   └── utils/
│       ├── transformations.cpp
│       └── vision_helpers.cpp
├── urdf/
│   └── (hvis aktuelt)
├── rviz/
│   └── cube_detection.rviz
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

## 👥 Arbeidsdeling

| Person | Ansvar | Filer |
|:---|:---|:---|
| 1 | Kamera-node og fargedeteksjon | `camera_node.cpp`, `cube_detection.cpp` |
| 2 | Robot-kontroller | `robot_controller_node.cpp`, `move_to_position.cpp` |
| 3 | Task manager | `task_manager_node.cpp`, hjelpefiler i `utils/` |
| 4 | Launch-filer, konfigurasjon, testing, RViz, dokumentasjon | `*.launch.py`, `*.yaml`, `*.rviz`, `README.md` |

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
    ros2 launch ur_cube_pointer bringup.launch.py
    ```

---
