# Arbeidsfordeling og leveranser

## 👤 Person 1 – Kamera og kube-deteksjon

**Ansvar:** Utvikle og teste kamera-pipeline for bildeinnhenting og fargedeteksjon av kuber.

**Oppgaver:**
- Konfigurere og teste kamera
- Utvikle `camera_node`
- Detektere røde, gule og blå kuber i bildet
- Transformere bildekoordinater til posisjoner
- Publisere kube-posisjoner med egendefinert melding
- Kalibrere og justere fargeterskler via `camera/config/camera_params.yaml`

**Leveranser:**
- `src/camera/`

---

## 👤 Person 2 – Robotkontroller

**Ansvar:** Kontrollere robotens bevegelser og implementere bevegelsesnoder.

**Oppgaver:**
- Lage `robot_controller_node`
- Flytte til home-posisjon og kube-posisjoner

**Leveranser:**
- `src/robot_controller/`

---

## 👤 Person 3 – Oppgavelogikk og feilhåndtering

**Ansvar:** Koordinere flyt og sekvenser, og håndtere avvik i systemet.

**Oppgaver:**
- Utvikle `task_manager_node` (src/task_manager/src/task_manager_node.cpp)
- Sekvens: oversiktsbilde → deteksjon → peking på kuber
- Håndtere feilsituasjoner, f.eks. manglende kube
- Starte fallback-strategier (ta nytt bilde)

**Leveranser:**
- `src/task_manager/`

---

## 👤 Person 4 – Infrastruktur, testing og dokumentasjon

**Ansvar:** Oppsett, integrasjon, testing og dokumentasjon av systemet.

**Oppgaver:**
- Lage og vedlikeholde launch-filer
- Lage egendefinert meldingsstruktur
- Dokumentere struktur og bygge README.md
- Holde Git-repo strukturert
- Koordinere systemintegrasjon og gjennomføre systemtester

**Leveranser:**
- `src/custom_interfaces/`
- README.md
- PLAN.md

---