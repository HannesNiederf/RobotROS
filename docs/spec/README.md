# Spezifikation robotROS

Dieser Ordner enthält die verbindliche Spezifikation des Projekts. Die Dokumente sind Kopien aus dem
claude.ai-Projekt „RobotROS“ (Stand der Originale: 18.11.2026, kopiert am 24.11.2026).

| Datei | Inhalt | Stand |
|---|---|---|
| `softwarespezifikation.md` | Entscheidungsdokument D-01 bis D-28: Architektur, ROS-Schnittstellen, Sicherheit, Navigation, Missionslogik, Fehlercodes, Datenmodell, UI, Simulation, Betrieb, REQ-Anforderungen | v1.0 |
| `spez_aenderung_01_rueckwaerts.md` | **ÄM-01, hat Vorrang:** Rückwärtseinfahrt unter die Gitterbox, Sektormaske des Lidar, D-29 bis D-38 | Revision 2 |
| `spez_firmware_protokoll.md` | Nano-Firmware, serielles Protokoll Version 1, Sicherheitsautomat, Tests | v1.0 |
| `spez_parameter_tests.md` | Parameterkatalog, Kalibrierung C1–C13, Teststrategie, Abnahmekriterien M1–M3 | v1.0 |
| `pinplan_nano.md` | Pin-Budget, Varianten A/B/C, Sicherheitsrelais mit Quittierung | Version 3 (Entwurf) |

## Umbenennung in den Kopien
Die Originale verwenden noch den früheren Arbeitsnamen. In den Kopien wurde er nach
`docs/adr/0001-projektname-robotros.md` ersetzt (Paketpräfix `robotros_`, Dienst `robotros.service`,
Pfade `/var/lib/robotros/`, `/opt/robotros/`, Datenbank `robotros.db`, Variablen `ROBOTROS_MODE`,
`ROBOTROS_DEV`, Hostname `robotros`, udev-Namen `robotros_nano_*`, Repository `RobotROS`, Anzeigename robotROS).
Sonst ist der Text unverändert, auch dort, wo er inzwischen überholt ist (siehe Errata).

Verweise der Form `claude/<datei>.md` meinen das claude.ai-Projekt. Für die fünf Dateien oben gilt die
Kopie in diesem Ordner. `projektwissen.md`, `bauteile_auswertung.md` und `anleitung_github.md` liegen
nur im claude.ai-Projekt.

## Rangfolge bei Widersprüchen
Die jüngere Änderung gilt:
1. D-39 (Surface mit Ubuntu 24.04, siehe unten) und Runde 26 (unten),
2. ÄM-01 (`spez_aenderung_01_rueckwaerts.md`),
3. `softwarespezifikation.md`, `spez_firmware_protokoll.md`, `spez_parameter_tests.md`,
4. `projektwissen.md` (nur im claude.ai-Projekt).

Hardwarefragen (Pins, Teile) bleiben in `pinplan_nano.md` und `bauteile_auswertung.md`.
Neue Entscheidungen, die die Spezifikation ändern, kommen als ADR nach `docs/adr/`.

## Spätere Entscheidungen, die noch nicht in den Dokumenten stehen
- **D-39 (23.11.2026):** Das Surface läuft mit Ubuntu 24.04.5, linux-surface-Kernel und ROS 2 Jazzy Desktop
  (Neuinstallation).
- **Pi 5 (Entscheidung Claude, 23.11.2026):** Ubuntu Server 24.04 arm64 mit ROS 2 Jazzy `ros-base`.
- **Runde 26 (18.11.2026):**
  - Der Wagen ist rechteckig und steht auf 4 festen Füßen. Die Simulation nimmt den Wagen mit vier Füßen als
    Standard. `cart_occlusion_mode` bleibt vorerst `SECTOR`, die Umstellung auf `BOX` folgt nach der Messung C14.
  - Roboterbreite 300 mm: `robot_width` = 0,30 m, `footprint_robot` = 0,30 m (Länge offen). Der Wagen braucht
    damit mindestens 360 mm lichte Weite zwischen den Füßen. Platzhalter für die Simulation: Wagen 0,40 m außen,
    `footprint_cart` = 0,42 m.
  - Das Wettbewerbsdatum ist nur als „Frühjahr“ bekannt; geplant wird auf den frühesten Fall (ab ca. 20. März 2027).

## Errata (Verweis- und Standfehler, keine inhaltliche Änderung)
- `softwarespezifikation.md` §10, Einleitung: „7.3 Zeile 3“ meint **§3.3** (Datenfluss, Multiplexer B hinter
  dem Collision Monitor).
- `softwarespezifikation.md` §3.2: Der Knoten `status_signals` (`spez_firmware_protokoll.md` §8.2) gehört zum
  Paket `robotros_mission`.
- `softwarespezifikation.md` §15.2 und §16: Das Surface läuft mit Ubuntu 24.04 (D-39). Das Repository existiert
  und heißt `RobotROS`; der Baum in §16 zeigt noch den alten Namen des Wurzelordners.
- `spez_parameter_tests.md` §2.1: `robot_width` 0,30 m, `footprint_robot` 0,30 m, `footprint_cart` 0,42 m
  (Platzhalter nach Runde 26), statt 0,26 / 0,26 / 0,32.
- `spez_parameter_tests.md` §10 Gruppe G und §11 Punkt 1: Das GitHub-Repository ist angelegt und verbunden.
