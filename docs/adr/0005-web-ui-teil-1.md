# ADR 0005: Web-Oberfläche, erster Teil (Kopfzeile und Kartenseite)

Status: angenommen (Claude, 25.11.2026; Reihenfolge vereinbart mit Hannes am 24.11., „weiter“ am 25.11.)

## Inhalt
- **`robotros_web`** (Spec §3.2): Der Knoten `web_server` liefert `web_ui/` statisch auf Port 8080 aus.
  `rosbridge_websocket` läuft auf Port 9090. Beides startet über `launch/web.launch.py`; die Simulation startet
  es mit `web:=true` (macht `tools/sim_start.sh`).
- **`robotros_map` / `map_manager`, erster Teil:**
  - aus `/map` werden `/web/map_image` (PNG, Graustufen, eigener PNG-Kodierer in `core/map_image.py`) und
    `/web/map_meta` (bei Änderung und alle 5 s);
  - `/robot_pose` aus TF map → base_footprint (5 Hz);
  - Dienst `/map/get_snapshot`, vorerst ohne Stationen, Zonen und Kontaktpunkte.
  
  Kartendateien, Moduswechsel, Stationen usw. folgen.
- **`web_ui/`** (Spec §13.1/13.3, nur ein Teil):
  - Kopfzeile mit Verbindung (inklusive Overlay „Verbindung verloren“ und Wiederverbinden 1 → 5 s),
    Sicherheitszustand, „Jetzt / Vorhaben“ (`/activity`, ADR 0004) und Sprachwahl EN/DE/IT (gespeichert in
    `localStorage`).
  - Kartenseite mit Karte, Roboterpfeil und geplantem Weg; ein Ziel wird durch Antippen und Bestätigen gesetzt.
  - Größen nach §13.1: Touchziele ≥ 56 px, Schrift ≥ 18 px, ausgelegt für 1368 × 912.

## Abweichungen und Zwischenlösungen
- **Ziel per Antippen → `/goal_pose`** (Nav2 `bt_navigator`, wie RViz). Die Spezifikation sieht einen Auftrag
  `GOTO_POINT` über den `mission_manager` vor. Den gibt es noch nicht; sobald er existiert, wird umgestellt.
  Als Blickrichtung am Ziel gilt die Richtung vom Roboter zum Ziel (hier gewählt).
- **Kein Not-Stopp-Knopf in der Kopfzeile.** Er soll `/mission/software_estop` aufrufen, und diesen Dienst gibt es
  noch nicht. Ein Knopf ohne Wirkung wäre gefährlich, deshalb fehlt er bewusst. Der Not-Aus-Taster am Roboter und
  der Collision Monitor bleiben unverändert.
- **`eventemitter2`** liegt nicht als eigene Datei in `vendor/`: Das Bündel `roslib.min.js` 1.4.1 enthält es
  bereits. Herkunft, Prüfsumme und Lizenz stehen in `web_ui/vendor/README.md`.
- `/plan` abonniert die UI direkt, gedrosselt auf 1 s. Die Spezifikation nennt dafür kein eigenes Web-Topic.

## Tests
- `robotros_map/test/test_map_image.py`: PNG-Kodierung, Zeilenreihenfolge, Werte.
- `web_ui/test/*.test.mjs` (`node --test`): Kartenumrechnung und i18n-Platzhalter mit Dezimalkomma.
- `tools/web_ui_check.py` (Playwright, Chromium ohne Fenster), in der CI innerhalb von
  `sim_start.sh nav --check`. Geprüft wird: verbunden, Karte und Pose da, Zustand „Bereit“, Tätigkeitszeile gefüllt,
  Antippen → Bestätigen → der Roboter fährt zum angetippten Punkt, Sprachwechsel, keine JavaScript-Fehler.
