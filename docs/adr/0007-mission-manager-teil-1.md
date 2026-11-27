# ADR 0007: mission_manager Teil 1 (Aufträge, Stationen, Software-Not-Stopp)

Status: angenommen (Claude, 26.11.2026; „großer Block“, mit Hannes am 25.11. vereinbart)

## Inhalt
- Paket `robotros_mission`, Knoten `mission_manager` (Spezifikation §9, §12). Logik ohne ROS in
  `core/mission.py` (Zustandsautomat, Warteschlange) und `core/store.py` (SQLite), Einheitstests in
  `test/test_mission.py`.
- **Datenbank:** Schema wie §12 (`sql/001_initial.sql`), WAL, `foreign_keys`, `synchronous=FULL`, Version in
  `settings.schema_version`. Pfad `db_path` aus `mission.yaml` (`/var/lib/robotros/robotros.db`). Die Simulation
  verwendet `~/.robotros/sim.db` (Startargument `db`), Prüfläufe mit `--check` je eine frische Datei unter
  `build/sim_start/`. Karte: Name = Dateiname der Karten-YAML ohne Endung; Stationen gehören zu dieser Karte.
- **Ergänzung des Schemas:** `sql/002_cancel_return.sql` fügt `orders.cancel_requested` hinzu. Grund: Ein
  abgebrochener Transport mit Wagen an Bord bringt den Wagen erst zur Quelle zurück (§9.3) und muss das auch nach
  einem Neustart wissen.
- **Stationsdienste:** `/map/upsert_station` und `/map/delete_station` bietet der `mission_manager` an, nicht der
  `map_manager`, weil nur der Missionsknoten in die Datenbank schreibt (D-28). Er veröffentlicht `/web/stations`;
  der `map_manager` übernimmt die Liste in `/map/get_snapshot`. Beim Anlegen setzt `occupied` die Belegung (Wagen
  `cart@<Stationsname>`, falls kein Name angegeben ist). Stationen in offenen Aufträgen lassen sich weder ändern
  noch löschen (E309).
- **Aufträge:** `TRANSPORT` und `GOTO_POINT` mit den Prüfungen aus §9.3 (E301–E304, E309, E310), Reservierung,
  Sortierung nach Priorität und Anlegereihenfolge (Datenbank-ID statt `created_at`, damit keine Uhrzeit in die
  Logik eingeht), erneute Prüfung beim Start, Abbrechen (mit Wagen zurück zur Quelle), `RETRY`, `SET_PRIORITY`,
  `RESUME_AT_STEP`. Ablauf TRANSPORT: `P_pre` der Quelle (ÄM-01 D-33) → `/docking/engage_cart` → `P_pre` des
  Ziels → `/docking/release_cart`.
- **Sicherheit:** Fällt der Sicherheitszustand aus ARMED, bricht die Mission die laufende Fahrt ab (PAUSED_SAFETY).
  Wird der Roboter **ohne** Quittierdialog wieder scharf geschaltet, bleibt der Auftrag angehalten (AWAIT_RESUME),
  bis der Bediener fortsetzt; nichts fährt überraschend los. `/mission/software_estop` (Trigger) bricht ab und ruft
  `/hw/set_arm(false)`; `/mission/ack_safety` schaltet über `/hw/set_arm(true)` scharf und setzt je nach
  `resume_mode` fort, hält an oder bricht ab. Die Sicherheitslogik im Nano bzw. `fake_nano` bleibt unverändert.
- **Fehler:** ohne Wagen → DEFERRED + Meldung; mit Wagen → HELD_WITH_CART mit Optionen RETRY und
  RETURN_TO_SOURCE; fehlgeschlagener Wagenschritt → NEEDS_OPERATOR mit RETRY und CANCEL_ORDER (CANCEL_ORDER heißt:
  der Bediener hat geprüft, dass kein Wagen an Bord ist). Neustart während einer Fahrt → AWAIT_RESUME, während
  eines Wagenschritts → NEEDS_OPERATOR (§9.2).
- **Hier festgelegt (nicht in der Spezifikation):** Ziel eines GOTO_POINT außerhalb einer freien Kartenzelle
  (Wert 0–25) → E203. Nav2-Abbruch → E203 mit der Nav2-Fehlernummer im Text. Ohne erreichbaren Docking-Server →
  Auftrag DEFERRED mit E305 (es hat sich nichts bewegt). Lokalisiert = `/robot_pose` jünger als 2 s.

## Noch nicht enthalten
Docking und Laden (GOTO_DOCK, IDLE → Dock, CHARGING), der Docking-Server selbst (Einfahren/Heben), SetMode
(MANUAL), Sperrzonen (E205), Kontakt-Recovery (§6.4), Lokalisierungsgüte (E201/E202, `loc_sigma_xy` = −1),
Zurückfahren nach Akku-Warnung, Ereignis „Is a cart on board?“ nach dem Boot, tägliches Backup.
Die Web-Oberfläche nutzt die Dienste noch nicht (folgt: Not-Stopp-Knopf, Seiten Aufträge und Stationen).

## Prüfung
`test/test_mission.py` (ohne ROS) und `robotros_sim/scripts/mission_check.py` in `tools/sim_start.sh nav --check`
(CI): Stationen anlegen, E303/E301, Transport bis zur Vorprüfposition der Quelle und DEFERRED mit E305,
GOTO_POINT mit Software-Not-Stopp, Quittieren und Fortsetzen, Abbrechen eines wartenden Auftrags.
