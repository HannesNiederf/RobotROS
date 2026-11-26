-- Schema of softwarespezifikation.md section 12 (verbatim apart from formatting).
CREATE TABLE maps (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL, yaml_path TEXT NOT NULL,
  resolution REAL, created_at TEXT, is_active INTEGER DEFAULT 0);
CREATE TABLE stations (id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL REFERENCES maps(id), name TEXT NOT NULL,
  kind TEXT NOT NULL CHECK(kind IN ('STORAGE','DOCK')), x REAL, y REAL, yaw REAL,
  approach_distance REAL DEFAULT 0.40, enabled INTEGER DEFAULT 1, UNIQUE(map_id, name));
CREATE TABLE carts (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL,
  station_id INTEGER REFERENCES stations(id),
  state TEXT NOT NULL CHECK(state IN ('STORED','CARRIED','UNKNOWN')));
CREATE TABLE orders (id INTEGER PRIMARY KEY AUTOINCREMENT,
  type TEXT NOT NULL CHECK(type IN ('TRANSPORT','GOTO_POINT','GOTO_DOCK')),
  origin TEXT DEFAULT 'USER', priority INTEGER DEFAULT 5 CHECK(priority BETWEEN 0 AND 9),
  source_station_id INTEGER REFERENCES stations(id), dest_station_id INTEGER REFERENCES stations(id),
  goal_x REAL, goal_y REAL, goal_yaw REAL, cart_id INTEGER REFERENCES carts(id),
  state TEXT NOT NULL, step TEXT, error_code TEXT, error_detail TEXT, attempts INTEGER DEFAULT 0,
  created_at TEXT, started_at TEXT, finished_at TEXT);
CREATE TABLE order_events (id INTEGER PRIMARY KEY AUTOINCREMENT, order_id INTEGER REFERENCES orders(id),
  ts TEXT, event TEXT, detail TEXT);
CREATE TABLE keepout_zones (id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL REFERENCES maps(id),
  source TEXT NOT NULL CHECK(source IN ('MANUAL','CLIFF_AUTO')), polygon_json TEXT NOT NULL, note TEXT, created_at TEXT);
CREATE TABLE contact_points (id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL REFERENCES maps(id),
  x REAL, y REAL, hits INTEGER DEFAULT 1, permanent INTEGER DEFAULT 0, first_ts TEXT, last_ts TEXT);
CREATE TABLE robot_state (id INTEGER PRIMARY KEY CHECK(id=1), lifted INTEGER DEFAULT 0, active_order_id INTEGER,
  last_x REAL, last_y REAL, last_yaw REAL, updated_at TEXT);
CREATE TABLE settings (key TEXT PRIMARY KEY, value TEXT);
