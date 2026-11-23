# Spec: softwarespezifikation.md section 5.1 and 9.5 (topic /mission/alerts).
string SEVERITY_INFO="INFO"
string SEVERITY_WARN="WARN"
string SEVERITY_ERROR="ERROR"

uint32 id
string severity
string code               # error code, e.g. E204
string detail
bool needs_action
string[] options
