# Chosen here (ADR 0004), not fixed by the spec: plain-language activity of the robot (topic /activity).
# Keys are i18n keys of web_ui/i18n/<lang>.json (spec 13.4); args are "name=value" for the {name} placeholders.
std_msgs/Header header
string now_key       # what the robot does now, e.g. "activity.driving"
string[] now_args    # e.g. ["speed=0.12"]
string plan_key      # what it intends, e.g. "plan.goto_eta"
string[] plan_args   # e.g. ["dist=1.9", "eta=16", "x=2.5", "y=-0.8"]
