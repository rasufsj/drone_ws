import math
from pathlib import Path

import yaml
from jinja2 import Environment, FileSystemLoader, StrictUndefined

BASE = Path(__file__).resolve().parent  # .../models

# carrega ctx.yaml
ctx = yaml.safe_load((BASE / "ctx.yaml").read_text(encoding="utf-8")) or {}

env = Environment(
    loader=FileSystemLoader(str(BASE)),  # inclui a partir de models/
    undefined=StrictUndefined,
    autoescape=False,
)

# IMPORTANT: disponibiliza para TODOS os templates/macros/imports
env.globals["math"] = math

tpl = env.get_template("mrs_robots_description/sdf/drones/f450.sdf.jinja")
out = tpl.render(**ctx)

out_path = BASE / "mrs_robots_description/sdf/drones/f450.sdf"
out_path.write_text(out, encoding="utf-8")
print(f"Gerado: {out_path}")