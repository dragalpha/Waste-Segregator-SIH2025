# Optional utility functions for YOLO
# ai/yolo/utils_yolo.py
# Mapping from object label -> (category, subtype)
WASTE_MAP = {
	# DRY
	"plastic": ("dry", "plastic"),
	"bottle": ("dry", "plastic"),
	"cup": ("dry", "plastic"),
	"wrapper": ("dry", "plastic"),
	"bag": ("dry", "plastic"),
	"packet": ("dry", "plastic"),
	"paper": ("dry", "paper"),
	"book": ("dry", "paper"),
	"notebook": ("dry", "paper"),
	"cardboard": ("dry", "cardboard"),
	"carton": ("dry", "cardboard"),
	"box": ("dry", "cardboard"),
	"can": ("dry", "metal"),
	"tin": ("dry", "metal"),
	"metal": ("dry", "metal"),
	"glass": ("dry", "glass"),
	"jar": ("dry", "glass"),
	"textile": ("dry", "textile"),
	"cloth": ("dry", "textile"),
	"fabric": ("dry", "textile"),
	"electronic": ("dry", "e-waste"),
	"charger": ("dry", "e-waste"),

	# WET
	"food": ("wet", "food"),
	"fruit": ("wet", "food"),
	"vegetable": ("wet", "food"),
	"banana": ("wet", "food"),
	"peel": ("wet", "food"),
	"leaf": ("wet", "organic"),
	"plant": ("wet", "organic"),
	"organic": ("wet", "organic"),
	"liquid": ("wet", "liquid"),

	# HAZARD
	"battery": ("hazard", "battery"),
	"broken_glass": ("hazard", "broken_glass"),
	"broken": ("hazard", "broken_glass"),  # generic
	"needle": ("hazard", "sharp"),
	"knife": ("hazard", "sharp"),
	"syringe": ("hazard", "medical"),
	"mask": ("hazard", "medical"),
	"chemical": ("hazard", "chemical"),
	"paint": ("hazard", "chemical"),
	"acid": ("hazard", "chemical"),
}

def classify_label(label: str):
	"""
	Map a model label string to (category, subtype).
	Returns ("unknown", "unknown") if no mapping present.
	"""
	if not label:
		return ("unknown", "unknown")
	key = label.lower().replace(" ", "_")
	return WASTE_MAP.get(key, ("unknown", "unknown"))
