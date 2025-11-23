#!/usr/bin/env python3
"""Telemetry CSV migration tool

Usage:
  python backend/tools/migrate_telemetry.py

What it does:
- Detects the current `backend/data/telemetry.csv` header.
- If header already matches the canonical header, exits.
- Otherwise backs up the original to `telemetry.csv.bak.<ts>` and writes
  a new `telemetry.csv` with the canonical header and migrated rows.

Mapping rules (best-effort):
- timestamp -> timestamp_utc
- lat, lon -> lat, lon
- compass -> heading_deg
- tds -> tds_ppm
- ultrasonic -> ultrasonic_cm
- image_path -> image_path
- waste -> yolo_raw (and classify to waste_category if possible)
- waste_type -> waste_subtype

This is a non-destructive migration: original file is backed up.
"""
import csv
import sys
from pathlib import Path
import datetime
import argparse
import shutil

ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = ROOT / "backend" / "data" if (ROOT / "backend" / "data").exists() else ROOT / "data"
CSV_FILE = DATA_DIR / "telemetry.csv"

CANONICAL_HEADER = [
    "timestamp_utc",
    "device_id",
    "boat_id",
    "lat",
    "lon",
    "heading_deg",
    "mq135_ppm",
    "mq2_ppm",
    "soil_dry_belt_pct",
    "soil_wet_belt_pct",
    "loadcell_grams",
    "tds_ppm",
    "ultrasonic_cm",
    "proximity_inductive",
    "image_path",
    "yolo_raw",
    "waste_category",
    "waste_subtype",
    "collection_event",
    "collection_bin_id",
    "battery_volt",
    "rssi",
]


def try_import_classifier():
    """Try to import classify_label from yolo utils; return None on failure."""
    try:
        # ensure project root is on path
        import sys
        sys.path.insert(0, str(ROOT / "backend" / "ai"))
        from yolo.utils_yolo import classify_label
        return classify_label
    except Exception:
        try:
            sys.path.insert(0, str(ROOT / "backend"))
            from ai.yolo.utils_yolo import classify_label
            return classify_label
        except Exception:
            return None


def migrate_row(old_row, old_header, classify_label_fn=None):
    # create dict for canonical row with empty defaults
    dest = {k: "" for k in CANONICAL_HEADER}

    # helper to get value if present
    hmap = {h.lower(): i for i, h in enumerate(old_header)}

    def get_old(key):
        i = hmap.get(key)
        if i is None:
            return ""
        try:
            return old_row[i]
        except IndexError:
            return ""

    # map straightforward columns
    dest["timestamp_utc"] = get_old("timestamp") or get_old("timestamp_utc")
    dest["lat"] = get_old("lat")
    dest["lon"] = get_old("lon")
    dest["heading_deg"] = get_old("compass") or get_old("heading_deg")
    dest["tds_ppm"] = get_old("tds") or get_old("tds_ppm")
    dest["ultrasonic_cm"] = get_old("ultrasonic") or get_old("ultrasonic_cm")
    dest["image_path"] = get_old("image_path") or get_old("image")
    # raw label
    raw = get_old("waste") or get_old("yolo_raw")
    dest["yolo_raw"] = raw

    # waste subtype
    dest["waste_subtype"] = get_old("waste_type") or get_old("waste_subtype")

    # try to classify raw into category if function available
    if raw and classify_label_fn:
        try:
            cat, subtype = classify_label_fn(raw.split(":")[0])
            dest["waste_category"] = cat if cat != "unknown" else raw
            if not dest["waste_subtype"] and subtype:
                dest["waste_subtype"] = subtype
        except Exception:
            dest["waste_category"] = raw
    else:
        dest["waste_category"] = raw

    return [dest[k] for k in CANONICAL_HEADER]


def main():
    parser = argparse.ArgumentParser(description="Migrate telemetry CSV to canonical header")
    parser.add_argument("-i", "--input", help="Path to telemetry CSV file to migrate (optional)")
    parser.add_argument("--dry-run", action="store_true", help="Show a preview of mapped rows without modifying files")
    parser.add_argument("--preview", type=int, default=10, help="Number of rows to preview when using --dry-run")
    args = parser.parse_args()

    # decide which file to operate on: explicit input -> existing DATA_DIR file -> repo-root telemetry.csv
    csv_path = None
    if args.input:
        csv_path = Path(args.input)
    elif CSV_FILE.exists():
        csv_path = CSV_FILE
    else:
        # fallback to repo root telemetry.csv
        root_fallback = ROOT / "telemetry.csv"
        if root_fallback.exists():
            csv_path = root_fallback

    if csv_path is None or not csv_path.exists():
        print("No telemetry.csv found at:", args.input or CSV_FILE)
        return 1

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            old_header = next(reader)
        except StopIteration:
            print("CSV is empty")
            return 1

        # if dry-run requested, preview mapped rows and exit without writing
        if args.dry_run:
            print("--dry-run: previewing mapping of rows to canonical header")
            print("Old header:", old_header)
            old_header_l = [h.strip().lower() for h in old_header]
            preview_n = max(1, args.preview)
            shown = 0
            print("Canonical header:")
            print(",".join(CANONICAL_HEADER))
            for r in reader:
                if not any(x.strip() for x in r):
                    continue
                mapped = migrate_row(r, old_header_l, try_import_classifier())
                print(",".join(str(x) for x in mapped))
                shown += 1
                if shown >= preview_n:
                    break
            print(f"--dry-run complete: showed {shown} rows (use --preview N to change)")
            return 0

        # normalize header strings
        old_header_l = [h.strip().lower() for h in old_header]

        if old_header_l == [h.lower() for h in CANONICAL_HEADER]:
            print("Telemetry CSV is already canonical. No action taken.")
            return 0

        print("Detected non-canonical header:")
        print(old_header)

        # prepare migration
        ts = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        backup = csv_path.with_suffix(f".bak.{ts}")
        try:
            csv_path.rename(backup)
            print(f"Backed up original to: {backup}")
        except PermissionError:
            # On Windows the file may be open by another process preventing rename.
            # Fall back to copying the file to create a backup and continue.
            try:
                shutil.copy2(csv_path, backup)
                print(f"Could not rename file (in use). Copied backup to: {backup}")
            except Exception as e:
                print("Failed to create backup via copy:", e)
                raise

        classify_label_fn = try_import_classifier()
        if classify_label_fn:
            print("Loaded classify_label for mapping labels.")
        else:
            print("No classify_label available; raw label values will be used.")

        out_rows = []
        migrated = 0
        kept = 0
        for r in reader:
            # skip blank rows
            if not any(x.strip() for x in r):
                continue

            # if row already matches canonical length, keep as-is
            if len(r) == len(CANONICAL_HEADER):
                out_rows.append(r)
                kept += 1
            else:
                newr = migrate_row(r, old_header_l, classify_label_fn)
                out_rows.append(newr)
                migrated += 1

        # write new CSV (same path as original)
        with csv_path.open("w", newline="", encoding="utf-8") as fw:
            writer = csv.writer(fw)
            writer.writerow(CANONICAL_HEADER)
            writer.writerows(out_rows)

        print(f"Migration complete. Migrated rows: {migrated}, Kept rows: {kept}")
        print(f"New telemetry file written to: {csv_path}")
        return 0


if __name__ == "__main__":
    sys.exit(main())
