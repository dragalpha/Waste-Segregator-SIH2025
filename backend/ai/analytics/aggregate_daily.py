from pathlib import Path
import pandas as pd


def tile_key(lat: float, lon: float, precision: int = 3):
    return (round(float(lat), precision), round(float(lon), precision))


def aggregate(csv_path: Path, tile_precision: int = 3) -> pd.DataFrame:
    df = pd.read_csv(csv_path, parse_dates=['timestamp_utc'])
    df = df.dropna(subset=['lat', 'lon'])
    df['lat'] = pd.to_numeric(df['lat'], errors='coerce')
    df['lon'] = pd.to_numeric(df['lon'], errors='coerce')
    df = df.dropna(subset=['lat', 'lon'])
    df['tile_lat'] = df['lat'].round(tile_precision)
    df['tile_lon'] = df['lon'].round(tile_precision)
    df['date'] = pd.to_datetime(df['timestamp_utc']).dt.date

    def weight_row(r):
        w = 1
        try:
            p = (r.get('waste_category') or '').lower()
            if 'hazard' in p:
                w = 3
            elif 'wet' in p:
                w = 2
        except Exception:
            w = 1
        return w

    df['weight'] = df.apply(lambda row: weight_row(row), axis=1)
    df['loadcell_grams'] = pd.to_numeric(df.get('loadcell_grams', 0), errors='coerce').fillna(0)

    agg = df.groupby(['tile_lat', 'tile_lon', 'date']).agg(
        waste_count=('weight', 'count'),
        total_mass=('loadcell_grams', 'sum'),
        avg_tds=('tds_ppm', lambda x: pd.to_numeric(x, errors='coerce').mean())
    ).reset_index()
    return agg
