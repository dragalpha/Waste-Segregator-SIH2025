import pandas as pd
from folium import Map
from folium.plugins import HeatMap
from pathlib import Path
import logging

logger = logging.getLogger("heatmap")

CSV = Path(__file__).resolve().parents[2] / "data" / "telemetry.csv"
OUT = Path(__file__).resolve().parents[2] / "data" / "heatmap.html"
OUT_PNG = Path(__file__).resolve().parents[2] / "data" / "heatmap_snapshot.png"


def load_csv():
    if not CSV.exists():
        raise FileNotFoundError(f"No CSV at {CSV}")
    df = pd.read_csv(CSV)
    return df


def prepare_heat_data(df: pd.DataFrame):
    # expect columns lat, lon, waste_category
    df = df[pd.to_numeric(df['lat'], errors='coerce').notnull()]
    df['lat'] = pd.to_numeric(df['lat'])
    df['lon'] = pd.to_numeric(df['lon'])
    heat_data = []
    for r in df.itertuples(index=False):
        try:
            w = 1.0
            pred = getattr(r, 'waste_category', '') or ''
            if isinstance(pred, str):
                p = pred.lower()
                if 'hazard' in p:
                    w = 3.0
                elif 'wet' in p:
                    w = 2.0
            heat_data.append([r.lat, r.lon, w])
        except Exception:
            continue
    return heat_data


def make_map(heat_data, center=None):
    if center is None and heat_data:
        center = [heat_data[0][0], heat_data[0][1]]
    elif center is None:
        center = [22.5726, 88.3639]
    m = Map(location=center, zoom_start=13)
    HeatMap(heat_data).add_to(m)
    return m


def save_heatmap():
    df = load_csv()
    if df.empty:
        logger.info("No data to plot")
        return
    heat_data = prepare_heat_data(df)
    center = [df['lat'].mean(), df['lon'].mean()]
    m = make_map(heat_data, center=center)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    m.save(str(OUT))
    logger.info("Saved heatmap to %s", OUT)

    # try to snapshot HTML to PNG using selenium if available
    try:
        from selenium import webdriver
        from selenium.webdriver.chrome.options import Options
        opts = Options()
        opts.headless = True
        opts.add_argument('--no-sandbox')
        opts.add_argument('--disable-dev-shm-usage')
        driver = webdriver.Chrome(options=opts)
        driver.set_window_size(1200, 800)
        driver.get(OUT.as_uri())
        driver.save_screenshot(str(OUT_PNG))
        driver.quit()
        logger.info("Saved heatmap snapshot to %s", OUT_PNG)
    except Exception:
        logger.info("selenium not available or failed; skipping PNG snapshot")


if __name__ == '__main__':
    try:
        save_heatmap()
    except Exception as e:
        logger.exception("failed to generate heatmap: %s", e)
