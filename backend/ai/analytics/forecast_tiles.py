from pathlib import Path
import pandas as pd
import numpy as np


def forecast_series(series: pd.Series, days: int = 7) -> pd.Series:
    try:
        from statsmodels.tsa.holtwinters import ExponentialSmoothing
        model = ExponentialSmoothing(series, trend='add', seasonal=None)
        fit = model.fit(optimized=True)
        pred = fit.forecast(days)
        return pd.Series(pred)
    except Exception:
        # fallback linear
        x = np.arange(len(series))
        if len(x) < 2:
            return pd.Series([series.iloc[-1]] * days)
        coeffs = np.polyfit(x, series.values, 1)
        xs = np.arange(len(series), len(series)+days)
        preds = np.polyval(coeffs, xs)
        return pd.Series(preds)


def forecast_tiles(agg_df: pd.DataFrame, days: int = 7):
    out = {}
    for (lat, lon), grp in agg_df.groupby(['tile_lat', 'tile_lon']):
        grp2 = grp.sort_values('date')
        wc_series = grp2['waste_count'].astype(float).reset_index(drop=True)
        mass_series = grp2['total_mass'].astype(float).reset_index(drop=True)
        if len(wc_series) == 0:
            continue
        wc_pred = forecast_series(wc_series, days=days)
        mass_pred = forecast_series(mass_series, days=days)
        key = f"{lat}_{lon}"
        out[key] = {
            'tile': (lat, lon),
            'waste_count_pred': wc_pred.tolist(),
            'mass_pred': mass_pred.tolist()
        }
    return out
