from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
import io
import logging

logger = logging.getLogger("report")

BASE = Path(__file__).resolve().parents[2]
CSV = BASE / "data" / "telemetry.csv"
HEATMAP_PNG = BASE / "data" / "heatmap_snapshot.png"
OUT_PDF = BASE / "data" / "report_latest.pdf"
PRESENTATION = Path("/mnt/data/SIH2025-IDEA-Presentation25014.pdf")


def _plot_pie(df: pd.DataFrame) -> bytes:
    last7 = df[pd.to_datetime(df['timestamp_utc']) >= (pd.Timestamp.utcnow() - pd.Timedelta(days=7))]
    counts = last7['waste_subtype'].fillna('unknown').value_counts()
    fig, ax = plt.subplots(figsize=(6,4))
    counts.plot.pie(ax=ax, autopct='%1.1f%%')
    ax.set_ylabel('')
    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight')
    plt.close(fig)
    buf.seek(0)
    return buf.read()


def _plot_mass_bar(df: pd.DataFrame) -> bytes:
    df['date'] = pd.to_datetime(df['timestamp_utc']).dt.date
    s = df.groupby('date')['loadcell_grams'].apply(lambda x: pd.to_numeric(x, errors='coerce').sum())
    fig, ax = plt.subplots(figsize=(8,4))
    s.plot.bar(ax=ax)
    ax.set_ylabel('mass (g)')
    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight')
    plt.close(fig)
    buf.seek(0)
    return buf.read()


def _plot_tds_line(df: pd.DataFrame) -> bytes:
    df['ts'] = pd.to_datetime(df['timestamp_utc'])
    s = pd.to_numeric(df['tds_ppm'], errors='coerce')
    fig, ax = plt.subplots(figsize=(8,3))
    s.plot(ax=ax)
    ax.set_ylabel('TDS (ppm)')
    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight')
    plt.close(fig)
    buf.seek(0)
    return buf.read()


def build_pdf():
    if not CSV.exists():
        raise FileNotFoundError("No telemetry CSV found")
    df = pd.read_csv(CSV)
    pie_png = _plot_pie(df)
    mass_png = _plot_mass_bar(df)
    tds_png = _plot_tds_line(df)

    c = canvas.Canvas(str(OUT_PDF), pagesize=A4)
    width, height = A4
    c.setFont('Helvetica-Bold', 16)
    c.drawString(40, height - 40, 'Waste Segregation Report')

    c.drawImage(ImageReader(io.BytesIO(pie_png)), 40, height - 300, width=240, height=240)
    c.drawImage(ImageReader(io.BytesIO(mass_png)), 300, height - 300, width=240, height=180)
    c.drawImage(ImageReader(io.BytesIO(tds_png)), 40, height - 560, width=500, height=150)

    if HEATMAP_PNG.exists():
        try:
            c.drawImage(str(HEATMAP_PNG), 40, 40, width=500, height=300)
        except Exception:
            logger.exception('failed to add heatmap png')

    # add reference to presentation if exists
    if PRESENTATION.exists():
        c.showPage()
        c.setFont('Helvetica', 12)
        c.drawString(40, height - 40, 'Reference Presentation:')
        c.drawString(40, height - 60, str(PRESENTATION))

    c.showPage()
    c.save()
    print('Saved report to', OUT_PDF)


if __name__ == '__main__':
    try:
        build_pdf()
    except Exception as e:
        logger.exception('report generation failed: %s', e)
