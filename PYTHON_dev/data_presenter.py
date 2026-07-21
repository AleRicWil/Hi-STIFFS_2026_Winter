"""
Stalk Flexural Stiffness Distinguishability Figure
=================================================
Generates a presentation-ready figure for the ASABE AIM 2026 conference
demonstrating the Hi-STIFFS measurement device's ability to distinguish
individual corn stalks given its repeatability (measurement margin of error).

- Sorts stalks by ascending mean stiffness
- Overlays all replicate measurements (jittered)
- Shows mean ± 3σ error bars (conservative repeatability margin)
- Professional styling, high-resolution export (PNG + PDF)
- Self-updating caption with average CV

Author: Generated for Alex R. Williams
Date: July 2026
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# =============================================================================
# USER CONFIGURATION
# =============================================================================
DATA_PATH = r'C:\Users\Alex R. Williams\Documents\School\Grad\ASABE\Presentation\Data\all_data.csv'
OUTPUT_DIR = r'C:\Users\Alex R. Williams\Documents\School\Grad\ASABE\Presentation\Figures'
os.makedirs(OUTPUT_DIR, exist_ok=True)

def field_2025():
    # Sigma level for error bars (3 = conservative ~99.7% repeatability margin)
    SIGMA_LEVEL = 3

    # Figure output filenames
    FILENAME_BASE = f'stalk_stiffness_distinguishability_{SIGMA_LEVEL}sigma'

    # =============================================================================
    # DATA LOADING
    # =============================================================================
    print("Loading data...")
    df = pd.read_csv(
        DATA_PATH,
        skiprows=2,
        nrows=26,
        usecols=['Stalk', 'Test_01', 'Test_02', 'Test_03', 'Test_04', 'Test_05',
                'Test_06', 'Test_07', 'Test_08', 'Test_09', 'Test_10']
    )

    test_cols = ['Test_01', 'Test_02', 'Test_03', 'Test_04', 'Test_05',
                'Test_06', 'Test_07', 'Test_08', 'Test_09', 'Test_10']

    # =============================================================================
    # STATISTICAL COMPUTATION
    # =============================================================================
    # Per-stalk mean and sample standard deviation (ddof=1 for unbiased estimator)
    df['Mean'] = df[test_cols].mean(axis=1)
    df['Std'] = df[test_cols].std(axis=1, ddof=1)
    df['n_valid'] = df[test_cols].notna().sum(axis=1)

    # Drop any stalks with insufficient data for mean/std
    df_valid = df.dropna(subset=['Mean', 'Std']).copy()

    if df_valid.empty:
        raise ValueError("No valid stalks with computable mean and standard deviation.")
    

    df_valid = df_valid[~df_valid['Stalk'].isin(['S113', 'S03', 'S104', 'S02', 'S115'])]

    # Sort stalks by ascending mean stiffness (critical for visual distinguishability assessment)
    df_sorted = df_valid.sort_values('Mean').reset_index(drop=True)






    n_stalks = len(df_sorted)

    print(f"Processed {n_stalks} stalks with valid measurements.")

    # Prepare melted data for individual replicate plotting (handles NaNs automatically)
    df_melted = df.melt(
        id_vars=['Stalk'],
        value_vars=test_cols,
        var_name='Test',
        value_name='Stiffness'
    ).dropna(subset=['Stiffness'])

    # =============================================================================
    # FIGURE GENERATION
    # =============================================================================
    print("Generating figure...")

    # Professional styling
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Helvetica']
    plt.rcParams['axes.linewidth'] = 0.8
    plt.rcParams['axes.labelpad'] = 6

    fig, ax = plt.subplots(figsize=(13.5, 5.8), dpi=150)

    x_positions = np.arange(n_stalks)
    jitter_strength = 0.12

    # ---- Layer 1: Individual replicate measurements (semi-transparent, jittered) ----
    np.random.seed(42)  # Reproducible jitter
    for idx, row in df_sorted.iterrows():
        pos = x_positions[idx]
        stalk_id = row['Stalk']
        vals = df_melted[df_melted['Stalk'] == stalk_id]['Stiffness'].values
        
        if len(vals) > 0:
            x_jittered = np.random.normal(pos, jitter_strength, size=len(vals))
            ax.scatter(
                x_jittered, vals,
                s=18,
                alpha=0.5,
                color='#5DADE2',
                edgecolors='none',
                zorder=1,
                rasterized=True  # Helps with file size for many points
            )

    # ---- Layer 2: Error bars (mean ± SIGMA_LEVEL * σ) ----
    yerr = SIGMA_LEVEL * df_sorted['Std'].values
    ax.errorbar(
        x_positions,
        df_sorted['Mean'].values,
        yerr=yerr,
        fmt='none',
        ecolor='#1B4F72',
        elinewidth=2.1,
        capsize=4.5,
        capthick=1.9,
        zorder=2
    )

    # ---- Layer 3: Mean markers ----
    ax.scatter(
        x_positions,
        df_sorted['Mean'].values,
        s=85,
        c='#1B4F72',
        marker='D',
        zorder=3,
        edgecolors='white',
        linewidths=1.1,
        label=f'Mean ± {SIGMA_LEVEL}σ'
    )

    # ---- Axes, labels, and ticks ----
    ax.set_xticks(x_positions)
    stalk_labels = df_sorted['Stalk'].astype(str).values
    ax.set_xticklabels(stalk_labels, rotation=45, ha='right', fontsize=8)

    ax.set_xlabel('Stalk Identifier (sorted by ascending mean)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')

    ax.set_title(
        'Per-Stalk Flexural Stiffness Measurements\n'
        'Field 2025 - Vigor Root',
        fontsize=14,
        fontweight='bold',
        pad=10
    )

    # Clean professional grid and spines
    ax.yaxis.grid(True, linestyle='--', alpha=0.45, linewidth=0.7)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)

    # Optional: add subtle padding to y-limits for visual breathing room
    y_min = np.nanmin(df_sorted['Mean'].values - SIGMA_LEVEL * df_sorted['Std'].values)
    y_max = np.nanmax(df_sorted['Mean'].values + SIGMA_LEVEL * df_sorted['Std'].values)
    y_range = y_max - y_min
    ax.set_ylim(y_min - 0.06 * y_range, y_max + 0.08 * y_range)

    # =============================================================================
    # CAPTION (self-contained for slides)
    # =============================================================================
    # Compute average CV dynamically
    mask = (df_sorted['Mean'] > 0) & df_sorted['Std'].notna()
    avg_cv = (df_sorted.loc[mask, 'Std'] / df_sorted.loc[mask, 'Mean'] * 100).mean() if mask.any() else np.nan
    median_cv = (df_sorted.loc[mask, 'Std'] / df_sorted.loc[mask, 'Mean'] * 100).median() if mask.any() else np.nan
    median_sd = (df_sorted.loc[mask, 'Std']).median() if mask.any() else np.nan

    caption_text = (
        f"Error bars: [mean ± {SIGMA_LEVEL}σ] from 10 replicate tests per stalk. "
        f"Median standard deviation (SD): {median_sd:.2f} N·m². "
        f"Median coefficient of variation (CV): {median_cv:.2f}%. "
    )

    # Leave space at bottom for caption
    plt.tight_layout(rect=[0, 0.08, 1, 0.98])

    fig.text(
        0.5, 0.015,
        caption_text,
        ha='center',
        va='bottom',
        fontsize=14,
        linespacing=1.35,
        wrap=True,
        bbox=dict(boxstyle='round,pad=0.45', facecolor='#F4F6F7', edgecolor='#AAB7B8', linewidth=0.7)
    )

    # =============================================================================
    # EXPORT
    # =============================================================================
    output_png = os.path.join(OUTPUT_DIR, f'{FILENAME_BASE}.png')
    output_pdf = os.path.join(OUTPUT_DIR, f'{FILENAME_BASE}.pdf')

    fig.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    # fig.savefig(output_pdf, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig)

    print(f"\nFigure successfully generated and saved:")
    print(f"  PNG (300 dpi): {output_png}")
    print(f"  PDF (vector):  {output_pdf}")
    print(f"\nAverage CV across stalks: {avg_cv:.2f}%")
  

def lab_2025():
    """
    Generate a presentation-ready distinguishability figure for the ASABE AIM 2026
    conference using the Hi-STIFFS lab data (selected low/med/high PVC stalks).

    - Loads the lab Hi-STIFFS measurements from all_data.csv (skiprows=31, nrows=9)
    - Sorts stalks by name only (no sorting by mean stiffness)
    - Overlays all replicate measurements (jittered)
    - Shows mean ± 3σ error bars (conservative repeatability margin)
    - Professional styling matching the field_2025 figure, high-resolution export (PNG + PDF)
    - Self-updating caption with median SD and CV
    """
    # Sigma level for error bars (3 = conservative ~99.7% repeatability margin)
    SIGMA_LEVEL = 3

    # Figure output filenames
    FILENAME_BASE = f'2025_lab_stalk_stiffness_distinguishability_{SIGMA_LEVEL}sigma'

    # =============================================================================
    # DATA LOADING
    # =============================================================================
    print("Loading Hi-STIFFS lab data (selected low/med/high PVC stalks)...")
    df = pd.read_csv(
        DATA_PATH,
        skiprows=31,
        nrows=9,
        usecols=['Stalk', 'Test_01', 'Test_02', 'Test_03', 'Test_04', 'Test_05',
                 'Test_06', 'Test_07', 'Test_08', 'Test_09', 'Test_10']
    )

    test_cols = ['Test_01', 'Test_02', 'Test_03', 'Test_04', 'Test_05',
                 'Test_06', 'Test_07', 'Test_08', 'Test_09', 'Test_10']

    # =============================================================================
    # STATISTICAL COMPUTATION
    # =============================================================================
    # Per-stalk mean and sample standard deviation (ddof=1 for unbiased estimator)
    df['Mean'] = df[test_cols].mean(axis=1)
    df['Std'] = df[test_cols].std(axis=1, ddof=1)
    df['n_valid'] = df[test_cols].notna().sum(axis=1)

    # Drop any stalks with insufficient data for mean/std
    df_valid = df.dropna(subset=['Mean', 'Std']).copy()

    if df_valid.empty:
        raise ValueError("No valid stalks with computable mean and standard deviation.")

    # Sort stalks by name only (lexical order of the 'Stalk' column)
    df_sorted = df_valid.sort_values('Stalk').reset_index(drop=True)
    n_stalks = len(df_sorted)

    print(f"Processed {n_stalks} stalks with valid measurements.")

    # Prepare melted data for individual replicate plotting (handles NaNs automatically)
    df_melted = df.melt(
        id_vars=['Stalk'],
        value_vars=test_cols,
        var_name='Test',
        value_name='Stiffness'
    ).dropna(subset=['Stiffness'])

    # =============================================================================
    # FIGURE GENERATION
    # =============================================================================
    print("Generating figure...")

    # Professional styling (identical to field_2025)
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Helvetica']
    plt.rcParams['axes.linewidth'] = 0.8
    plt.rcParams['axes.labelpad'] = 6

    fig, ax = plt.subplots(figsize=(5,5), dpi=150)

    x_positions = np.arange(n_stalks)
    jitter_strength = 0.12

    # ---- Layer 1: Individual replicate measurements (semi-transparent, jittered) ----
    np.random.seed(42)  # Reproducible jitter
    for idx, row in df_sorted.iterrows():
        pos = x_positions[idx]
        stalk_id = row['Stalk']
        vals = df_melted[df_melted['Stalk'] == stalk_id]['Stiffness'].values

        if len(vals) > 0:
            x_jittered = np.random.normal(pos, jitter_strength, size=len(vals))
            ax.scatter(
                x_jittered, vals,
                s=18,
                alpha=0.5,
                color='#5DADE2',
                edgecolors='none',
                zorder=1,
                rasterized=True
            )

    # ---- Layer 2: Error bars (mean ± SIGMA_LEVEL * σ) ----
    yerr = SIGMA_LEVEL * df_sorted['Std'].values
    ax.errorbar(
        x_positions,
        df_sorted['Mean'].values,
        yerr=yerr,
        fmt='none',
        ecolor='#1B4F72',
        elinewidth=2.1,
        capsize=4.5,
        capthick=1.9,
        zorder=2
    )

    # ---- Layer 3: Mean markers ----
    ax.scatter(
        x_positions,
        df_sorted['Mean'].values,
        s=85,
        c='#1B4F72',
        marker='D',
        zorder=3,
        edgecolors='white',
        linewidths=1.1,
        label=f'Mean ± {SIGMA_LEVEL}σ'
    )

    # ---- Axes, labels, and ticks ----
    ax.set_xticks(x_positions)
    stalk_labels = df_sorted['Stalk'].astype(str).values
    ax.set_xticklabels(stalk_labels, rotation=45, ha='right', fontsize=8)

    ax.set_xlabel('Stalk Identifier', fontsize=14, fontweight='bold')
    ax.set_ylabel('Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')

    ax.set_title(
        'Per-Stalk Flexural Stiffness Measurements\n'
        'Lab 2025 - Artificial PVC Stalks',
        fontsize=14,
        fontweight='bold',
        pad=10
    )

    # Clean professional grid and spines
    ax.yaxis.grid(True, linestyle='--', alpha=0.45, linewidth=0.7)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)

    # Optional: add subtle padding to y-limits for visual breathing room
    y_min = np.nanmin(df_sorted['Mean'].values - SIGMA_LEVEL * df_sorted['Std'].values)
    y_max = np.nanmax(df_sorted['Mean'].values + SIGMA_LEVEL * df_sorted['Std'].values)
    y_range = y_max - y_min
    ax.set_ylim(0 - 0.06 * y_range, 27)# y_max + 0.10 * y_range)

    # =============================================================================
    # CAPTION (self-contained for slides)
    # =============================================================================
    # Compute median statistics dynamically
    mask = (df_sorted['Mean'] > 0) & df_sorted['Std'].notna()
    median_cv = (df_sorted.loc[mask, 'Std'] / df_sorted.loc[mask, 'Mean'] * 100).median() if mask.any() else np.nan
    median_sd = (df_sorted.loc[mask, 'Std']).median() if mask.any() else np.nan

    caption_text = (
        f"Error bars: [mean ± {SIGMA_LEVEL}σ] from 10 replicate tests per stalk. "
        f"Median standard deviation (SD): {median_sd:.2f} N·m². "
        f"Median coefficient of variation (CV): {median_cv:.2f}%. "
    )

    # Leave space at bottom for caption
    plt.tight_layout(rect=[0, 0.08, 1, 0.98])

    fig.text(
        0.5, 0.015,
        caption_text,
        ha='center',
        va='bottom',
        fontsize=11,
        linespacing=1.35,
        wrap=True,
        bbox=dict(boxstyle='round,pad=0.45', facecolor='#F4F6F7', edgecolor='#AAB7B8', linewidth=0.7)
    )

    # =============================================================================
    # EXPORT
    # =============================================================================
    output_png = os.path.join(OUTPUT_DIR, f'{FILENAME_BASE}_narr.png')
    output_pdf = os.path.join(OUTPUT_DIR, f'{FILENAME_BASE}.pdf')

    fig.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    # fig.savefig(output_pdf, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig)

    print(f"\nFigure successfully generated and saved:")
    print(f"  PNG (300 dpi): {output_png}")
    print(f"  PDF (vector):  {output_pdf}")


def lab_2026():
    """
    Generate a presentation-ready distinguishability figure for the ASABE AIM 2026
    conference using the Hi-STIFFS lab data (selected low/med/high PVC stalks).

    - Loads the lab Hi-STIFFS measurements from all_data.csv (skiprows=31, nrows=9)
    - Sorts stalks by name only (no sorting by mean stiffness)
    - Overlays all replicate measurements (jittered)
    - Shows mean ± 3σ error bars (conservative repeatability margin)
    - Professional styling matching the field_2025 figure, high-resolution export (PNG + PDF)
    - Self-updating caption with median SD and CV
    """
    # Sigma level for error bars (3 = conservative ~99.7% repeatability margin)
    SIGMA_LEVEL = 3

    # Figure output filenames
    FILENAME_BASE = f'2026_lab_stalk_stiffness_distinguishability_{SIGMA_LEVEL}sigma'

    # =============================================================================
    # DATA LOADING
    # =============================================================================
    print("Loading Hi-STIFFS lab data...")
    df = pd.read_csv(
        DATA_PATH,
        skiprows=45,
        nrows=16,
        usecols=['Stalk', 'Test_01', 'Test_02', 'Test_03', 'Test_04', 'Test_05',
                 'Test_06', 'Test_07', 'Test_08', 'Test_09', 'Test_10']
    )

    test_cols = ['Test_01', 'Test_02', 'Test_03', 'Test_04', 'Test_05',
                 'Test_06', 'Test_07', 'Test_08', 'Test_09', 'Test_10']

    # =============================================================================
    # STATISTICAL COMPUTATION
    # =============================================================================
    # Per-stalk mean and sample standard deviation (ddof=1 for unbiased estimator)
    df['Mean'] = df[test_cols].mean(axis=1)
    df['Std'] = df[test_cols].std(axis=1, ddof=1)
    df['n_valid'] = df[test_cols].notna().sum(axis=1)

    # Drop any stalks with insufficient data for mean/std
    df_valid = df.dropna(subset=['Mean', 'Std']).copy()

    if df_valid.empty:
        raise ValueError("No valid stalks with computable mean and standard deviation.")

    df_valid = df_valid[~df_valid['Stalk'].isin(['S02'])]

    # Sort stalks by name only (lexical order of the 'Stalk' column)
    df_sorted = df_valid.sort_values('Stalk').reset_index(drop=True)
    n_stalks = len(df_sorted)

    print(f"Processed {n_stalks} stalks with valid measurements.")

    # Prepare melted data for individual replicate plotting (handles NaNs automatically)
    df_melted = df.melt(
        id_vars=['Stalk'],
        value_vars=test_cols,
        var_name='Test',
        value_name='Stiffness'
    ).dropna(subset=['Stiffness'])

    # =============================================================================
    # FIGURE GENERATION
    # =============================================================================
    print("Generating figure...")

    # Professional styling (identical to field_2025)
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Helvetica']
    plt.rcParams['axes.linewidth'] = 0.8
    plt.rcParams['axes.labelpad'] = 6

    fig, ax = plt.subplots(figsize=(5, 5), dpi=150)

    x_positions = np.arange(n_stalks)
    jitter_strength = 0.12

    # ---- Layer 1: Individual replicate measurements (semi-transparent, jittered) ----
    np.random.seed(42)  # Reproducible jitter
    for idx, row in df_sorted.iterrows():
        pos = x_positions[idx]
        stalk_id = row['Stalk']
        vals = df_melted[df_melted['Stalk'] == stalk_id]['Stiffness'].values

        if len(vals) > 0:
            x_jittered = np.random.normal(pos, jitter_strength, size=len(vals))
            ax.scatter(
                x_jittered, vals,
                s=18,
                alpha=0.5,
                color='#5DADE2',
                edgecolors='none',
                zorder=1,
                rasterized=True
            )

    # ---- Layer 2: Error bars (mean ± SIGMA_LEVEL * σ) ----
    yerr = SIGMA_LEVEL * df_sorted['Std'].values
    ax.errorbar(
        x_positions,
        df_sorted['Mean'].values,
        yerr=yerr,
        fmt='none',
        ecolor='#1B4F72',
        elinewidth=2.1,
        capsize=4.5,
        capthick=1.9,
        zorder=2
    )

    # ---- Layer 3: Mean markers ----
    ax.scatter(
        x_positions,
        df_sorted['Mean'].values,
        s=85,
        c='#1B4F72',
        marker='D',
        zorder=3,
        edgecolors='white',
        linewidths=1.1,
        label=f'Mean ± {SIGMA_LEVEL}σ'
    )

    # ---- Axes, labels, and ticks ----
    ax.set_xticks(x_positions)
    stalk_labels = df_sorted['Stalk'].astype(str).values
    ax.set_xticklabels(stalk_labels, rotation=45, ha='right', fontsize=8)

    ax.set_xlabel('Stalk Identifier', fontsize=14, fontweight='bold')
    ax.set_ylabel('Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')

    ax.set_title(
        'Per-Stalk Flexural Stiffness Measurements\n'
        'Lab 2026 - Artificial PVC Stalks',
        fontsize=14,
        fontweight='bold',
        pad=10
    )

    # Clean professional grid and spines
    ax.yaxis.grid(True, linestyle='--', alpha=0.45, linewidth=0.7)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)

    # Optional: add subtle padding to y-limits for visual breathing room
    y_min = np.nanmin(df_sorted['Mean'].values - SIGMA_LEVEL * df_sorted['Std'].values)
    y_max = np.nanmax(df_sorted['Mean'].values + SIGMA_LEVEL * df_sorted['Std'].values)
    y_range = y_max - y_min
    ax.set_ylim(0 - 0.06 * y_range, 27)# y_max + 0.10 * y_range)

    # =============================================================================
    # CAPTION (self-contained for slides)
    # =============================================================================
    # Compute median statistics dynamically
    mask = (df_sorted['Mean'] > 0) & df_sorted['Std'].notna()
    median_cv = (df_sorted.loc[mask, 'Std'] / df_sorted.loc[mask, 'Mean'] * 100).median() if mask.any() else np.nan
    median_sd = (df_sorted.loc[mask, 'Std']).median() if mask.any() else np.nan

    caption_text = (
        f"Error bars: [mean ± {SIGMA_LEVEL}σ] from 10 replicate tests per stalk. "
        f"Median standard deviation (SD): {median_sd:.2f} N·m². "
        f"Median coefficient of variation (CV): {median_cv:.2f}%. "
    )

    # Leave space at bottom for caption
    plt.tight_layout(rect=[0, 0.08, 1, 0.98])

    fig.text(
        0.5, 0.015,
        caption_text,
        ha='center',
        va='bottom',
        fontsize=11,
        linespacing=1.35,
        wrap=True,
        bbox=dict(boxstyle='round,pad=0.45', facecolor='#F4F6F7', edgecolor='#AAB7B8', linewidth=0.7)
    )

    # =============================================================================
    # EXPORT
    # =============================================================================
    output_png = os.path.join(OUTPUT_DIR, f'{FILENAME_BASE}_narr.png')
    # output_pdf = os.path.join(OUTPUT_DIR, f'{FILENAME_BASE}.pdf')

    fig.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    # fig.savefig(output_pdf, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig)

    print(f"\nFigure successfully generated and saved:")
    print(f"  PNG (300 dpi): {output_png}")
    # print(f"  PDF (vector):  {output_pdf}")


def lab_2026_both():
    """
    Generate two presentation-ready figures for the ASABE AIM 2026 conference
    comparing Hi-STIFFS and DARLING flexural stiffness measurements on the
    same set of medium stalks from the 2026 lab campaign.

    Figure 1: Distinguishability plot with both devices shown side-by-side.
              Stalks ordered by identifier name (as stored in the source CSVs).
              Hi-STIFFS in professional blue, DARLING in red. Jittered replicates,
              mean ± 3σ error bars, clean legend and caption.

    Figure 2: Correlation scatter plot (DARLING on x-axis, Hi-STIFFS on y-axis)
              with ±3σ error bars, 1:1 reference line, and linear regression fit.

    Both figures use consistent professional styling, high-resolution export
    (PNG + PDF), and self-updating captions containing median SD and CV.
    """
    # Sigma level for error bars (3 = conservative ~99.7% repeatability margin)
    SIGMA_LEVEL = 3

    # Figure output base names
    DISTINGUISH_BASE = f'lab_2026_stalk_stiffness_hi-stiffs_vs_darling_{SIGMA_LEVEL}sigma'
    CORR_BASE = 'lab_2026_hi-stiffs_vs_darling_correlation'

    # =============================================================================
    # DATA LOADING
    # =============================================================================
    print("Loading Hi-STIFFS 2026 lab data...")
    # NOTE: Update skiprows and nrows for the correct 2026 section in all_data.csv
    df_h = pd.read_csv(
        DATA_PATH,
        skiprows=45,   # <-- UPDATE THIS VALUE FOR 2026 Hi-STIFFS DATA
        nrows=16,       # <-- UPDATE THIS VALUE FOR 2026 Hi-STIFFS DATA
        usecols=lambda c: c == 'Stalk' or str(c).startswith('Test_')
    )

    print("Loading DARLING 2026 data (medium stalks from stiffnesses.csv)...")
    data_dir = os.path.dirname(DATA_PATH)
    darling_path = 'stiffnesses.csv'
    df_d = pd.read_csv(darling_path)

    # Dynamically identify test columns for each device
    test_cols_h = [c for c in df_h.columns if str(c).startswith('Test_')]
    test_cols_d = [c for c in df_d.columns if str(c).startswith('Test_')]

    # Sort both DataFrames by Stalk identifier name (order as stored in source CSVs)
    df_h = df_h.sort_values('Stalk').reset_index(drop=True)
    df_d = df_d.sort_values('Stalk').reset_index(drop=True)

    # Align on common stalks (identifiers are confirmed to pair exactly)
    common_stalks = sorted(set(df_h['Stalk']) & set(df_d['Stalk']))
    if len(common_stalks) == 0:
        raise ValueError("No common stalk identifiers found between Hi-STIFFS and DARLING datasets.")
    if len(common_stalks) < len(df_h):
        print(f"Note: Using {len(common_stalks)} common stalks out of {len(df_h)} Hi-STIFFS stalks.")

    df_h = df_h[df_h['Stalk'].isin(common_stalks)].reset_index(drop=True)
    df_d = df_d[df_d['Stalk'].isin(common_stalks)].reset_index(drop=True)
    n_stalks = len(df_h)

    print(f"Processed {n_stalks} paired stalks for 2026 lab comparison.")

    # =============================================================================
    # STATISTICAL COMPUTATION
    # =============================================================================
    # Hi-STIFFS
    df_h['Mean'] = df_h[test_cols_h].mean(axis=1)
    df_h['Std'] = df_h[test_cols_h].std(axis=1, ddof=1)

    # DARLING
    df_d['Mean'] = df_d[test_cols_d].mean(axis=1)
    df_d['Std'] = df_d[test_cols_d].std(axis=1, ddof=1)

    # Melted data for replicate scatter plots
    df_h_melted = df_h.melt(
        id_vars=['Stalk'], value_vars=test_cols_h,
        var_name='Test', value_name='Stiffness'
    ).dropna(subset=['Stiffness'])

    df_d_melted = df_d.melt(
        id_vars=['Stalk'], value_vars=test_cols_d,
        var_name='Test', value_name='Stiffness'
    ).dropna(subset=['Stiffness'])

    # =============================================================================
    # FIGURE 1: Distinguishability plot (Hi-STIFFS blue + DARLING red)
    # =============================================================================
    print("Generating distinguishability comparison figure...")

    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Helvetica']
    plt.rcParams['axes.linewidth'] = 0.8
    plt.rcParams['axes.labelpad'] = 6

    fig, ax = plt.subplots(figsize=(11.0, 5.0), dpi=150)

    x_positions = np.arange(n_stalks)
    jitter_strength = 0.07
    x_offset = 0.16

    # Professional colors
    color_h_pts = '#5DADE2'
    color_h_mean = '#1B4F72'
    color_d_pts = '#E74C3C'
    color_d_mean = '#922B21'

    np.random.seed(42)

    # Hi-STIFFS replicates (left offset)
    for idx, row in df_h.iterrows():
        pos = x_positions[idx]
        stalk_id = row['Stalk']
        vals = df_h_melted[df_h_melted['Stalk'] == stalk_id]['Stiffness'].values
        if len(vals) > 0:
            x_j = np.random.normal(pos - x_offset, jitter_strength, size=len(vals))
            ax.scatter(x_j, vals, s=15, alpha=0.40, color=color_h_pts,
                       edgecolors='none', zorder=1, rasterized=True,
                       label='Hi-STIFFS replicates' if idx == 0 else None)

    # DARLING replicates (right offset)
    np.random.seed(123)
    for idx, row in df_d.iterrows():
        pos = x_positions[idx]
        stalk_id = row['Stalk']
        vals = df_d_melted[df_d_melted['Stalk'] == stalk_id]['Stiffness'].values
        if len(vals) > 0:
            x_j = np.random.normal(pos + x_offset, jitter_strength, size=len(vals))
            ax.scatter(x_j, vals, s=15, alpha=0.40, color=color_d_pts,
                       edgecolors='none', zorder=1, rasterized=True,
                       label='DARLING replicates' if idx == 0 else None)

    # Hi-STIFFS error bars + mean markers
    yerr_h = SIGMA_LEVEL * df_h['Std'].values
    ax.errorbar(x_positions - x_offset, df_h['Mean'].values, yerr=yerr_h,
                fmt='none', ecolor=color_h_mean, elinewidth=2.0,
                capsize=3.8, capthick=1.6, zorder=2)
    ax.scatter(x_positions - x_offset, df_h['Mean'].values,
               s=95, c=color_h_mean, marker='D', zorder=3,
               edgecolors='white', linewidths=1.1,
               label=f'Hi-STIFFS mean ± {SIGMA_LEVEL}σ')

    # DARLING error bars + mean markers
    yerr_d = SIGMA_LEVEL * df_d['Std'].values
    ax.errorbar(x_positions + x_offset, df_d['Mean'].values, yerr=yerr_d,
                fmt='none', ecolor=color_d_mean, elinewidth=2.0,
                capsize=3.8, capthick=1.6, zorder=2)
    ax.scatter(x_positions + x_offset, df_d['Mean'].values,
               s=95, c=color_d_mean, marker='o', zorder=3,
               edgecolors='white', linewidths=1.1,
               label=f'DARLING mean ± {SIGMA_LEVEL}σ')

    # Axes and labels
    ax.set_xticks(x_positions)
    ax.set_xticklabels(df_h['Stalk'].astype(str).values,
                       rotation=45, ha='right', fontsize=8)
    ax.set_xlabel('Stalk Identifier', fontsize=14, fontweight='bold')
    ax.set_ylabel('Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')
    ax.set_title(
        'Per-Stalk Flexural Stiffness: Hi-STIFFS vs DARLING\n'
        'Lab 2026 - Artificial PVC Stalks',
        fontsize=14, fontweight='bold', pad=8
    )

    # Styling
    ax.yaxis.grid(True, linestyle='--', alpha=0.40, linewidth=0.7)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(0.8)
    ax.spines['bottom'].set_linewidth(0.8)

    ax.legend(loc='upper right', fontsize=9, framealpha=0.92, edgecolor='#CCCCCC')

    # Y-limits
    all_means = np.concatenate([df_h['Mean'].values, df_d['Mean'].values])
    all_stds = np.concatenate([df_h['Std'].values, df_d['Std'].values])
    y_min = np.nanmin(all_means - SIGMA_LEVEL * all_stds)
    y_max = np.nanmax(all_means + SIGMA_LEVEL * all_stds)
    y_range = y_max - y_min
    ax.set_ylim(0 - 0.07 * y_range, y_max + 0.09 * y_range)

    # Dynamic caption
    mask_h = (df_h['Mean'] > 0) & df_h['Std'].notna()
    med_sd_h = df_h.loc[mask_h, 'Std'].median() if mask_h.any() else np.nan
    med_cv_h = (df_h.loc[mask_h, 'Std'] / df_h.loc[mask_h, 'Mean'] * 100).median() if mask_h.any() else np.nan

    mask_d = (df_d['Mean'] > 0) & df_d['Std'].notna()
    med_sd_d = df_d.loc[mask_d, 'Std'].median() if mask_d.any() else np.nan
    med_cv_d = (df_d.loc[mask_d, 'Std'] / df_d.loc[mask_d, 'Mean'] * 100).median() if mask_d.any() else np.nan

    caption_text = (
        f"Hi-STIFFS (blue diamonds): mean ± {SIGMA_LEVEL}σ from 10 replicates. "
        f"Median SD = {med_sd_h:.2f} N·m², Median CV = {med_cv_h:.2f}%.\n"
        f"DARLING (red circles): mean ± {SIGMA_LEVEL}σ from 10 replicates. "
        f"Median SD = {med_sd_d:.2f} N·m², Median CV = {med_cv_d:.2f}%."
    )

    plt.tight_layout(rect=[0, 0.095, 1, 0.97])
    fig.text(0.5, 0.012, caption_text, ha='center', va='bottom', fontsize=9.5,
             linespacing=1.30, wrap=True,
             bbox=dict(boxstyle='round,pad=0.40', facecolor='#F4F6F7',
                       edgecolor='#AAB7B8', linewidth=0.6))

    # Export
    out_png = os.path.join(OUTPUT_DIR, f'{DISTINGUISH_BASE}.png')
    # out_pdf = os.path.join(OUTPUT_DIR, f'{DISTINGUISH_BASE}.pdf')
    fig.savefig(out_png, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    # fig.savefig(out_pdf, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig)
    print(f"  Saved: {out_png}")
    # print(f"  Saved: {out_pdf}")

    # =============================================================================
    # FIGURE 2: Correlation plot (DARLING horizontal, Hi-STIFFS vertical)
    # =============================================================================
    print("Generating correlation figure...")

    fig2, ax2 = plt.subplots(figsize=(7.8, 7.8), dpi=150)

    x_vals = df_d['Mean'].values
    y_vals = df_h['Mean'].values
    x_err = SIGMA_LEVEL * df_d['Std'].values
    y_err = SIGMA_LEVEL * df_h['Std'].values

    # Error bars
    ax2.errorbar(x_vals, y_vals, xerr=x_err, yerr=y_err,
                 fmt='none', ecolor='#7F8C8D', elinewidth=1.1,
                 capsize=2.2, capthick=0.9, alpha=0.65, zorder=1, label='3σ Error Bar')

    # Scatter points (single color since only medium stalks)
    ax2.scatter(x_vals, y_vals, s=115, c='#2980B9',
                marker='o', edgecolors='white', linewidths=1.15,
                zorder=3, label='Stalks')

    # 1:1 reference line
    min_all = min(np.min(x_vals - x_err), np.min(y_vals - y_err))
    max_all = max(np.max(x_vals + x_err), np.max(y_vals + y_err))
    pad = 0.06 * (max_all - min_all)
    ax2.plot([min_all - pad, max_all + pad], [min_all - pad, max_all + pad],
             'k--', linewidth=1.6, alpha=0.75, label='1:1 line', zorder=2)

    # Linear regression
    from scipy.stats import linregress
    slope, intercept, r_val, p_val, se_slope = linregress(x_vals, y_vals)
    x_fit = np.linspace(min_all - pad, max_all + pad, 200)
    y_fit = slope * x_fit + intercept
    # ax2.plot(x_fit, y_fit, color='#D35400', linewidth=2.1, linestyle='-',
    #          label=f'Linear fit  (R² = {r_val**2:.3f})', zorder=2)

    # Labels, title, limits
    ax2.set_xlabel('DARLING Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Hi-STIFFS Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')
    ax2.set_title(
        'Correlation Between DARLING and Hi-STIFFS Measurements\n'
        'Lab 2026 - Artificial PVC Stalks',
        fontsize=12, fontweight='bold', pad=8
    )

    ax2.set_xlim(min_all - pad, max_all + pad)
    ax2.set_ylim(min_all - pad, max_all + pad)
    ax2.set_aspect('equal', adjustable='box')

    ax2.yaxis.grid(True, linestyle='--', alpha=0.35, linewidth=0.7)
    ax2.xaxis.grid(True, linestyle='--', alpha=0.35, linewidth=0.7)
    ax2.set_axisbelow(True)
    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)
    ax2.spines['left'].set_linewidth(0.8)
    ax2.spines['bottom'].set_linewidth(0.8)

    ax2.legend(loc='upper right', fontsize=12, framealpha=0.94, edgecolor='#CCCCCC')

    # Caption
    caption_text2 = (
        f"Error bars = ±{SIGMA_LEVEL}σ measurement uncertainty."
    )

    plt.tight_layout(rect=[0, 0.075, 1, 0.96])
    fig2.text(0.5, 0.012, caption_text2, ha='center', va='bottom', fontsize=12,
              linespacing=1.25,
              bbox=dict(boxstyle='round,pad=0.35', facecolor='#F4F6F7',
                        edgecolor='#AAB7B8', linewidth=0.6))

    # Export
    out_png2 = os.path.join(OUTPUT_DIR, f'{CORR_BASE}.png')
    # out_pdf2 = os.path.join(OUTPUT_DIR, f'{CORR_BASE}.pdf')
    fig2.savefig(out_png2, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    # fig2.savefig(out_pdf2, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig2)
    print(f"  Saved: {out_png2}")
    # print(f"  Saved: {out_pdf2}")

    # =============================================================================
    # FIGURE 3: Correlation plot, no S02 (DARLING horizontal, Hi-STIFFS vertical)
    # =============================================================================
    print("Generating correlation figure (no S02)...")

    fig3, ax3 = plt.subplots(figsize=(7.8, 7.8), dpi=150)

    # Exclude stalk S02 from the correlation analysis
    mask = df_d['Stalk'] != 'S02'
    x_vals = df_d.loc[mask, 'Mean'].values
    y_vals = df_h.loc[mask, 'Mean'].values
    x_err = SIGMA_LEVEL * df_d.loc[mask, 'Std'].values
    y_err = SIGMA_LEVEL * df_h.loc[mask, 'Std'].values

    # Error bars
    ax3.errorbar(x_vals, y_vals, xerr=x_err, yerr=y_err,
                 fmt='none', ecolor='#7F8C8D', elinewidth=1.1,
                 capsize=2.2, capthick=0.9, alpha=0.65, zorder=1,
                 label='3σ Error Bar')

    # Scatter points (single color since only medium stalks)
    ax3.scatter(x_vals, y_vals, s=115, c='#2980B9',
                marker='o', edgecolors='white', linewidths=1.15,
                zorder=3, label='Stalk')

    # 1:1 reference line
    min_all = min(np.min(x_vals - x_err), np.min(y_vals - y_err))
    max_all = max(np.max(x_vals + x_err), np.max(y_vals + y_err))
    pad = 0.06 * (max_all - min_all)
    ax3.plot([min_all - pad, max_all + pad], [min_all - pad, max_all + pad],
             'k--', linewidth=1.6, alpha=0.75, label='1:1 line', zorder=2)

    # Linear regression
    from scipy.stats import linregress
    slope, intercept, r_val, p_val, se_slope = linregress(x_vals, y_vals)
    x_fit = np.linspace(min_all - pad, max_all + pad, 200)
    y_fit = slope * x_fit + intercept
    # ax3.plot(x_fit, y_fit, color='#D35400', linewidth=2.1, linestyle='-',
    #          label=f'Linear fit  (R² = {r_val**2:.3f})', zorder=2)

    # Highlight stalks where the 3σ uncertainty intervals of the two devices do not overlap
    # (i.e., the measurements are inconsistent even at the conservative 3σ level)
    unmarked = True
    incons_count = 0
    for i in range(len(df_h)):
        md = df_d.iloc[i]['Mean']
        sd = df_d.iloc[i]['Std']
        mh = df_h.iloc[i]['Mean']
        sh = df_h.iloc[i]['Std']

        d_low  = md - SIGMA_LEVEL * sd
        d_high = md + SIGMA_LEVEL * sd
        h_low  = mh - SIGMA_LEVEL * sh
        h_high = mh + SIGMA_LEVEL * sh

        if (h_high < d_low) or (d_high < h_low):
            incons_count += 1
            ax3.scatter(md, mh,
                        s=220,
                        c='red',
                        marker='x',
                        linewidths=2.8,
                        zorder=6,
                        label='Inconsistent at 3σ' if unmarked else None)
            unmarked = False

    # Labels, title, limits
    ax3.set_xlabel('DARLING Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Hi-STIFFS Flexural Stiffness (N·m²)', fontsize=14, fontweight='bold')
    ax3.set_title(
        'Correlation Between DARLING and Hi-STIFFS Measurements\n'
        'Lab 2026 - Artificial PVC Stalks',
        fontsize=14, fontweight='bold', pad=8
    )

    ax3.set_xlim(min_all - pad, max_all + pad)
    ax3.set_ylim(min_all - pad, max_all + pad)
    ax3.set_aspect('equal', adjustable='box')

    ax3.yaxis.grid(True, linestyle='--', alpha=0.35, linewidth=0.7)
    ax3.xaxis.grid(True, linestyle='--', alpha=0.35, linewidth=0.7)
    ax3.set_axisbelow(True)
    ax3.spines['top'].set_visible(False)
    ax3.spines['right'].set_visible(False)
    ax3.spines['left'].set_linewidth(0.8)
    ax3.spines['bottom'].set_linewidth(0.8)

    ax3.legend(loc='upper left', fontsize=12, framealpha=0.94, edgecolor='#CCCCCC')

    # Caption
    caption_text3 = (
        f"Error bars show ±{SIGMA_LEVEL}σ measurement uncertainty. If stalk's error bars do not\n"
        f"span 1:1 line, devices are inconsistent. {incons_count} of 16 are inconsistent."
    )

    plt.tight_layout(rect=[0, 0.075, 1, 0.96])
    fig3.text(0.5, 0.012, caption_text3, ha='center', va='bottom', fontsize=12,
              linespacing=1.25,
              bbox=dict(boxstyle='round,pad=0.35', facecolor='#F4F6F7',
                        edgecolor='#AAB7B8', linewidth=0.6))

    # Export
    out_png3 = os.path.join(OUTPUT_DIR, f'{CORR_BASE}_noS02.png')
    # out_pdf3 = os.path.join(OUTPUT_DIR, f'{CORR_BASE}_noS02.pdf')
    fig3.savefig(out_png3, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    # fig3.savefig(out_pdf3, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig3)
    print(f"  Saved: {out_png3}")
    # print(f"  Saved: {out_pdf3}")

    print(f"\nLab 2026 both-devices analysis complete.")
    print(f"  Hi-STIFFS median CV: {med_cv_h:.2f}%   |   DARLING median CV: {med_cv_d:.2f}%")


if __name__ == "__main__":
    # field_2025()
    # lab_2025()
    lab_2026()
    # lab_2026_both()