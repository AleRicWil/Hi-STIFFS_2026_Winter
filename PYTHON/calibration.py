# Standard libraries
import os
import csv
from pathlib import Path

# Installed packages
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector
import pandas as pd
from sklearn.linear_model import LinearRegression

# Workspace scripts
from process import HiSTIFFSData
from sensor_registry import SensorRegistry


def calculate_coefficients(summary_path, registry=None):
    """Calculate calibration coefficients using multiple linear regression to directly fit k, d, and c.

    Args:
        calibration_data (list): List of rows from the calibration summary CSV.
        cal_status_var (tk.StringVar): Tkinter variable to update the UI status.

    Returns:
        str: Formatted string of calculated coefficients.
    """
    registry = registry or SensorRegistry()
    sn = Path(summary_path).stem.split('_')[-1].zfill(3)    # serial number of sensor
    print(f'Parsed Serial#: {sn}')

    with open(summary_path, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        calibration_data = [row for row in reader]

    if not calibration_data:
        print("Status: No calibration data loaded")
        return ""

    try:
        # Extract and filter data, ensuring all columns are valid numbers
        valid_data = []
        for row in calibration_data:
            try:
                load = float(row[0])        # Newtons
                position = float(row[1])    # mm
                strain_1 = float(row[2])    # raw ADC count
                strain_2 = float(row[3])    # raw ADC count
                valid_data.append((load, position, strain_1, strain_2))
            except ValueError:
                continue

        if not valid_data:
            print("Status: No valid data for calculation")
            return ""

        # Unpack data
        loads, positions, strains_1, strains_2 = zip(*valid_data)
        loads = np.array(loads)
        positions = np.array(positions)
        strains_1 = np.array(strains_1)
        strains_2 = np.array(strains_2)

        # Match units
        positions = positions * 1e-3  # mm to m

        # Create design matrix A = [F*x, -F] for multiple linear regression
        A = np.column_stack((loads * positions, -loads))

        # Fit models for each strain type: V = c + k*(F*x) + beta2*(-F), where beta2 = k*d
        model_1 = LinearRegression().fit(A, strains_1)
        c_1 = model_1.intercept_
        k_1 = model_1.coef_[0]
        beta2_1 = model_1.coef_[1]
        d_1 = beta2_1 / k_1 if k_1 != 0 else 0

        model_2 = LinearRegression().fit(A, strains_2)
        c_2 = model_2.intercept_
        k_2 = model_2.coef_[0]
        beta2_2 = model_2.coef_[1]
        d_2 = beta2_2 / k_2 if k_2 != 0 else 0

        # Format result
        result = {f"{sn}": {'k1': k_1, 'd1': d_1, 'c1': c_1,
                            'k2': k_2, 'd2': d_2, 'c2': c_2}}
        print(result)
        registry = registry or SensorRegistry()
        registry.update_coeffs(sn, result[sn])

        # Predict strains
        strain_1_pred = model_1.predict(A)
        strain_2_pred = model_2.predict(A)

        # Compute R-squared for goodness of fit
        r2_1 = model_1.score(A, strains_1)
        r2_2 = model_2.score(A, strains_2)
        print(f"Channel 1 R-squared: {r2_1:.4f}")
        print(f"Channel 2 R-squared: {r2_2:.4f}")

        # Optional plotting: Measured vs Predicted for both channels on a single plot
        plt.figure(figsize=(8, 6))

        # Channel 1 scatter and fit line
        plt.scatter(strains_1, strain_1_pred, label='Channel 1 Data', color='green', alpha=0.7)
        parity_model_1 = LinearRegression().fit(strains_1.reshape(-1, 1), strain_1_pred)
        x_line_1 = np.linspace(min(strains_1), max(strains_1), 100)
        y_line_1 = parity_model_1.predict(x_line_1.reshape(-1, 1))
        plt.plot(x_line_1, y_line_1, color='green', linestyle='-', label='Channel 1 Fit')

        # Channel 2 scatter and fit line
        plt.scatter(strains_2, strain_2_pred, label='Channel 2 Data', color='red', alpha=0.7)
        parity_model_2 = LinearRegression().fit(strains_2.reshape(-1, 1), strain_2_pred)
        x_line_2 = np.linspace(min(strains_2), max(strains_2), 100)
        y_line_2 = parity_model_2.predict(x_line_2.reshape(-1, 1))
        plt.plot(x_line_2, y_line_2, color='red', linestyle='-', label='Channel 2 Fit')

        # Perfect fit line (y = x)
        all_measured = np.concatenate((strains_1, strains_2))
        all_predicted = np.concatenate((strain_1_pred, strain_2_pred))
        min_val = min(min(all_measured), min(all_predicted))
        max_val = max(max(all_measured), max(all_predicted))
        plt.plot([min_val, max_val], [min_val, max_val], 'k--', label='Perfect Fit (y = x)', alpha=0.5)

        plt.xlabel('Measured Strain (ADC Count)')
        plt.ylabel('Predicted Strain (ADC Count)')
        plt.title(f'Measured vs Predicted Strain\n(Channel 1 R² = {r2_1:.8f}, Channel 2 R² = {r2_2:.8f})')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        return result

    except Exception as e:
        print(f"Status: Error calculating coefficients: {str(e)}")
        return ""


def summary_from_dwells(collection_path):
    '''
    Generates summary CSV from a single raw data file containing multiple dwells 
    at different loads/positions.

    :param collection_path: file path of the raw data file containing multiple dwells
    '''
    # Load the data using HiSTIFFSData
    data = HiSTIFFSData(None, None, file_path=collection_path, debug=True)
    
    # Validate test type
    if data.test_type != 'Calibration':
        raise ValueError("Input file must be a calibration type collection.")
    
    # Extract sensor label (l) and serial number (sn)
    sn = data.sensor_sns[0]
    l = data.sensor_labels[0]
    loads = data.loads
    positions = data.positions

    if not sn or not l:
        raise ValueError("Failed to parse sensor serial number or label from metadata.")
    if not loads or not positions:
        raise ValueError("Failed to parse loads or positions from metadata.")
    
    # Generate dwell sequence: all loads at each position
    dwell_conditions = [(pos, load) for pos in positions for load in loads]
    num_dwells = len(dwell_conditions)
    if num_dwells == 0:
        raise ValueError("No dwell conditions parsed.")
    
    # Prepare data: filter channels
    data.filter_channels()
    s = data.data_dict[f'Sensor_{l}']
    time = s['time']
    strain_1_raw = s['strain_1_raw']
    strain_1_filter = s['strain_1_filter']
    strain_2_raw = s['strain_2_raw']
    strain_2_filter = s['strain_2_filter']
    
    # Set up interactive plot: 2 rows, 1 column, wide figure
    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Select Dwell Spans for Sensor {l} (SN: {sn}) - Press Enter to Confirm, 'r' to Reselect")
    
    # Plot Channel 1 (top)
    axs[0].plot(time, strain_1_raw, label='Raw', alpha=0.5, color='blue')
    axs[0].plot(time, strain_1_filter, label='Filtered', color='darkblue')
    axs[0].set_ylabel('Strain 1')
    axs[0].legend()
    axs[0].grid(True)
    
    # Plot Channel 2 (bottom)
    axs[1].plot(time, strain_2_raw, label='Raw', alpha=0.5, color='green')
    axs[1].plot(time, strain_2_filter, label='Filtered', color='darkgreen')
    axs[1].set_ylabel('Strain 2')
    axs[1].set_xlabel('Time (s)')
    axs[1].legend()
    axs[1].grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # List to store selected spans: [(min_t, max_t) for each dwell]
    selected_spans = [None] * num_dwells
    
    # Global variables for current selection
    current_min = None
    current_max = None
    current_shades = []  # List of axvspan artists for current selection
    
    def onselect(min_t, max_t):
        nonlocal current_min, current_max, current_shades
        # Remove previous shades
        for shade in current_shades:
            shade.remove()
        current_shades = []
        # Add new shades on both axes
        shade0 = axs[0].axvspan(min_t, max_t, color='yellow', alpha=0.3)
        shade1 = axs[1].axvspan(min_t, max_t, color='yellow', alpha=0.3)
        current_shades = [shade0, shade1]
        current_min = min_t
        current_max = max_t
        fig.canvas.draw()
    
    # Set up SpanSelector on the bottom axis (shared x)
    span = SpanSelector(axs[1], onselect, 'horizontal', useblit=True, props=dict(alpha=0.5, facecolor='red'))
    
    # Key press handler for confirmation or reselection
    confirmed = [False] * num_dwells
    def on_key(event):
        nonlocal current_min, current_max, current_shades
        if event.key == 'enter' and current_min is not None and current_max is not None:
            # Confirm current selection
            selected_spans[current_dwell_index] = (current_min, current_max)
            confirmed[current_dwell_index] = True
            # Change shade color to green for confirmed
            for shade in current_shades:
                shade.set_color('green')
                shade.set_alpha(0.5)
            fig.canvas.draw()
            # Clear current for next
            current_min = None
            current_max = None
            current_shades = []
        elif event.key == 'r':
            # Reselect: remove current shades
            for shade in current_shades:
                shade.remove()
            current_shades = []
            current_min = None
            current_max = None
            fig.canvas.draw()
    
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    plt.show(block=False)
    
    # Loop over each dwell
    for i, (pos, load) in enumerate(dwell_conditions):
        current_dwell_index = i
        print(f"\nSelect time span for position {pos} mm, load {load} N.")
        print("Drag on the plot to select, press 'Enter' to confirm, 'r' to reselect.")
        
        # Wait until confirmed
        while not confirmed[i]:
            plt.pause(0.1)  # Non-blocking pause to allow interaction
        
        print(f"Confirmed span: {selected_spans[i]}")
    
    # After all selections, close the plot
    plt.close(fig)
    
    # Compute averages using filtered data
    data_rows = []
    for i, (pos, load) in enumerate(dwell_conditions):
        min_t, max_t = selected_spans[i]
        if min_t is None or max_t is None or max_t - min_t < 1.0:
            print(f"Warning: Invalid or short span for dwell {i}. Skipping.")
            continue
        mask = (time >= min_t) & (time <= max_t)
        avg_1 = np.average(strain_1_filter[mask])
        avg_2 = np.average(strain_2_filter[mask])
        data_rows.append([load, pos, avg_1, avg_2])
    
    if not data_rows:
        print("No valid data rows collected.")
        return
    
    # Write summary CSV
    dir_path = os.path.dirname(collection_path)
    calibration_path = os.path.join(dir_path, f"calibration_{sn}.csv")
    headers = ["Load (N)", "Position (mm)", f"Strain_{l}1_avg", f"Strain_{l}2_avg"]
    
    with open(calibration_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        writer.writerows(data_rows)
    
    print(f"Summary CSV generated at: {calibration_path}")


def zero_shift_correction(collection_path):
    """
    Companion to summary_from_dwells().
    Loads a calibration raw data file, applies piecewise zero-shift correction
    based on user-selected later zero-load spans (one per channel independently).
    Each selected span's filtered average is subtracted from ALL data from that
    span onward → bringing the zero-load point to exactly zero.

    Overwrites the original CSV with corrected raw ADC values after each confirmed shift.
    """
    # Load data
    data = HiSTIFFSData(None, None, file_path=collection_path)
    if data.test_type != 'Calibration':
        raise ValueError("File must be a Calibration type collection.")

    # Parse sensor label and SN (always exactly one sensor for calibration)
    sn = data.sensor_sns[0]
    l = data.sensor_labels[0]
    # for row in data.header_rows:
        # if row and 'Sensor Serial#' in row[0]:
        #     parts = row[0].split(',')
        #     sn_part = parts[0].split(':')[-1].strip()
        #     l_part = parts[1].split(':')[-1].strip()
        #     sn = sn_part
        #     l = l_part
        #     break

    if not sn or not l:
        raise ValueError("Could not parse sensor SN and label from metadata.")

    print(f"Zero-shift correction for Sensor {l} (SN: {sn})")

    # Prepare data
    data.describe_channels()
    data.filter_channels()
    s = data.data_dict[f'Sensor_{l}']

    time = s['time']
    raw1 = s['strain_1_raw'].copy().astype(np.int32)
    raw2 = s['strain_2_raw'].copy().astype(np.int32)
    filt1 = s['strain_1_filter']
    filt2 = s['strain_2_filter']
    ini1 = s['ini_1']
    ini2 = s['ini_2']

    # Interactive plot – exactly as requested: two rows, one column, wide
    fig, axs = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    fig.suptitle(f"Zero-Shift Correction – Sensor {l} (SN {sn})\n"
                 f"Select later zero-load spans → Enter to confirm\n"
                 f"'r' = reselect current, 'q' = finish & exit", fontsize=12)

    # Channel 1
    line_raw1, = axs[0].plot(time, raw1, color='C0', alpha=0.4, lw=0.8, label='Raw')
    line_filt1, = axs[0].plot(time, filt1, color='C0', lw=1.6, label='Filtered')
    axs[0].axhline(ini1, color='red', linestyle='--', lw=1.2, label='Initial Zero')
    axs[0].set_ylabel(f'Strain {l}1 (raw ADC)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True, alpha=0.3)

    # Channel 2
    line_raw2, = axs[1].plot(time, raw2, color='C2', alpha=0.4, lw=0.8, label='Raw')
    line_filt2, = axs[1].plot(time, filt2, color='C2', lw=1.6, label='Filtered')
    axs[1].axhline(ini2, color='red', linestyle='--', lw=1.2, label='Initial Zero')
    axs[1].set_ylabel(f'Strain {l}2 (raw ADC)')
    axs[1].set_xlabel('Time (s)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Storage for corrections: list of (start_time, offset) per channel
    corrections1 = []
    corrections2 = []

    # Temporary current selection per channel
    cur1_min, cur1_max = None, None
    cur2_min, cur2_max = None, None
    shade1 = None
    shade2 = None

    def update_shade(ax, shade_ref, min_t, max_t, color='yellow'):
        nonlocal shade1, shade2
        if shade_ref is not None:
            shade_ref.remove()
        new_shade = ax.axvspan(min_t, max_t, color=color, alpha=0.35)
        if ax == axs[0]:
            shade1 = new_shade
        else:
            shade2 = new_shade
        fig.canvas.draw_idle()

    # SpanSelector for Channel 1
    def onselect1(min_t, max_t):
        nonlocal cur1_min, cur1_max
        cur1_min, cur1_max = min_t, max_t
        update_shade(axs[0], shade1, min_t, max_t)

    span1 = SpanSelector(axs[0], onselect1, 'horizontal', useblit=True,
                         props=dict(alpha=0.6, facecolor='red'))

    # SpanSelector for Channel 2
    def onselect2(min_t, max_t):
        nonlocal cur2_min, cur2_max
        cur2_min, cur2_max = min_t, max_t
        update_shade(axs[1], shade2, min_t, max_t)

    span2 = SpanSelector(axs[1], onselect2, 'horizontal', useblit=True,
                         props=dict(alpha=0.6, facecolor='red'))

    # Key handler
    def on_key(event):
        nonlocal cur1_min, cur1_max, cur2_min, cur2_max, shade1, shade2

        if event.key == 'enter':
            applied = False

            if cur1_min is not None and cur1_max is not None:
                mask = (time >= cur1_min) & (time <= cur1_max)
                offset = float(np.mean(filt1[mask]))
                corrections1.append((cur1_min, offset))
                print(f"  Ch1 correction @ {cur1_min:.3f}s → offset = {offset:+.1f} ADC")
                raw1[time >= cur1_min] -= int(round(offset))  # Apply immediately
                line_raw1.set_ydata(raw1)                     # Update plot

                update_shade(axs[0], shade1, cur1_min, cur1_max, color='lime')
                cur1_min = cur1_max = None
                applied = True

            if cur2_min is not None and cur2_max is not None:
                mask = (time >= cur2_min) & (time <= cur2_max)
                offset = float(np.mean(filt2[mask]))
                corrections2.append((cur2_min, offset))
                print(f"  Ch2 correction @ {cur2_min:.3f}s → offset = {offset:+.1f} ADC")
                raw2[time >= cur2_min] -= int(round(offset))  # Apply immediately
                line_raw2.set_ydata(raw2)                     # Update plot

                update_shade(axs[1], shade2, cur2_min, cur2_max, color='lime')
                cur2_min = cur2_max = None
                applied = True

            if applied:
                # Overwrite CSV immediately after apply
                df = pd.read_csv(collection_path, skiprows=data.data_index + 1)
                df[f'Strain_{l}1_raw'] = raw1
                df[f'Strain_{l}2_raw'] = raw2

                # Rebuild file: preserve metadata, replace data
                with open(collection_path, 'r') as f:
                    lines = f.readlines()

                data_start_idx = next(i for i, line in enumerate(lines)
                                      if '===BEGIN_DATA===' in line)
                header_block = lines[:data_start_idx + 2]  # Up to column header

                with open(collection_path, 'w', newline='') as f:
                    f.writelines(header_block)
                    df.to_csv(f, index=False, header=False)

                print("  Shift applied, plot updated, and CSV overwritten")

                fig.canvas.draw_idle()

        elif event.key == 'r':  # Reselect current (clears last selection)
            if shade1 is not None:
                shade1.remove()
                shade1 = None
            if shade2 is not None:
                shade2.remove()
                shade2 = None
            cur1_min = cur1_max = cur2_min = cur2_max = None
            fig.canvas.draw_idle()
            print("  Reselect enabled – drag again")

        elif event.key == 'q':  # Finish
            print("\nAll corrections applied and saved. Exiting...")
            plt.close(fig)
            return

    fig.canvas.mpl_connect('key_press_event', on_key)

    print("Interactive mode active.")
    print("   • Drag on top plot  → Channel 1 zero span")
    print("   • Drag on bottom plot → Channel 2 zero span")
    print("   • Press Enter after each selection (applies & saves)")
    print("   • Press 'r' to reselect current span")
    print("   • Press 'q' when finished → exit")

    plt.show(block=True)


if __name__ == "__main__":
    # run_calibration('A', '001', [10, 50], [60, 100])   
    old_paths = [r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163645_01.csv',
                 r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163754_01.csv',
                 r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163843_01.csv',
                 r'Hi-STIFFS_2026_Winter\Raw Data\2026-02-13\2026-02-13_163945_01.csv']
    # generate_summary(old_paths)

    # zero_shift_correction(r'Hi-STIFFS_2026_Winter\Raw Data\2026-03-13\2026-03-13_133242_01.csv')
    # summary_from_dwells(r'Hi-STIFFS_2026_Winter\Raw Data\2026-03-13\2026-03-13_133242_01.csv')
    calculate_coefficients(r'Hi-STIFFS_2026_Winter\Raw Data\2026-03-13\calibration_005.csv')

    
    