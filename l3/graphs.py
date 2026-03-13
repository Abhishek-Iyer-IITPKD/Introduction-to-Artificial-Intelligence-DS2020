import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# Measured data
ab_depths = np.array([3, 4, 5, 6, 7, 8])
ab_times = np.array([0.1249, 0.3423, 2.6021, 3.5372, 33.4538, 42.4144])

# Expectimax data including depth 7
expect_depths = np.array([3, 4, 5, 6, 7])
expect_times = np.array([0.0690, 0.4362, 3.0428, 21.7508, 157.8944])

def plot_linear_growth_final():
    # Adjusted figure size for better vertical visibility
    fig, ax = plt.subplots(figsize=(10, 7))

    # Plotting straight lines from point to point
    ax.plot(ab_depths, ab_times, label='Alpha-Beta Pruning', color='blue', linewidth=2, marker='o')
    ax.plot(expect_depths, expect_times, label='Expectimax', color='red', linewidth=2, marker='s')

    # Lab constraint markers
    ax.axhline(y=3, color='green', linestyle='--', alpha=0.6, label='3s Time Limit')
    ax.axhline(y=5, color='orange', linestyle='--', alpha=0.6, label='5s Time Limit')
    ax.axhline(y=10, color='magenta', linestyle='--', alpha=0.6, label='10s Time Limit')

    # Main Axis Formatting (X step 1, Y step 20)
    ax.set_title('Time vs. Search Depth', fontsize=14)
    ax.set_xlabel('Search Depth', fontsize=12)
    ax.set_ylabel('Execution Time (seconds)', fontsize=12)
    
    # Setting the ticks
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(20))
    
    ax.grid(True, which='both', linestyle=':', alpha=0.5)
    ax.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig('comparison.png')
    plt.show()

if __name__ == "__main__":
    plot_linear_growth_final()
