from pure_pursuit import pure_pursuit_PID
from generate_path import Path
import matplotlib.pyplot as plt
import pandas as pd

# generate a path
path = Path()
generated_path = path.generate_path().copy()

# algorithm and plot
final_data = pure_pursuit_PID(generated_path)
path.plot_path(final_data)