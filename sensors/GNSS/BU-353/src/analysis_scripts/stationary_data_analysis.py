
# I have managed to write a small script to estimate two types of errors in the stationary GPS data (through northing and easting data)
# Mean Absolute Error (MAE) is the mean of absolute differences of values, with a known reference or a target value
# Root Mean Square Error is the root of average of squared differences, with a known reference or a target value

import pandas as pd
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
import numpy as np

df = pd.read_excel('/LAB1/src/analysis_scripts/ros2_stationary_data.xlsx')

nm = df['northing']
em = df['easting']

# Actual northing and easting corresponding to the loaction where I collected the stationary_data. 
northing_actual = 4688267.07
easting_actual = 326840.18

# Calculate Mean Absolute Error 

df['northing_error'] = abs(nm - northing_actual)
mean_abs_error_northing = (df['northing_error'].mean())

df['easting_error'] =  abs (em - easting_actual)
mean_abs_error_easting = (df['easting_error'].mean())

# Calculate Root Mean Square Error 

rmse_northing = np.sqrt(((nm - northing_actual ) ** 2).mean())
rmse_easting= np.sqrt(((em - easting_actual) ** 2).mean())

print('MEA of Northing is', mean_abs_error_northing)
print('MEA of Easting is', mean_abs_error_easting)
print('RMSE of Northing is', rmse_northing)
print('RMSE of Easting is', rmse_easting)