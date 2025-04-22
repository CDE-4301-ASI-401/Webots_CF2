import pandas as pd
df_drone_1 = pd.read_csv('/home/yanyew/webots_sim/path logs/CF11_drone_path.csv')
df_drone_2 = pd.read_csv('/home/yanyew/webots_sim/path logs/CF2_drone_path.csv')
df_drone_3 = pd.read_csv('/home/yanyew/webots_sim/path logs/CF3_drone_path.csv')

x1, y1 = df_drone_1['x'], df_drone_1['y']
x2, y2 = df_drone_2['x'], df_drone_2['y']
x3, y3 = df_drone_3['x'], df_drone_3['y']