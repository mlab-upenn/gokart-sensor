import pandas as pd
import pyproj

# Set up projections
proj_latlon = pyproj.Proj(proj='latlong',datum='WGS84')
proj_utm = pyproj.Proj(proj="utm", zone=16, datum='WGS84')

# Create a transformer
transformer = pyproj.Transformer.from_proj(proj_latlon, proj_utm)

# Use pandas to read the csv file
df = pd.read_csv('wp.csv')

# Create empty lists to store the new x, y UTM coordinates
x_list = []
y_list = []

# Loop over each row in the DataFrame
for index, row in df.iterrows():
    # Transform the lon, lat to UTM coordinates
    x, y = transformer.transform(row['lon'], row['lat'])
    x_list.append(x)
    y_list.append(y)

# Create new DataFrame containing the UTM coordinates
new_df = pd.DataFrame({'x': x_list, 'y': y_list})

# Save new DataFrame to a new CSV file
new_df.to_csv('wp_utm.csv', index=False)