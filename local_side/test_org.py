import math
lat1, lon1 = 38.942092377471965, -92.31851601331908   # current position
lat2, lon2 = 38.942112017126064, -92.31966100770907   # target position
yaw = 90                                              # 0–360°

R = 6371000
phi1, phi2 = math.radians(lat1), math.radians(lat2)
d_phi     = math.radians(lat2 - lat1)
d_lambda  = math.radians(lon2 - lon1)

# Distance
a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lambda/2)**2
dist = R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

# Bearing
y = math.sin(d_lambda) * math.cos(phi2)
x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(d_lambda)
bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

# Angle difference
diff = ((bearing - yaw + 180) % 360) - 180
print(f"Distance: {dist:.2f} m, Bearing: {bearing:.2f}°, Yaw: {yaw:.2f}°, Angle Difference: {diff:.2f}°")
