import math

def get_coordinates(anchor_lat, anchor_lon, x_distance, y_distance):
    # Convert latitude and longitude to radians
    anchor_lat = math.radians(anchor_lat)
    anchor_lon = math.radians(anchor_lon)
    
    # Earth radius in meters
    R = 6371000
    
    # Calculate new latitude and longitude using inverse Haversine formula
    new_lat = math.asin(math.sin(anchor_lat) * math.cos(y_distance/R) + 
                        math.cos(anchor_lat) * math.sin(y_distance/R) * math.cos(x_distance/R))
    new_lon = anchor_lon + math.atan2(math.sin(x_distance/R) * math.cos(anchor_lat), 
                                      math.cos(y_distance/R) - math.sin(anchor_lat) * math.sin(new_lat))
    
    # Convert new latitude and longitude back to degrees
    new_lat = math.degrees(new_lat)
    new_lon = math.degrees(new_lon)
    
    return new_lat, new_lon


anchor_lat = 23.01
anchor_lon = 121.11
x_distance = 500
y_distance = 500

object_lat, object_lon = get_coordinates(anchor_lat, anchor_lon, x_distance, y_distance)

print("Object coordinates: ({}, {})".format(object_lat, object_lon))