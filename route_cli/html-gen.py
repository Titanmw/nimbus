import folium

# Waypoints im Format [lat, lon]
waypoints = [[48.21919, 16.44845], [48.21927, 16.44848], [48.21928, 16.44855], [48.21932, 16.44862], [48.2192, 16.44873], [48.21915, 16.44877], [48.21889, 16.44857], [48.21888, 16.44861], [48.21875, 16.44852], [48.21873, 16.44851], [48.21874, 16.44848], [48.21836, 16.44828], [48.21835, 16.44831], [48.21827, 16.44827], [48.2182, 16.44828], [48.21804, 16.44839], [48.21797, 16.44839], [48.21795, 16.44839], [48.21786, 16.44827], [48.21782, 16.44812], [48.21784, 16.44694], [48.21807, 16.44583], [48.21809, 16.44577], [48.21815, 16.4457], [48.21817, 16.44564], [48.21934, 16.44599], [48.22001, 16.44619], [48.22102, 16.44653], [48.22112, 16.44657], [48.22135, 16.44534]]

# Karte zentrieren auf ersten Punkt
m = folium.Map(location=waypoints[0], zoom_start=16)

# Route als Linie hinzufügen
folium.PolyLine(waypoints, color="blue", weight=4.5, opacity=0.8).add_to(m)

# Marker für Start/Ziel
folium.Marker(waypoints[0], popup="Start", icon=folium.Icon(color='green')).add_to(m)
folium.Marker(waypoints[-1], popup="Ziel", icon=folium.Icon(color='red')).add_to(m)

# Karte speichern
m.save("route_map.html")
print("✅ Karte gespeichert als 'route_map.html'")
