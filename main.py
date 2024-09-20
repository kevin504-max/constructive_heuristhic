import numpy as np
import gi as ox
import geopandas as gpd
from shapely.geometry import Point
from geopy.distance import geodesic
import matplotlib.pyplot as plt

# CONSTRUCTIVE HEURISTIC
# ================================================================================================================================================================================================================================================================================================================================================================

CITIES = [
    'Belo Horizonte, Brazil',
    'Uberlândia, Brazil',
    'Contagem, Brazil',
    'Juiz de Fora, Brazil',
    'Montes Claros, Brazil',
    'Governador Valadares, Brazil',
    'Uberaba, Brazil',
    'Ipatinga, Brazil',
    'Sete Lagoas, Brazil',
    'Divinópolis, Brazil',
    'Santa Luzia, Brazil',
    'Teófilo Otoni, Brazil',
    'Ribeirão das Neves, Brazil',
    'Sabará, Brazil',
    'Varginha, Brazil',
    'Betim, Brazil',
    'Pouso Alegre, Brazil',
    'Ibirité, Brazil',
    'Patos de Minas, Brazil',
    'Poços de Caldas, Brazil',
    'Conselheiro Lafaiete, Brazil',
    'Araguari, Brazil',
    'Itabira, Brazil',
    'Araxá, Brazil',
    'Passos, Brazil',
    'Ubá, Brazil',
    'Coronel Fabriciano, Brazil',
    'Barbacena, Brazil',
    'Ituiutaba, Brazil',
    'Muriaé, Brazil',
    'Pará de Minas, Brazil',
    'Lavras, Brazil',
    'Itajubá, Brazil',
    'Nova Serrana, Brazil',
    'Itaúna, Brazil',
    'Caratinga, Brazil',
    'Timóteo, Brazil',
    'João Monlevade, Brazil',
    'Unaí, Brazil',
    'Cataguases, Brazil',
    'Nova Lima, Brazil',
    'Itabirito, Brazil',
    'Manhuaçu, Brazil',
    'Iturama, Brazil',
    'São João del Rei, Brazil',
    'Alfenas, Brazil',
    'Ouro Preto, Brazil',
    'Viçosa, Brazil',
    'Paracatu, Brazil',
    'Mariana, Brazil',
    'Araxá, Brazil',
    'Ponte Nova, Brazil',
    'Leopoldina, Brazil',
    'Três Corações, Brazil',
    'Viçosa, Brazil',
    'Mariana, Brazil',
    'Lagoa Santa, Brazil',
    'Formiga, Brazil',
    'Igarapé, Brazil',
    'Itapecerica, Brazil',
]

def get_cities_in_minas_gerais():
    coordinates = []
    for city in CITIES:
        coords = get_city_coordinates(city)
        if coords:
            coordinates.append((city, coords))
    
    return coordinates

def get_city_coordinates(city_name):
    # Obtém a geometria da cidade
    gdf = ox.geocode_to_gdf(city_name)
    if not gdf.empty:
        # Reprojeta para um CRS geográfico (e.g., EPSG:4326)
        gdf = gdf.to_crs(epsg=4326)
        return gdf.geometry.centroid.y.values[0], gdf.geometry.centroid.x.values[0]
    else:
        return None

def distance_matrix(cities):
    # Retorna uma matriz de distâncias geodésicas
    n = len(cities)
    dist = np.zeros((n, n))
    for i in range(n):
        for j in range(i, n):
            dist[i][j] = dist[j][i] = geodesic(cities[i], cities[j]).kilometers
    return dist

def nearest_neighbor(dist_matrix, start):
    n = len(dist_matrix)
    unvisited = set(range(n))
    unvisited.remove(start)
    path = [start]
    total_distance = 0

    current_city = start
    while unvisited:
        next_city = min(unvisited, key=lambda city: dist_matrix[current_city][city])
        path.append(next_city)
        total_distance += dist_matrix[current_city][next_city]
        unvisited.remove(next_city)
        current_city = next_city

    total_distance += dist_matrix[current_city][start]  # Fechar o ciclo
    return path, total_distance

def solve_multiple_salesmen(cities, num_salesmen):
    dist_matrix = distance_matrix(cities)
    paths = []
    total_distances = []

    # Dividir as cidades de forma equilibrada entre os carteiros
    n = len(cities)
    city_indices = np.arange(n)
    np.random.shuffle(city_indices)  # Embaralhar para garantir uma distribuição aleatória
    
    # Dividir as cidades igualmente entre os carteiros
    city_chunks = np.array_split(city_indices, num_salesmen)

    for chunk in city_chunks:
        start = chunk[0]  # Seleciona a primeira cidade do conjunto como ponto de partida
        chunk_cities = [cities[i] for i in chunk]  # Subconjunto de cidades para o carteiro
        dist_matrix_chunk = distance_matrix(chunk_cities)  # Calcula a matriz de distâncias para o subconjunto
        path, total_dist = nearest_neighbor(dist_matrix_chunk, 0)  # Aplica o nearest neighbor para o subconjunto
        paths.append([chunk[i] for i in path])  # Converte o índice local para o índice global das cidades
        total_distances.append(total_dist)

    return paths, total_distances

# ================================================================================================================================================================================================================================================================================================================================================================
# REFINEMENT HEURISTIC
def two_opt(path, dist_matrix):
    improved = True
    best_path = path[:]
    best_distance = calculate_total_distance(best_path, dist_matrix)
    
    while improved:
        improved = False
        for i in range(1, len(path) - 2):
            for j in range(i + 1, len(path)):
                if j - i == 1:  # Não troca adjacentes
                    continue
                new_path = best_path[:]
                new_path[i:j] = best_path[j-1:i-1:-1]  # Inverte o segmento de i a j
                new_distance = calculate_total_distance(new_path, dist_matrix)
                if new_distance < best_distance:
                    best_path = new_path
                    best_distance = new_distance
                    improved = True
        path = best_path[:]
    
    return best_path, best_distance

def calculate_total_distance(path, dist_matrix):
    total_distance = 0
    for i in range(len(path) - 1):
        total_distance += dist_matrix[path[i]][path[i + 1]]
    total_distance += dist_matrix[path[-1]][path[0]]  # Fechar o ciclo
    return total_distance

def refine_routes(paths, dist_matrix):
    refined_paths = []
    refined_distances = []
    
    for path in paths:
        new_path, new_dist = two_opt(path, dist_matrix)
        refined_paths.append(new_path)
        refined_distances.append(new_dist)
    
    return refined_paths, refined_distances

def plot_paths(cities, paths, refined_paths=None):
    plt.figure(figsize=(10, 10))

    # Plotar as rotas originais
    if paths:
        for i, path in enumerate(paths):
            x_coords = [cities[city][1][1] for city in path] + [cities[path[0]][1][1]]  # Adiciona o ponto inicial ao final
            y_coords = [cities[city][1][0] for city in path] + [cities[path[0]][1][0]]
            
            plt.plot(x_coords, y_coords, marker='o', linestyle='dashed', markersize=8, label=f'Rota {i + 1} (Original)')

            for i in range(len(path) - 1):
                plt.annotate('', xy=(x_coords[i + 1], y_coords[i + 1]), xytext=(x_coords[i], y_coords[i]),
                             arrowprops=dict(arrowstyle='->', connectionstyle='arc3', color='red'))
    
    # Plotar as rotas refinadas
    if refined_paths:
        for i, path in enumerate(refined_paths):
            x_coords = [cities[city][1][1] for city in path] + [cities[path[0]][1][1]]  # Adiciona o ponto inicial ao final
            y_coords = [cities[city][1][0] for city in path] + [cities[path[0]][1][0]]
            
            plt.plot(x_coords, y_coords, marker='o', linestyle='dotted', markersize=8, label=f'Rota {i + 1} (Refinada)', linewidth=2)

            for i in range(len(path) - 1):
                plt.annotate('', xy=(x_coords[i + 1], y_coords[i + 1]), xytext=(x_coords[i], y_coords[i]),
                             arrowprops=dict(arrowstyle='->', connectionstyle='arc3', color='blue'))

    # Adicionar rótulos das cidades
    for name, (lat, lon) in cities:
        plt.text(lon, lat, f'{cities.index((name, (lat, lon))) + 1}', fontsize=12, ha='center', va='center')         

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Trajetórias Percorridas')
    plt.legend()  # Adiciona a legenda
    plt.grid(True)
    plt.show()

def plot_single_path(cities, path, refined_path=None, route_num=1):
    plt.figure(figsize=(10, 10))

    # Plotar a rota original
    if path:
        x_coords = [cities[city][1][1] for city in path] + [cities[path[0]][1][1]]  # Adiciona o ponto inicial ao final
        y_coords = [cities[city][1][0] for city in path] + [cities[path[0]][1][0]]

        plt.plot(x_coords, y_coords, marker='o', linestyle='dashed', markersize=8, label=f'Rota {route_num} (Original)')

        for i in range(len(path) - 1):
            plt.annotate('', xy=(x_coords[i + 1], y_coords[i + 1]), xytext=(x_coords[i], y_coords[i]),
                         arrowprops=dict(arrowstyle='->', connectionstyle='arc3', color='red'))

    # Plotar a rota refinada, se existir
    if refined_path:
        x_coords = [cities[city][1][1] for city in refined_path] + [cities[refined_path[0]][1][1]]
        y_coords = [cities[city][1][0] for city in refined_path] + [cities[refined_path[0]][1][0]]

        plt.plot(x_coords, y_coords, marker='o', linestyle='dotted', markersize=8, label=f'Rota {route_num} (Refinada)', linewidth=2)

        for i in range(len(refined_path) - 1):
            plt.annotate('', xy=(x_coords[i + 1], y_coords[i + 1]), xytext=(x_coords[i], y_coords[i]),
                         arrowprops=dict(arrowstyle='->', connectionstyle='arc3', color='blue'))

    # Adicionar rótulos das cidades
    for name, (lat, lon) in cities:
        plt.text(lon, lat, f'{cities.index((name, (lat, lon))) + 1}', fontsize=12, ha='center', va='center')

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(f'Trajetória da Rota {route_num}')
    plt.legend()  # Adiciona a legenda
    plt.grid(True)
    plt.show()

def plot_paths_one_by_one(cities, paths, refined_paths=None):
    # Plotar cada rota individualmente
    for i, path in enumerate(paths):
        refined_path = refined_paths[i] if refined_paths else None
        plot_single_path(cities, path, refined_path, route_num=i + 1)

# Exemplo de uso
cities = get_cities_in_minas_gerais()
print("Cidades em Minas Gerais:")
for city, _ in cities:
        print(cities.index((city, _)) + 1, ' - ', city.split(",")[0])

num_salesmen = 5
paths, total_distances = solve_multiple_salesmen([city[1] for city in cities], num_salesmen)
print("\nCaminhos:", paths)
print("Distâncias Totais:", total_distances)

# Refinar as rotas geradas pela heurística construtiva
refined_paths, refined_distances = refine_routes(paths, distance_matrix([city[1] for city in cities]))
print("\nCaminhos Refinados:", refined_paths)
print("Distâncias Refinadas:", refined_distances)

plot_paths_one_by_one(cities, paths, refined_paths)