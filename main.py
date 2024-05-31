import tkinter as tk
from tkinter import filedialog, simpledialog
from xml.etree.ElementTree import Element, SubElement, ElementTree, parse
import math
import heapq
from shapely.geometry import Polygon, LineString, Point

# Класс для представления препятствия
class Obstacle:
    def __init__(self, points, impermeability):
        self.points = points
        self.impermeability = impermeability
        self.polygon = Polygon(points)

# Класс приложения
class App:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-3>", self.on_click)
        self.canvas.bind("<Double-1>", self.finish_obstacle)
        self.canvas.bind("<Button-1>", self.set_start_finish)
        self.obstacles = []
        self.current_obstacle = []
        self.start = None
        self.finish = None
        self.route = []
        self.route_length = 0
        self.route_time = 0
        self._create_menu()

    # Создание меню
    def _create_menu(self):
        menu_bar = tk.Menu(self.root)
        self.root.config(menu=menu_bar)

        file_menu = tk.Menu(menu_bar, tearoff=0)
        menu_bar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Map", command=self.save_map)
        file_menu.add_command(label="Load Map", command=self.load_map)
        file_menu.add_command(label="Save Route", command=self.save_route)

        run_menu = tk.Menu(menu_bar, tearoff=0)
        menu_bar.add_cascade(label="Run", menu=run_menu)
        run_menu.add_command(label="Find Route", command=self.find_route)

    # Обработка кликов мыши для добавления точек препятствий
    def on_click(self, event):
        if not self.current_obstacle:
            impermeability = simpledialog.askfloat("Input", "Enter impermeability (0-100): ", minvalue=0, maxvalue=100)
            if impermeability is None:
                return
            self.current_obstacle.append((event.x, event.y, impermeability))
        else:
            self.current_obstacle.append((event.x, event.y))
        self.canvas.create_line(event.x - 2, event.y - 2, event.x + 2, event.y + 2, fill="black")

    # Завершение создания препятствия двойным кликом
    def finish_obstacle(self, event):
        if len(self.current_obstacle) > 1:
            points = [p[:2] for p in self.current_obstacle]
            impermeability = self.current_obstacle[0][2]
            grey_value = int(255 * (1 - impermeability / 100))
            fill_color = f'#{grey_value:02x}{grey_value:02x}{grey_value:02x}'
            self.canvas.create_polygon(points, outline='black', fill=fill_color)
            self.obstacles.append(Obstacle(points, impermeability))
            self.current_obstacle = []

    # Установка начальной и конечной точек правым кликом мыши
    def set_start_finish(self, event):
        if not self.start:
            self.start = (event.x, event.y)
            self.canvas.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5, fill='green')
        elif not self.finish:
            self.finish = (event.x, event.y)
            self.canvas.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5, fill='red')
            self.find_route()

    # Сохранение карты в XML-файл
    def save_map(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".xml", filetypes=[("XML files", "*.xml"), ("All files", "*.*")])
        if not file_path:
            return

        root = Element('map')

        for obstacle in self.obstacles:
            obstacle_elem = SubElement(root, 'obstacle')
            points_elem = SubElement(obstacle_elem, 'points')
            for point in obstacle.points:
                point_elem = SubElement(points_elem, 'point')
                point_elem.set('x', str(point[0]))
                point_elem.set('y', str(point[1]))
            impermeability_elem = SubElement(obstacle_elem, 'impermeability')
            impermeability_elem.text = str(obstacle.impermeability)

        tree = ElementTree(root)
        tree.write(file_path)

    # Загрузка карты из XML-файла
    def load_map(self):
        file_path = filedialog.askopenfilename(filetypes=[("XML files", "*.xml"), ("All files", "*.*")])
        if not file_path:
            return

        tree = parse(file_path)
        root = tree.getroot()

        self.obstacles.clear()
        self.canvas.delete("all")

        for obstacle_elem in root.findall('obstacle'):
            points = []
            for point_elem in obstacle_elem.find('points').findall('point'):
                x = float(point_elem.get('x'))
                y = float(point_elem.get('y'))
                points.append((x, y))
            impermeability = float(obstacle_elem.find('impermeability').text)
            grey_value = int(255 * (1 - impermeability / 100))
            fill_color = f'#{grey_value:02x}{grey_value:02x}{grey_value:02x}'
            self.canvas.create_polygon(points, outline='black', fill=fill_color)
            self.obstacles.append(Obstacle(points, impermeability))

    # Сохранение маршрута в XML-файл
    def save_route(self):
        if not self.route:
            return

        file_path = filedialog.asksaveasfilename(defaultextension=".xml", filetypes=[("XML files", "*.xml"), ("All files", "*.*")])
        if not file_path:
            return

        root = Element('route')

        length_elem = SubElement(root, 'length')
        length_elem.text = str(self.route_length)

        time_elem = SubElement(root, 'time')
        time_elem.text = str(self.route_time)

        points_elem = SubElement(root, 'points')
        for point in self.route:
            point_elem = SubElement(points_elem, 'point')
            point_elem.set('x', str(point[0]))
            point_elem.set('y', str(point[1]))

        tree = ElementTree(root)
        tree.write(file_path)

    # Эвристическая функция для алгоритма A*
    def heuristic(self, a, b):
        return math.dist(a, b)

    # Реализация алгоритма A*
    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                if self.is_obstacle_free(current, neighbor):
                    tentative_g_score = g_score[current] + self.dist_between(current, neighbor) * (1 + self.get_speed_reduction((current, neighbor)) / 100)
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    # Получение соседних точек для текущей точки
    def get_neighbors(self, point):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx != 0 or dy != 0:
                    neighbor = (point[0] + dx, point[1] + dy)
                    if 0 <= neighbor[0] < self.canvas.winfo_width() and 0 <= neighbor[1] < self.canvas.winfo_height():
                        neighbors.append(neighbor)
        return neighbors

    # Вычисление расстояния между двумя точками
    def dist_between(self, a, b):
        return math.dist(a, b)

    # Восстановление пути из карты came_from
    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    # Поиск маршрута от начальной до конечной точки
    def find_route(self):
        if not self.start or not self.finish:
            return

        self.route = self.a_star(self.start, self.finish)
        if self.route:
            self.route_length = sum(self.dist_between(self.route[i], self.route[i + 1]) for i in range(len(self.route) - 1))
            self.route_time = self.calculate_route_time()
            self.display_route()
        else:
            print("No route found!")

    # Вычисление времени маршрута с учетом препятствий
    def calculate_route_time(self):
        total_time = 0
        for i in range(len(self.route) - 1):
            segment_length = self.dist_between(self.route[i], self.route[i + 1])
            speed_reduction = self.get_speed_reduction((self.route[i], self.route[i + 1]))
            if speed_reduction == 100:
                return float('inf')
            segment_time = segment_length / (1 - speed_reduction / 100)
            total_time += segment_time
        return total_time

    # Получение скорости прохождения отрезка с учетом препятствий
    def get_speed_reduction(self, segment):
        for obstacle in self.obstacles:
            if self.segment_intersects_obstacle(segment, obstacle):
                return obstacle.impermeability
        return 0

    # Проверка пересечения отрезка с препятствием
    def segment_intersects_obstacle(self, segment, obstacle):
        segment_line = LineString([segment[0], segment[1]])
        return segment_line.intersects(obstacle.polygon)

    # Проверка отсутствия препятствий на пути
    def is_obstacle_free(self, start, end):
        segment = (start, end)
        for obstacle in self.obstacles:
            if self.segment_intersects_obstacle(segment, obstacle):
                if obstacle.impermeability == 100:
                    return False
        return True

    # Отображение пути на карте
    def display_route(self):
        for i in range(len(self.route) - 1):
            self.canvas.create_line(self.route[i][0], self.route[i][1], self.route[i + 1][0], self.route[i + 1][1], fill="blue", width=2)

if __name__ == "__main__":
    app = App()
    app.root.mainloop()
