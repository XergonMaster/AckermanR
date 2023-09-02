import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

class ScanPlotter(Node):
    def __init__(self):
        super().__init__('scan_plotter')
        # Suscribe al tópico /scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # Tamaño de la cola de mensajes
        )
        
        # Configura el gráfico inicial
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_xlabel('Ángulo (radianes)')
        self.ax.set_ylabel('Distancia (metros)')
        self.ax.set_title('Escaneo láser')
        plt.ion()  # Activa el modo interactivo
        plt.show()

    def scan_callback(self, data):
        # Obtén los ángulos (radianes) del escaneo
        angles = [data.angle_min + i * data.angle_increment for i in range(len(data.ranges))]
    
        # Obtén las distancias medidas
        distances = data.ranges
        
        # Limpia los datos anteriores y plotea los nuevos datos
        self.ax.clear()
        self.ax.scatter(angles, distances, s=1)
        self.ax.set_xlabel('Ángulo (radianes)')
        self.ax.set_ylabel('Distancia (metros)')
        self.ax.set_title('Escaneo láser')
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    plotter = ScanPlotter()
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()