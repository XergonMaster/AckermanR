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

    def scan_callback(self, data):
        # Obtén los ángulos (radianes) del escaneo
        angles = [data.angle_min + i * data.angle_increment for i in range(len(data.ranges))]
    
        # Obtén las distancias medidas
        distances = data.ranges

        # Crea un gráfico de dispersión (scatter plot)
        plt.figure(figsize=(8, 6))
        plt.scatter(angles, distances, s=1)
        plt.xlabel('Ángulo (radianes)')
        plt.ylabel('Distancia (metros)')
        plt.title('Escaneo láser')

        # Muestra el gráfico
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = ScanPlotter()
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()