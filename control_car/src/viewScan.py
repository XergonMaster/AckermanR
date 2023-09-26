import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # Frecuencia en Hz para recibir mensajes
        )
        self.subscription  # Para prevenir que la suscripción sea eliminada automáticamente
        self.scan_data = []
        self.scan_angle_increment = 0.0  # Inicializar la variable de incremento de ángulo

    def scan_callback(self, msg):
        # Procesar los datos del escáner láser
        self.scan_data = msg.ranges
        self.scan_angle_increment = msg.angle_increment  # Obtener el ángulo de incremento

def main(args=None):
    rclpy.init(args=args)

    node = ScanSubscriber()

    plt.ion()  # Habilitar modo interactivo de Matplotlib

    fig = plt.figure(figsize=(15, 5))

    # Crear dos subplots: uno polar y otro rectangular
    ax_polar = fig.add_subplot(131, projection='polar')
    ax_scatter = fig.add_subplot(132)
    ax_info = fig.add_subplot(133)

    # Inicialmente, mostrar el gráfico polar
    current_subplot = ax_polar

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            
            if node.scan_data:
                current_subplot.clear()

                # Cambiar entre gráfico polar y gráfico de dispersión
                if current_subplot == ax_polar:
                    angles = np.linspace(0, 2 * np.pi, len(node.scan_data), endpoint=False)
                    current_subplot.plot(angles, node.scan_data, 'o', markersize=2)
                    current_subplot.set_title('Datos del escáner láser (Scatter)')
                    current_subplot.set_xlim(0, 2 * np.pi)
                    current_subplot.set_ylim(0, 10)
                    current_subplot.set_xlabel('Ángulo (radianes)')
                    current_subplot.set_ylabel('Distancia (metros)')
                    current_subplot = ax_scatter
                else:
                    angles = np.linspace(0, 2 * np.pi, len(node.scan_data), endpoint=False)
                    current_subplot.clear()
                    current_subplot.plot(angles, node.scan_data, 'o', markersize=2)
                    current_subplot.set_title('Datos del escáner láser (Polar)')
                    current_subplot = ax_polar

                # Imprimir información adicional en el subplot vacío
                total_data_points = len(node.scan_data)
                resolution_degrees = np.degrees(node.scan_angle_increment)
                info_text = f"Total de datos: {total_data_points}\nResolución del sensor en grados: {resolution_degrees}°\nÁngulos cada 45 grados:"
                for i in range(0, 360, 45):
                    angle_index = int(i / resolution_degrees)
                    if angle_index < total_data_points:
                        angle = angles[angle_index]
                        distance = node.scan_data[angle_index]
                        info_text += f"\n{np.degrees(angle)}°: {distance} metros"
                
                ax_info.clear()
                ax_info.text(0.1, 0.5, info_text, fontsize=10)
                ax_info.axis('off')

                plt.pause(0.01)  # Breve pausa para actualizar el gráfico dinámicamente

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
