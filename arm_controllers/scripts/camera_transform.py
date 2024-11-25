#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthRetriever:
    def __init__(self):
        rospy.init_node('depth_retriever', anonymous=True)

        # Parametri dei topic
        self.centers_input_topic = rospy.get_param('~centers_input_topic', '/detection/centers')
        self.position_output_topic = rospy.get_param('~position_output_topic', '/position_after_depth')
        self.depth_image_topic = rospy.get_param('~depth_image_topic', '/camera/depth/image_raw')
        self.depth_info_topic = rospy.get_param('~depth_info_topic', '/camera/depth/camera_info')

        # Publisher per le coordinate 3D
        self.position_pub = rospy.Publisher(self.position_output_topic, PointStamped, queue_size=10)

        # Inizializzazione dei sottoscrittori
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_K = None

        rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback)
        rospy.Subscriber(self.depth_info_topic, CameraInfo, self.depth_info_callback)
        rospy.Subscriber(self.centers_input_topic, PointStamped, self.centers_callback)

        rospy.loginfo("DepthRetriever inizializzato.")
        rospy.loginfo("Sottoscritto a: %s", self.centers_input_topic)
        rospy.loginfo("Sottoscritto alla mappa di profondità: %s", self.depth_image_topic)
        rospy.loginfo("Pubblica coordinate 3D su: %s", self.position_output_topic)

    def depth_image_callback(self, msg):
        try:
            # Converti il messaggio ROS Image in un'immagine OpenCV
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Assicurati che la profondità sia in metri
            if self.depth_image.dtype != np.float32:
                # Modifica questa linea in base all'unità di misura della tua mappa di profondità
                # Ad esempio, se la profondità è in millimetri:
                self.depth_image = self.depth_image.astype(np.float32) / 1000.0
        except CvBridgeError as e:
            rospy.logerr("Errore CvBridge durante la conversione della mappa di profondità: %s", e)

    def depth_info_callback(self, msg):
        if self.depth_K is None:
            self.depth_K = np.array(msg.K).reshape(3, 3)
            rospy.loginfo("Parametri intrinseci della camera di profondità ricevuti.")
            rospy.loginfo("K_depth: \n%s", self.depth_K)

    def centers_callback(self, msg):
        if self.depth_K is None:
            rospy.logwarn("Parametri intrinseci della camera di profondità non disponibili ancora.")
            return

        if self.depth_image is None:
            rospy.logwarn("Mappa di profondità non disponibile ancora.")
            return

        # Estrai x e y in metri dal messaggio
        x_m = msg.point.x  # In metri
        y_m = msg.point.y  # In metri

        # Converti x e y in pixel usando i parametri intrinseci della camera a colori
        # Assumendo che x_m = X / Z e y_m = Y / Z, ma Z è da determinare
        # Per ottenere u e v, abbiamo:
        # u = fx * X + cx = fx * (x_m * Z) + cx
        # v = fy * Y + cy = fy * (y_m * Z) + cy
        # Tuttavia, Z è la profondità che vogliamo ottenere dalla mappa di profondità

        # Invece, possiamo calcolare u e v basandoci sulla direzione (x_m, y_m)
        # e poi prendere z dalla mappa di profondità a quel pixel.

        # Supponiamo che (x_m, y_m) siano normalizzati rispetto a Z, quindi possiamo scegliere Z=1 per calcolare u, v
        # Poi usare u, v per prendere la vera profondità z dalla mappa di profondità

        # Calcolo u e v assumendo Z = 1
        u = self.depth_K[0, 0] * x_m + self.depth_K[0, 2]
        v = self.depth_K[1, 1] * y_m + self.depth_K[1, 2]

        # Converti in interi
        u_int = int(round(u))
        v_int = int(round(v))

        rospy.loginfo("Mapping (x_m, y_m) = (%.3f, %.3f) to pixel (u, v) = (%d, %d)", x_m, y_m, u_int, v_int)

        # Verifica che (u, v) siano all'interno della mappa di profondità
        height, width = self.depth_image.shape
        if u_int < 0 or u_int >= width or v_int < 0 or v_int >= height:
            rospy.logwarn("Coordinate pixel fuori dai limiti: u=%d, v=%d", u_int, v_int)
            return

        # Ottieni la profondità z dalla mappa di profondità
        z = self.depth_image[v_int, u_int]

        if np.isnan(z) or z <= 0:
            rospy.logwarn("Profondità non valida a pixel (%d, %d): z=%.3f", u_int, v_int, z)
            return

        rospy.loginfo("Profondità trovata a pixel (%d, %d): z=%.3f metri", u_int, v_int, z)

        # Crea il messaggio PointStamped con x, y originali e z calcolato
        point_3d = PointStamped()
        point_3d.header = msg.header
        point_3d.point.x = x_m
        point_3d.point.y = y_m
        point_3d.point.z = z

        # Pubblica il punto 3D
        self.position_pub.publish(point_3d)
        rospy.loginfo("Pubblicato punto 3D: x=%.3f, y=%.3f, z=%.3f metri", x_m, y_m, z)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        depth_retriever = DepthRetriever()
        depth_retriever.run()
    except rospy.ROSInterruptException:
        pass
