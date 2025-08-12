import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
from rcl_interfaces.msg._set_parameters_result import SetParametersResult

class FaceDetecNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.service_ = self.create_service(FaceDetector,'/face_detector',self.detect_face_callback)
        self.declare_parameter('number_of_times_to_upsample',1)
        self.declare_parameter('model','hog')
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value
        self.default_image_path = get_package_share_directory('demo_python_service')+'/resource/default.jpg'
        self.get_logger().info(f'人脸识别服务已经启动')    
        self.add_on_set_parameters_callback(self.parameters_callback)
        # self.set_parameters([rclpy.Parameter('model',rclpy.Parameter.Type.STRING,'cnn')])
        

    def parameters_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f'{parameter.name}->{parameter.value}')
            if parameter.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = parameter.value
            if parameter.name == 'model':
                self.model = parameter.value
            return SetParametersResult(successful=True)

    def detect_face_callback(self,request,response):
        self.get_logger().info(f'detect函数被调用')
        if request.image.data:   #确保image内部是有内容的，有的话就转换为opencv格式，否则直接导入resource中的图像
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info(f'传入图为空，使用默认图像')
        start_time = time.time()
        self.get_logger().info(f'加载完图像，开始识别')
        #检测人脸
        face_locations = face_recognition.face_locations(cv_image, 
        number_of_times_to_upsample = self.number_of_times_to_upsample, model=self.model)
        end_time = time.time()
        response.use_time = end_time - start_time
        response.number = len(face_locations)
        for top,right,bottom,left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response  #必须返回给客户端

def main():
    rclpy.init()
    node = FaceDetecNode('Face_detection_node')
    rclpy.spin(node)
    rclpy.shutdown()