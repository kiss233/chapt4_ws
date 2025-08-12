import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import face_recognition
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType

class FaceDetecCilentNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.test1_image_path = get_package_share_directory('demo_python_service')+'/resource/test2.jpg'
        self.get_logger().info(f'人脸识别服务客户端已经启动')
        self.client = self.create_client(FaceDetector,'/face_detector')   #必须与服务端的命名一致才能收到！！！！！
        self.image = cv2.imread(self.test1_image_path)
    
    def call_set_parameters(self,parameters):   
        # 调用服务，修改参数值
        # 1.创建客户端，等待服务上线
        update_param_client = self.create_client(SetParameters,'/Face_detection_node/set_parameters')
        while update_param_client.wait_for_service(timeout_sec=1) is False:
            self.get_logger().info(f'等待参数更新服务上线！')
        #2.创建request
        request = SetParameters.Request()
        request.parameters = parameters      
        #3.调用服务器更新参数
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)  # 等待服务器返回响应
        response = future.result()
        return response
    
    def update_detect_model(self,model='hog'):
        """根据传入的model,构造Parameter,然后调用call_set_parameter更新服务端参数"""   
    # 1. 创建参数对象
        param = Parameter()
        param.name = 'model'
    #2. 创建参数值对象并赋值
        param_value = ParameterValue()
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING
        param.value = param_value
    #3.请求更新参数
        response = self.call_set_parameters([param])
        for result in response.results:
         self.get_logger().info(f'设置参数结果{result.successful}{result.reason}')     

    def send_request(self):
        #用于判断服务是否在线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'等待服务上线')
        #构造Request
        request = FaceDetector.Request()   
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        #发送请求并等待处理完成
        future = self.client.call_async(request)  #创建一个服务请求并异步获取结果，并没有立马包含处理结果，需要等待服务端处理完才会把结果放在future中
        rclpy.spin_until_future_complete(self,future)  #能够同时执行spin的同时检测future是否完成，如果完成则退出
        response = future.result()
        self.get_logger().info(f'接到响应：图像中共有{response.number}张人脸，耗时{response.use_time}')
        # self.show_response(response)
    
    def show_response(self,response1):  #显示图片
        self.get_logger().info(f'给图片添加识别框')
        for i in range(response1.number):
            top = response1.top[i]
            right = response1.right[i]
            left = response1.left[i]
            bottom = response1.bottom[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),2)
        cv2.imshow('Face Detection Result',self.image)
        cv2.waitKey(0)  #也是阻塞的,如果没点击关闭就会阻塞spin的运行


def main():
    rclpy.init()
    node = FaceDetecCilentNode('Face_detec_Client_node')
    node.update_detect_model('hog')
    node.send_request()
    # node.update_detect_model('cnn')
    # node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()