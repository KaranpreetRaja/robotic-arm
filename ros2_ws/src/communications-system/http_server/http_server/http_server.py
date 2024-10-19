import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading

class HttpNode(Node):

    def __init__(self):
        super().__init__('http_node')
        self.get_logger().info("HttpNode has been initialized")

        self.lock = threading.Lock()
        self.publishers = {}
        self.service_clients = {}

    def add_publisher(self, topic_name, message_type):
        with self.lock:
            self.publishers[topic_name] = self.create_publisher(message_type, topic_name, 10)

    def publish_message(self, topic_name, message):
        with self.lock:
            if topic_name in self.publishers:
                self.publishers[topic_name].publish(message)
            else:
                self.get_logger().warn("Publisher for topic %s not found", topic_name)
        
    
    def add_service_client(self, service_name, service_type):
        with self.lock:
            self.service_clients[service_name] = self.create_client(service_type, service_name)

    def call_service(self, service_name, request):
        with self.lock:
            if service_name in self.service_clients:
                future = self.service_clients[service_name].call_async(request)
                rclpy.spin_until_future_complete(self, future)
                return future.result()
            else:
                self.get_logger().warn("Service client for service %s not found", service_name)
                return None
        
        
# Initialize the ROS node
def init_ros_node():
    global node
    rclpy.init()
    node = HttpNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    app = FastAPI()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )

    # Start the ROS node in a separate thread
    thread = threading.Thread(target=init_ros_node)
    thread.start()

    @app.get("/test")
    async def read_root():
        return {"message": "Hello World"}

    uvicorn.run(app, host="127.0.0.1", port=8080)


    