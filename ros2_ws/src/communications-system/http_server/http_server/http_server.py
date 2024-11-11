import asyncio
import json
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Depends, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
from std_msgs.msg import String

class HttpNode(Node):

    def __init__(self):
        super().__init__('http_node')
        self.get_logger().info("HttpNode has been initialized")

        self.lock = threading.Lock()
        self.publishers = {}
        self.service_clients = {}
        self.websocket_client = None
        self.subscribed_topics = [] # Add topics here
        
    def init_subscribers(self):
        subscribers = list()
        for topic in self.subscribed_topics:
            self.get_logger().info(f"subscribing to topic: {topic}")
            subscribers.append(
                self.create_subscription(
                    String,
                    topic,
                    lambda message, topic=topic: self.send_to_client(topic, message.data),
                    10,
                )
            )
        return subscribers
    
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
    
    async def send_to_client(self, topic, data):
        if self.websocket_client:
            self.websocket_client.send_text(json.dumps({"topic": topic, "message": data}))
        
# Initialize the ROS node
def init_ros_node():
    global node
    rclpy.init()
    node = HttpNode()
    node.init_subscribers
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
    
    @app.websocket("/ws")
    async def websocket_endpoint(websocket):
        await websocket.accept()
        node.websocket_client = websocket
        try:
            while True:
                data = await websocket.receive_text()
                message = json.loads(data)
                topic = message["topic"]
                message = message["message"]  
                
                if topic and message:   
                    node.publish_message(topic, message)

        except WebSocketDisconnect:
            node.websocket_client = None

    uvicorn.run(app, host="127.0.0.1", port=8080)


    