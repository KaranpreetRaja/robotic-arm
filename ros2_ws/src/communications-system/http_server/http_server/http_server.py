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
        self._publishers = []
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
            publisher = self.create_publisher(message_type, topic_name, 10)
            self._publishers.append(publisher)  # Append to list

    def publish_message(self, topic_name, message):
        with self.lock:
            for publisher in self._publishers:
                if publisher.topic_name == topic_name:
                    msg = String()
                    msg.data = message
                    publisher.publish(msg)
                    return
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
            await self.websocket_client.send_text(json.dumps({"topic": topic, "message": data}))
        
# Initialize the ROS node
def init_ros_node():
    global node
    rclpy.init()
    node = HttpNode()
    node.init_subscribers()
    rclpy.spin(node)
    rclpy.shutdown()


def main(args=None):
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
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        node.websocket_client = websocket
        try:
            while True:
                data = await websocket.receive_text()
                message = json.loads(data)
                topic = message["topic"]
                message = message["message"]  
                
                if topic and message: 
                    if not any(publisher.topic_name == topic for publisher in node._publishers):
                        node.add_publisher(topic, String)
                    
                    node.publish_message(topic, message)

        except WebSocketDisconnect:
            node.websocket_client = None

    uvicorn.run(app, host="127.0.0.1", port=8080)

    rclpy.shutdown()

if __name__ == '__main__':
    main()