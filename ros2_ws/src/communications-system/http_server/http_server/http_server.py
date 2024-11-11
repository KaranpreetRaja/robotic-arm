import asyncio
import json
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Depends, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
from std_msgs.msg import String
from functools import partial

class HttpNode(Node):
    def __init__(self):
        super().__init__('http_node')
        self.get_logger().info("HttpNode has been initialized")
        
        self.lock = threading.Lock()
        self._publishers = []
        self.service_clients = {}
        self.websocket_client = None
        self.subscribed_topics = []  # Add topics here
        self.subscribers = []
        self._message_queue = asyncio.Queue()
        self._background_tasks = set()
        
    def init_subscribers(self):
        for topic in set(self.subscribed_topics):
            self.get_logger().info(f"subscribing to topic: {topic}")
            self.subscribers.append(
                self.create_subscription(
                    String,
                    topic,
                    partial(self.message_callback, topic=topic),
                    10,
                )
            )

    def message_callback(self, message, topic):
        """Synchronous callback that queues messages for async processing"""
        if self.websocket_client:
            self._message_queue.put_nowait({
                "topic": topic,
                "message": message.data
            })
    
    async def process_message_queue(self):
        """Async task to process queued messages"""
        while True:
            try:
                message = await self._message_queue.get()
                if self.websocket_client:
                    await self.websocket_client.send_text(json.dumps(message))
                self._message_queue.task_done()
            except Exception as e:
                self.get_logger().error(f"Error processing message: {str(e)}")
            await asyncio.sleep(0.01)  # Prevent CPU spinning
    
    def add_publisher(self, topic_name, message_type):
        with self.lock:
            publisher = self.create_publisher(message_type, topic_name, 10)
            self._publishers.append(publisher)

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

class NodeManager:
    def __init__(self):
        self.node = None

    def init_ros_node(self):
        rclpy.init()
        self.node = HttpNode()
        self.node.init_subscribers()
        rclpy.spin(self.node)
        rclpy.shutdown()

def create_app():
    node_manager = NodeManager()
    
    # Start the ROS node in a separate thread
    thread = threading.Thread(target=node_manager.init_ros_node)
    thread.daemon = True
    thread.start()
    
    app = FastAPI()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )

    @app.on_event("startup")
    async def startup_event():
        # Wait for node to be initialized
        while node_manager.node is None:
            await asyncio.sleep(0.1)
        # Start the message queue processor
        task = asyncio.create_task(node_manager.node.process_message_queue())
        node_manager.node._background_tasks.add(task)
        task.add_done_callback(node_manager.node._background_tasks.discard)

    @app.get("/test")
    async def read_root():
        return {"message": "Hello World"}
    
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        node_manager.node.websocket_client = websocket
        try:
            while True:
                data = await websocket.receive_text()
                message = json.loads(data)
                topic = message["topic"]
                message_content = message["message"]  
                
                if topic and message_content: 
                    if not any(publisher.topic_name == topic for publisher in node_manager.node._publishers):
                        node_manager.node.add_publisher(topic, String)
                    
                    node_manager.node.publish_message(topic, message_content)

        except WebSocketDisconnect:
            node_manager.node.websocket_client = None
    
    @app.post("/add_subscription")
    async def add_subscription(subscription: dict):
        topic = subscription["topic"]
        if topic not in node_manager.node.subscribed_topics:
            node_manager.node.subscribed_topics.append(topic)
            node_manager.node.subscribers.append(
                node_manager.node.create_subscription(
                    String,
                    topic,
                    partial(node_manager.node.message_callback, topic=topic),
                    10,
                )
            )
        return {"status": "success"}

    return app

def main(args=None):
    app = create_app()
    uvicorn.run(app, host="127.0.0.1", port=8080)

if __name__ == '__main__':
    main()